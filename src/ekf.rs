use crate::math::{Matrix6x6, Quaternion, Vector3};
use crate::measurements::{ImuMeasurement, RangeMeasurement, VioMeasurement};
use crate::state::{NodeState, Pose3D};

use num_traits::Float;

const GRAVITY: Vector3 = Vector3 { x: 0.0, y: 0.0, z: 9.81 };

/// EKF-based localizer for a single node.
#[derive(Debug, Clone)]
pub struct Localizer {
    state: NodeState,
    process_noise: Matrix6x6,
}

impl Localizer {
    /// Creates a new `Localizer` with the given initial state and process noise matrix Q.
    pub fn new(node_id: u8, initial_pose: Pose3D, initial_covariance: Matrix6x6, timestamp_us: u64, process_noise: Matrix6x6) -> Self {
        let state = NodeState::new(node_id, initial_pose, initial_covariance, timestamp_us);
        
        Self {
            state,
            process_noise
        }
    }

    /// IMU prediction step: propagates state and covariance forward by `imu.dt`.
    pub fn predict(&mut self, imu: &ImuMeasurement) {
        
        let a_ned = self.state.pose.orientation.rotate_vector(&imu.accel).sub(&GRAVITY);

        let velocity_new = self.state.pose.velocity.add(&a_ned.scale(imu.dt));
        let position_new = self.state.pose.position.add(&self.state.pose.velocity.scale(imu.dt)).add(&a_ned.scale(0.5 * imu.dt*imu.dt));

        let omega = imu.gyro;
        
        let orientation_new = if omega.norm() < 1e-10 {
            self.state.pose.orientation
        } else {
            let angle = omega.norm() * imu.dt;
            let axis = omega.scale(1.0/omega.norm());
            let dq = Quaternion::from_axis_angle(&axis, angle);
            self.orientation().multiply(&dq).normalize()
        };

        let mut f = Matrix6x6::identity();
        f.set(0, 3, imu.dt);
        f.set(1, 4, imu.dt);
        f.set(2, 5, imu.dt);

        let f_t = f.transpose();

        let p_new = f.mul(&self.state.covariance).mul(&f_t).add(&self.process_noise.scale(imu.dt));

        self.state.pose.position = position_new;
        self.state.pose.velocity = velocity_new;
        self.state.pose.orientation = orientation_new;
        self.state.covariance = p_new;
        self.state.timestamp_us = imu.timestamp_us;
    }

    /// VIO update step: fuses a position measurement into the filter.
    pub fn update_vio(&mut self, vio: &VioMeasurement) {
        // innovation: y = z - H * x
        let z = [vio.position.x, vio.position.y, vio.position.z, 0.0, 0.0, 0.0];

        let mut h = Matrix6x6::zeros();
        h.set(0,0, 1.0);
        h.set(1,1, 1.0);
        h.set(2,2, 1.0);

        let x = [
            self.state.pose.position.x,
            self.state.pose.position.y,
            self.state.pose.position.z,
            self.state.pose.velocity.x,
            self.state.pose.velocity.y,
            self.state.pose.velocity.z,
        ];

        let hx = h.mul_vector(&x);
        let y = [
            z[0] - hx[0],
            z[1] - hx[1],
            z[2] - hx[2],
            z[3] - hx[3],
            z[4] - hx[4],
            z[5] - hx[5],
        ];

        let h_t = h.transpose();

        // Innovation covariance 
        // S = H * P * Hᵀ + R

        let s = h.mul(&self.state.covariance).mul(&h_t).add(&vio.covariance);

        // Compute Kalman gain
        // K = P * Hᵀ * S⁻¹
        let s_inv = match s.inverse() {
            Some(inv) => inv,
            None => return,
        };
        let k = self.state.covariance.mul(&h_t).mul(&s_inv);

        // Update state
        // x_new = x + K * y
        
        let k_y = k.mul_vector(&y);
        let x_new = [
            x[0] + k_y[0],
            x[1] + k_y[1],
            x[2] + k_y[2],
            x[3] + k_y[3],
            x[4] + k_y[4],
            x[5] + k_y[5],
        ];

        self.state.pose.position = Vector3::new(x_new[0], x_new[1], x_new[2]);
        self.state.pose.velocity = Vector3::new(x_new[3], x_new[4], x_new[5]);

        // Update covariance
        // P_new = (I - K * H) * P

        let i = Matrix6x6::identity();
        let p_new = i.sub(&k.mul(&h)).mul(&self.state.covariance);
        self.state.covariance = p_new;
    }

    /// Range update step: fuses a distance measurement to a known anchor.
    /// Skips the update if the expected range to `anchor_position` is < 1e-6.
    pub fn update_range(&mut self, meas: &RangeMeasurement, anchor_position: &Vector3) {
        let dx = self.state.pose.position.x - anchor_position.x;
        let dy = self.state.pose.position.y - anchor_position.y;
        let dz = self.state.pose.position.z - anchor_position.z;

        let d_expected = (dx*dx + dy*dy + dz*dz).sqrt();

        if d_expected < 1e-6 {
            return;
        }
        // Calculate innovation
        let y = meas.distance - d_expected;

        // Jacobian H
        let h = [dx/d_expected, dy/d_expected, dz/d_expected, 0.0, 0.0, 0.0];

        // innovation covariance

        let mut s = 0.0;

        for i in 0..6 {
            for j in 0..6 {
                s += h[i] * &self.state.covariance.data[i * 6 + j] * h[j];
            }
        }

        s += meas.sigma * meas.sigma;

        // Kalman gain

        let mut k = self.state.covariance.mul_vector(&h);
        for i in 0..6 {
            k[i] /= s;
        }

        // Update state

        let x = [
            self.state.pose.position.x,
            self.state.pose.position.y,
            self.state.pose.position.z,
            self.state.pose.velocity.x,
            self.state.pose.velocity.y,
            self.state.pose.velocity.z,
        ];

        let x_new = [
            x[0] + k[0] * y,
            x[1] + k[1] * y,
            x[2] + k[2] * y,
            x[3] + k[3] * y,
            x[4] + k[4] * y,
            x[5] + k[5] * y,
        ];

        self.state.pose.position = Vector3::new(x_new[0], x_new[1], x_new[2]);
        self.state.pose.velocity = Vector3::new(x_new[3], x_new[4], x_new[5]);


        // Update covariance

        let i = Matrix6x6::identity();
        let mut data = [0.0; 36];
        for i in 0..6 {
            for j in 0..6 {
                data[i * 6 + j] = k[i] * h[j];
            }
        }
        let k_h = Matrix6x6{ data: data};
        let p_new = i.sub(&k_h).mul(&self.state.covariance);
        self.state.covariance = p_new;

    }

    /// Returns a reference to the current full node state.
    pub fn state(&self) -> &NodeState {
        &self.state
    }

    /// Returns a reference to the current position vector.
    pub fn position(&self) -> &Vector3 {
        &self.state.pose.position
    }

    /// Returns a reference to the current velocity vector.
    pub fn velocity(&self) -> &Vector3 {
        &self.state.pose.velocity
    }

    /// Returns a reference to the current orientation quaternion.
    pub fn orientation(&self) -> &Quaternion {
        &self.state.pose.orientation
    }

    /// Returns a reference to the current 6×6 covariance matrix.
    pub fn covariance(&self) -> &Matrix6x6 {
        &self.state.covariance
    }
}
