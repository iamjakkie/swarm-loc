extern crate std;

use std::vec::Vec;

use num_traits::Float;
use rand::{Rng, RngExt, SeedableRng};
use rand::rngs::SmallRng;

use crate::math::{Matrix6x6, Quaternion, Vector3};
use crate::measurements::{ImuMeasurement, RangeMeasurement, VioMeasurement};
use crate::state::Pose3D;


// ---------------------------------------------------------------------------
// Trajectory generators
// ---------------------------------------------------------------------------

/// Generates a stationary hover trajectory at `position` for `duration_s` seconds.
pub fn hover_trajectory(position: Vector3, duration_s: f64, dt: f64) -> Vec<(f64, Pose3D)> {
    let n_steps = (duration_s / dt).round() as usize;
    let mut traj: Vec<(f64, Pose3D)> = Vec::with_capacity(n_steps);
    let pose = Pose3D {
        position,
        velocity: Vector3 {x: 0.0, y: 0.0, z: 0.0},
        orientation: Quaternion::identity()
    };
    for i in 0..n_steps {
        traj.push((i as f64 * dt, pose.clone()));
    }

    traj
}

/// Generates a square trajectory (4 equal sides of `side_length` metres, at constant
/// `speed` m/s) at the given `altitude` in the NED down axis.
pub fn square_trajectory(
    side_length: f64,
    speed: f64,
    altitude: f64,
    dt: f64,
) -> Vec<(f64, Pose3D)> {
    let steps = (side_length / speed / dt).round() as usize;
    let mut position = Vector3 { x: 0.0, y: 0.0, z: altitude};
    let mut traj: Vec<(f64, Pose3D)> = Vec::with_capacity(steps);

    for i in 0..4 {
        for step in 0..steps {
            let global_step = i * steps + step;
            let t = global_step as f64 * dt;
            match i {
                0 => {
                    position.x = step as f64 * speed * dt;
                    let velocity = Vector3::new(speed, 0.0, 0.0);
                    let pose = Pose3D::new(position, velocity, Quaternion::identity());
                    traj.push((t, pose));
                },
                1 => {
                    position.y = step as f64 * speed * dt;
                    let velocity = Vector3::new(0.0, speed, 0.0);
                    let pose = Pose3D::new(position, velocity, Quaternion::identity());
                    traj.push((t, pose));
                },
                2 => {
                    position.x -= speed * dt;
                    let velocity = Vector3::new(-speed, 0.0, 0.0);
                    let pose = Pose3D::new(position, velocity, Quaternion::identity());
                    traj.push((t, pose));
                },
                3 => {
                    position.y -= speed * dt;
                    let velocity = Vector3::new(0.0, -speed, 0.0);
                    let pose = Pose3D::new(position, velocity, Quaternion::identity());
                    traj.push((t, pose));
                },
                _ => unreachable!()
            }
        }
    }

    traj
}

// ---------------------------------------------------------------------------
// Sensor simulators
// ---------------------------------------------------------------------------

/// Simulates IMU measurements from a ground-truth trajectory.
/// Adds zero-mean Gaussian noise with the given sigmas.
/// Seeded with `seed` for reproducibility.
pub fn simulate_imu(
    true_poses: &[(f64, Pose3D)],
    accel_noise_sigma: f64,
    gyro_noise_sigma: f64,
    seed: u64,
) -> Vec<(f64, ImuMeasurement)> {
    let mut rng = SmallRng::seed_from_u64(seed);

    let mut measurements = Vec::with_capacity(true_poses.len());
    for i in 0..true_poses.len() - 1 {
        let (t, pose) = &true_poses[i];
        let (t_next, next_pose) = &true_poses[i + 1];

        let dt = t_next - t;

        let u1: f64 = rng.random();
        let u2: f64 = rng.random();
        let noise_accel_x = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * accel_noise_sigma;
        let u1: f64 = rng.random();
        let u2: f64 = rng.random();
        let noise_accel_y = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * accel_noise_sigma;
        let u1: f64 = rng.random();
        let u2: f64 = rng.random();
        let noise_accel_z = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * accel_noise_sigma;
        
        let accel = Vector3 {
            x: (next_pose.velocity.x - pose.velocity.x) / dt + noise_accel_x,
            y: (next_pose.velocity.y - pose.velocity.y) / dt + noise_accel_y,
            z: (next_pose.velocity.z - pose.velocity.z) / dt + 9.81 + noise_accel_z,
        };

        let u1: f64 = rng.random();
        let u2: f64 = rng.random();
        let noise_gyro_x = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * gyro_noise_sigma;
        let u1: f64 = rng.random();
        let u2: f64 = rng.random();
        let noise_gyro_y = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * gyro_noise_sigma;
        let u1: f64 = rng.random();
        let u2: f64 = rng.random();
        let noise_gyro_z = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * gyro_noise_sigma;

        let gyro = Vector3 {
            x: (next_pose.orientation.x - pose.orientation.x) / dt + noise_gyro_x,
            y: (next_pose.orientation.y - pose.orientation.y) / dt + noise_gyro_y,
            z: (next_pose.orientation.z - pose.orientation.z) / dt + noise_gyro_z,
        };

        let imu = ImuMeasurement {
            accel,
            gyro,
            dt,
            timestamp_us: (*t * 1_000_000.0) as u64
        };

        measurements.push((*t, imu));
    }

    if let Some((t, _)) = true_poses.last() {
        measurements.push((*t, measurements.last().unwrap().1.clone()));
    }

    measurements
}

/// Simulates VIO position fixes from a ground-truth trajectory.
/// Includes Gaussian noise and an accumulating drift bias.
pub fn simulate_vio(
    true_poses: &[(f64, Pose3D)],
    position_noise_sigma: f64,
    drift_rate: f64,
    vio_rate_hz: f64,
    seed: u64,
) -> Vec<(f64, VioMeasurement)> {
    let mut rng = SmallRng::seed_from_u64(seed);

    let duration = true_poses.last().unwrap().0 - true_poses.first().unwrap().0;
    let samples = (vio_rate_hz * duration) as usize;

    let mut measurements: Vec::<(f64, VioMeasurement)> = Vec::with_capacity(samples);

    let vio_dt = 1.0 / vio_rate_hz;
    let mut next_vio_t = 0.0;

    let mut bias_x = 0.0;
    let mut bias_y = 0.0;
    let mut bias_z = 0.0;

    for (t, pose) in true_poses {
        if *t >= next_vio_t {
            next_vio_t += vio_dt;

            let u1: f64 = rng.random();
            let u2: f64 = rng.random();
            let noise_x = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * position_noise_sigma;
            let u1: f64 = rng.random();
            let u2: f64 = rng.random();
            let noise_y = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * position_noise_sigma;
            let u1: f64 = rng.random();
            let u2: f64 = rng.random();
            let noise_z = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * position_noise_sigma;

            let u1: f64 = rng.random();
            let u2: f64 = rng.random();
            bias_x += (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * drift_rate * vio_dt;
            let u1: f64 = rng.random();
            let u2: f64 = rng.random();
            bias_y += (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * drift_rate * vio_dt;
            let u1: f64 = rng.random();
            let u2: f64 = rng.random();
            bias_z += (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * drift_rate * vio_dt;
            
            let position = Vector3 {
                x: pose.position.x + noise_x + bias_x,
                y: pose.position.y + noise_y + bias_y,
                z: pose.position.z + noise_z + bias_z
            };

            let mut covariance = Matrix6x6::zeros();
            covariance.set(0, 0, position_noise_sigma * position_noise_sigma);
            covariance.set(1, 1, position_noise_sigma * position_noise_sigma);
            covariance.set(2, 2, position_noise_sigma * position_noise_sigma);

            let vio = VioMeasurement {
                position,
                covariance,
                timestamp_us: (*t * 1_000_000.0) as u64,
            };

            measurements.push((*t, vio));
        }

        
    }

    measurements
}

/// Simulates UWB range measurements from a ground-truth trajectory to fixed anchors.
pub fn simulate_ranges(
    true_poses: &[(f64, Pose3D)],
    anchors: &[(u8, Vector3)],
    range_noise_sigma: f64,
    range_rate_hz: f64,
    seed: u64,
) -> Vec<(f64, RangeMeasurement)> {
    let mut rng = SmallRng::seed_from_u64(seed);

    let duration = true_poses.last().unwrap().0 - true_poses.first().unwrap().0;
    let samples = (range_rate_hz * duration) as usize * anchors.len();

    let mut measurements: Vec::<(f64, RangeMeasurement)> = Vec::with_capacity(samples);

    let range_dt = 1.0 / range_rate_hz;
    let mut next_range_t = 0.0;

    for (t, pose) in true_poses {
        if *t >= next_range_t {
            next_range_t += range_dt;

            for (neighbor_id, position) in anchors {
                let neighbor_id = *neighbor_id;
                let u1: f64 = rng.random();
                let u2: f64 = rng.random();
                let noise = (Float::sqrt(-2.0 * Float::ln(u1)) * Float::cos(2.0 * std::f64::consts::PI * u2)) * range_noise_sigma;

                let distance = Float::sqrt(Float::powi(pose.position.x - position.x,2) + Float::powi(pose.position.y - position.y,2) + Float::powi(pose.position.z - position.z,2)) + noise;

                let range = RangeMeasurement {
                    neighbor_id,
                    distance,
                    sigma: range_noise_sigma,
                    timestamp_us: (*t * 1_000_000.0) as u64
                };

                measurements.push((*t, range));
            }

            
        }
    }

    measurements
}

// ---------------------------------------------------------------------------
// Evaluation metrics
// ---------------------------------------------------------------------------

pub mod metrics {
    use crate::math::Vector3;

    /// Absolute Trajectory Error: RMS of per-sample position errors.
    pub fn ate(estimated: &[(f64, Vector3)], truth: &[(f64, Vector3)]) -> f64 {
        todo!()
    }

    /// Relative Pose Error: RMS of relative translation errors between consecutive pairs.
    pub fn rpe(estimated: &[(f64, Vector3)], truth: &[(f64, Vector3)]) -> f64 {
        todo!()
    }

    /// Mean Normalised Innovation Squared across all update steps.
    pub fn nis_mean(innovations: &[(f64, f64)]) -> f64 {
        todo!()
    }
}
