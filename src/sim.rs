extern crate std;

use std::vec::Vec;

use crate::math::{Quaternion, Vector3};
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

                },
                2 => {

                },
                3 => {

                }
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
    todo!()
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
    todo!()
}

/// Simulates UWB range measurements from a ground-truth trajectory to fixed anchors.
pub fn simulate_ranges(
    true_poses: &[(f64, Pose3D)],
    anchors: &[(u8, Vector3)],
    range_noise_sigma: f64,
    range_rate_hz: f64,
    seed: u64,
) -> Vec<(f64, RangeMeasurement)> {
    todo!()
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
