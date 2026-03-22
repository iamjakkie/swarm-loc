use crate::math::{Matrix6x6, Quaternion, Vector3};
use crate::measurements::{ImuMeasurement, RangeMeasurement, VioMeasurement};
use crate::state::NodeState;

/// EKF-based localizer for a single node.
#[derive(Debug, Clone)]
pub struct Localizer {
    state: NodeState,
    process_noise: Matrix6x6,
}

impl Localizer {
    /// Creates a new `Localizer` with the given initial state and process noise matrix Q.
    pub fn new(node_id: u8, initial_state: NodeState, process_noise: Matrix6x6) -> Self {
        todo!()
    }

    /// IMU prediction step: propagates state and covariance forward by `imu.dt`.
    pub fn predict(&mut self, imu: &ImuMeasurement) {
        todo!()
    }

    /// VIO update step: fuses a position measurement into the filter.
    pub fn update_vio(&mut self, meas: &VioMeasurement) {
        todo!()
    }

    /// Range update step: fuses a distance measurement to a known anchor.
    /// Skips the update if the expected range to `anchor_position` is < 1e-6.
    pub fn update_range(&mut self, meas: &RangeMeasurement, anchor_position: &Vector3) {
        todo!()
    }

    /// Returns a reference to the current full node state.
    pub fn state(&self) -> &NodeState {
        todo!()
    }

    /// Returns a reference to the current position vector.
    pub fn position(&self) -> &Vector3 {
        todo!()
    }

    /// Returns a reference to the current velocity vector.
    pub fn velocity(&self) -> &Vector3 {
        todo!()
    }

    /// Returns a reference to the current orientation quaternion.
    pub fn orientation(&self) -> &Quaternion {
        todo!()
    }

    /// Returns a reference to the current 6×6 covariance matrix.
    pub fn covariance(&self) -> &Matrix6x6 {
        todo!()
    }
}
