use crate::math::{Matrix6x6, Vector3};

/// Raw IMU sample: accelerometer + gyroscope + integration timestep.
#[derive(Debug, Clone)]
pub struct ImuMeasurement {
    /// Body-frame acceleration (m/s²).
    pub accel: Vector3,
    /// Body-frame angular velocity (rad/s).
    pub gyro: Vector3,
    /// Integration timestep (s).
    pub dt: f64,
}

// ---------------------------------------------------------------------------

/// Visual-inertial odometry position fix.
#[derive(Debug, Clone)]
pub struct VioMeasurement {
    /// Estimated position in NED frame (m).
    pub position: Vector3,
    /// 6×6 measurement noise covariance.
    pub covariance: Matrix6x6,
    pub timestamp_us: u64,
}

// ---------------------------------------------------------------------------

/// UWB range measurement to a neighbor or anchor.
#[derive(Debug, Clone)]
pub struct RangeMeasurement {
    pub neighbor_id: u8,
    /// Measured distance (m).
    pub distance: f64,
    /// 1-sigma range noise (m).
    pub sigma: f64,
    pub timestamp_us: u64,
}
