use crate::math::{Matrix6x6, Quaternion, Vector3};

/// Position, velocity, and orientation of a node in the NED frame.
#[derive(Debug, Clone)]
pub struct Pose3D {
    pub position: Vector3,
    pub velocity: Vector3,
    pub orientation: Quaternion,
}

impl Pose3D {
    /// Creates a new `Pose3D`.
    pub fn new(position: Vector3, velocity: Vector3, orientation: Quaternion) -> Self {
        todo!()
    }
}

// ---------------------------------------------------------------------------

/// Full state of a single node: pose + covariance + metadata.
#[derive(Debug, Clone)]
pub struct NodeState {
    pub pose: Pose3D,
    pub covariance: Matrix6x6,
    pub timestamp_us: u64,
    pub node_id: u8,
}

impl NodeState {
    /// Creates a new `NodeState`.
    pub fn new(
        node_id: u8,
        pose: Pose3D,
        covariance: Matrix6x6,
        timestamp_us: u64,
    ) -> Self {
        todo!()
    }
}
