use swarm_loc::ekf::Localizer;
use swarm_loc::measurements::{ImuMeasurement, RangeMeasurement, VioMeasurement};
use swarm_loc::math::*;
use swarm_loc::state::*;

#[cfg(test)]
mod basic_types_tests {
    use swarm_loc::{math::{Matrix6x6, Quaternion}, state::{NodeState, Pose3D}};

    use super::*;

    #[test]
    fn test_pose3d_new() {
        let pos_v = Vector3::new(0.0, 0.0, -9.81);
        let vel_v = Vector3::new(15.0, 2.22, 0.01);
        let or_v = Quaternion { w: 12.2, x: 1.0, y: 1.0, z: 1.0 };
        let pose = Pose3D::new(
            pos_v,
            vel_v,
            or_v
        );

        assert_eq!(pose.position.x, pos_v.x);
        assert_eq!(pose.position.y, pos_v.y);
        assert_eq!(pose.position.z, pos_v.z);
        assert_eq!(pose.velocity.x, vel_v.x);
        assert_eq!(pose.velocity.y, vel_v.y);
        assert_eq!(pose.velocity.z, vel_v.z);
        assert_eq!(pose.orientation.w, or_v.w);
        assert_eq!(pose.orientation.x, or_v.x);
        assert_eq!(pose.orientation.y, or_v.y);
        assert_eq!(pose.orientation.z, or_v.z);
    }

    #[test]
    fn test_nodestate_new() {
        let pos_v = Vector3::new(0.0, 0.0, -9.81);
        let vel_v = Vector3::new(15.0, 2.22, 0.01);
        let or_v = Quaternion { w: 12.2, x: 1.0, y: 1.0, z: 1.0 };
        let pose = Pose3D::new(
            pos_v,
            vel_v,
            or_v
        );

        let covariance = Matrix6x6::identity();
        let timestamp_us = 1775128764;
        let node_id = 1;

        let ns = NodeState::new(
            node_id,
            pose,
            covariance.clone(),
            timestamp_us
        );

        assert_eq!(ns.node_id, node_id);
        assert_eq!(ns.timestamp_us, timestamp_us);
        assert_eq!(ns.covariance.data, covariance.data);
    }
}