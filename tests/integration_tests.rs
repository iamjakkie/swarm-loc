#[cfg(feature = "std")]
mod integration {
    use swarm_loc::ekf::Localizer;
    use swarm_loc::math::{Matrix6x6, Quaternion, Vector3};
    use swarm_loc::sim::{metrics, hover_trajectory, square_trajectory, simulate_imu, simulate_vio, simulate_ranges};
    use swarm_loc::state::Pose3D;

    #[test]
    fn test_hover() {
        let pose = Pose3D::new(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );

        let traj = hover_trajectory(pose.position, 1.0, 0.005);
        let imu = simulate_imu(&traj, 0.01, 0.001, 81);
        let vio = simulate_vio(&traj, 0.1, 0.0, 10.0, 81);

        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let mut vio_idx = 0;
        let mut estimated: Vec<(f64, Vector3)> = Vec::new();
        for i in 0..imu.len() {
            let (t, ref imu_meas) = imu[i];
            loc.predict(imu_meas);
            
            if vio_idx < vio.len() && vio[vio_idx].0 <= t {
                loc.update_vio(&vio[vio_idx].1);
                vio_idx +=1;
            }

            estimated.push((t, *loc.position()));
        }

        let truth: Vec<(f64, Vector3)> = imu.iter()
            .map(|(t,_)| (*t, traj[0].1.position))
            .collect();

        assert!(metrics::ate(&estimated, &truth) < 0.5)
    }
    // Issue #21: hover — ATE < 0.5m

    // Issue #22: square trajectory — EKF ATE < 2.0m, EKF ATE < VIO-only ATE

    // Issue #23: range anchors — ATE with ranges < ATE without ranges

    // Issue #24: NIS consistency — mean NIS between 2.0 and 12.0
}
