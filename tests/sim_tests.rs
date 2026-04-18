#![cfg(feature = "std")]
use swarm_loc::math::*;
use swarm_loc::sim::*;
use swarm_loc::sim::metrics::*;

#[cfg(test)]
mod sim_types {

    use super::*;

    #[test]
    fn test_hover_trajectory() {
        let traj = hover_trajectory(Vector3::new(1.0, 2.0, 3.0), 1.0, 0.005);

        assert_eq!(traj.len(), 200);
        assert_eq!(traj[0].1.position.x, 1.0);
        assert_eq!(traj[0].1.position.y, 2.0);
        assert_eq!(traj[0].1.position.z, 3.0);

        assert_eq!(traj[0].0, 0.0);
        assert!((traj[1].0 - 0.005).abs() < 1e-10);
        assert!((traj[199].0 - 0.995).abs() < 1e-10);


        for i in 1..traj.len() {
            assert!((traj[i].0 - traj[i-1].0 - 0.005).abs() < 1e-10);
            assert_eq!(traj[i].1.velocity.x, 0.0);
            assert_eq!(traj[i].1.velocity.y, 0.0);
            assert_eq!(traj[i].1.velocity.z, 0.0);
        }
    }

    #[test]
    fn test_square_trajectory() {
        let side = 10.0;
        let speed = 1.0;
        let altitude = 5.0;
        let dt = 0.01;
        let traj = square_trajectory(side, speed, altitude, dt);

        let steps = 4 * (side / speed / dt).round() as usize;

        assert_eq!(traj.len(), steps);

        for i in 0..steps {
            assert!((traj[i].1.position.z - altitude).abs() < 1e-6);
            if i == 0 {
                continue;
            }
            assert!((traj[i].0 - traj[i-1].0 - dt).abs() < 1e-6);
        }

        assert_eq!(traj[0].1.position.x, 0.0);
        assert_eq!(traj[0].1.position.y, 0.0);

        // corner 1
        assert!((traj[steps/4].1.position.x - 10.0).abs() < 0.1);
        assert!((traj[steps/4].1.position.y - 0.0).abs() < 0.1);
        assert!((traj[steps/4].0 - 10.0).abs() < 1e-6);
        // corner 2
        assert!((traj[steps/2].1.position.x - 10.0).abs() < 0.1);
        assert!((traj[steps/2].1.position.y - 10.0).abs() < 0.1);
        assert!((traj[steps/2].0 - 20.0).abs() < 1e-6);
        // corner 3
        assert!((traj[steps/4*3].1.position.x - 0.0).abs() < 0.1);
        assert!((traj[steps/4*3].1.position.y - 10.0).abs() < 0.1);
        // back to origin
        assert!((traj.last().unwrap().1.position.x).abs() < 0.1);
        assert!((traj.last().unwrap().1.position.y).abs() < 0.1);
    }

    #[test]
    fn test_imu_len() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let imu = simulate_imu(&traj, 0.0, 0.0, 81);

        assert_eq!(imu.len(), traj.len());
    }

    #[test]
    fn test_imu_zero_noise() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let imu = simulate_imu(&traj, 0.0, 0.0, 81);

        assert!((imu[0].1.accel.z - 9.81).abs() < 1e-6);
        assert!(imu[0].1.accel.x.abs() < 1e-6);
        assert!(imu[0].1.accel.y.abs() < 1e-6);
    }

    #[test]
    fn test_imu_deterministic() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let imu1 = simulate_imu(&traj, 0.1, 0.01, 81);
        let imu2 = simulate_imu(&traj, 0.1, 0.01, 81);

        assert_eq!(imu1[0].1.accel.x, imu2[0].1.accel.x);
        assert_eq!(imu1[99].1.accel.z, imu2[99].1.accel.z);
    }

    #[test]
    fn test_vio_rate() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let vio = simulate_vio(&traj, 0.0, 0.0, 10.0, 91);

        assert_eq!(vio.len(), 10);
    }

    #[test]
    fn test_vio_clean() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let vio = simulate_vio(&traj, 0.0, 0.0, 10.0, 91);

        assert!((vio[0].1.position.x - traj[0].1.position.x).abs() < 1e-6);

    }

    #[test]
    fn test_vio_drift() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let vio = simulate_vio(&traj, 0.0, 0.1, 10.0, 91);

        let err_early = vio[0].1.position.x.abs();
        let err_late = vio.last().unwrap().1.position.x.abs();

        assert!(err_late > err_early);
    }

    #[test]
    fn test_vio_deterministic() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let v1 = simulate_vio(&traj, 0.1, 0.0, 10.0, 42);
        let v2 = simulate_vio(&traj, 0.1, 0.0, 10.0, 42);
        assert_eq!(v1[0].1.position.x, v2[0].1.position.x);
    }

    #[test]
    fn test_ranges_output_rates() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let anchors = vec![(0u8, Vector3::new(10.0, 0.0, 0.0))];

        let ranges = simulate_ranges(&traj, &anchors, 0.0, 10.0, 42);
        assert_eq!(ranges.len(), 10); // 1s * 10Hz * 1 anchor

    }

    #[test]
    fn test_ranges_zero_noises() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let anchors = vec![(0u8, Vector3::new(10.0, 0.0, 0.0))];
        
        let ranges = simulate_ranges(&traj, &anchors, 0.0, 10.0, 42);
        assert!((ranges[0].1.distance - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_ranges_deterministic() {
        let traj = hover_trajectory(Vector3::new(0.0, 0.0, 0.0), 1.0, 0.005);
        let anchors = vec![(0u8, Vector3::new(10.0, 0.0, 0.0))];

        let r1 = simulate_ranges(&traj, &anchors, 0.1, 10.0, 42);
        let r2 = simulate_ranges(&traj, &anchors, 0.1, 10.0, 42);
        assert_eq!(r1[0].1.distance, r2[0].1.distance);
    }

    #[test]
    fn test_ate() {
        let truth = vec![
            (0.0, Vector3::new(0.0, 0.0, 0.0)),
            (0.1, Vector3::new(1.0, 0.0, 0.0)),
            (0.2, Vector3::new(2.0, 0.0, 0.0)),
        ];

        assert!((ate(&truth, &truth) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_ate_offset() {
        let truth = vec![
            (0.0, Vector3::new(0.0, 0.0, 0.0)),
            (0.1, Vector3::new(1.0, 0.0, 0.0)),
            (0.2, Vector3::new(2.0, 0.0, 0.0)),
        ];

        let offset = vec![
            (0.0, Vector3::new(1.0, 0.0, 0.0)),
            (0.1, Vector3::new(2.0, 0.0, 0.0)),
            (0.2, Vector3::new(3.0, 0.0, 0.0)),
        ];

        assert!((ate(&offset, &truth) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_rpe() {
        let truth = vec![
            (0.0, Vector3::new(0.0, 0.0, 0.0)),
            (0.1, Vector3::new(1.0, 0.0, 0.0)),
            (0.2, Vector3::new(2.0, 0.0, 0.0)),
        ];

        assert!((rpe(&truth, &truth) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_nis_zero() {
        let innovations = vec![(0.0, 1.0), (0.0, 1.0), (0.0, 2.0)];
        assert!((nis_mean(&innovations) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn test_nis_known() {
        let innovations = vec![(1.0, 1.0), (2.0, 4.0)];
        assert!((nis_mean(&innovations) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_nis_empty() {
        let innovations: Vec<(f64, f64)> = vec![];
        assert_eq!(nis_mean(&innovations), 0.0);
    }

}