use swarm_loc::ekf::Localizer;
use swarm_loc::measurements::{ImuMeasurement, RangeMeasurement, VioMeasurement};
use swarm_loc::math::*;
use swarm_loc::state::*;

// -- Localizer::new + predict -------------------------------------------------

#[cfg(test)]
mod predict_tests {
    
    fn make_localizer() -> Localizer {
        let pose = Pose3D::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity())
    }

    use super::*;

    #[test]
    fn test_zero_IMU() {
        let mut loc = make_localizer();

        let imu = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.0, 0.0, 0.0),
            dt: 0.01,
            timestamp_us: 10_000,
        };

        loc.predict(&imu);

        assert!((loc.position().x).abs() < 1e-9);
        assert!((loc.position().y).abs() < 1e-9);
        assert!((loc.position().z).abs() < 1e-9);
        assert!((loc.velocity().x).abs() < 1e-9);
        assert!((loc.velocity().y).abs() < 1e-9);
        assert!((loc.velocity().z).abs() < 1e-9);
    }

    #[test]
    fn test_constant_accel() {
        let mut loc = make_localizer();

        let imu = ImuMeasurement {
            accel: Vector3::new(1.0, 0.0, 9.81),
            gyro: Vector3::new(0.0, 0.0, 0.0),
            dt: 1.0,
            timestamp_us: 10_000,
        };

        loc.predict(&imu);

        println!("{:?}", loc);

        assert_eq!(loc.position().x, 0.5);
        assert!((loc.position().y).abs() < 1e-9);
        assert!((loc.position().z).abs() < 1e-9);
        assert_eq!(loc.velocity().x, 1.0);
        assert!((loc.velocity().y).abs() < 1e-9);
        assert!((loc.velocity().z).abs() < 1e-9);
    }

    #[test]
    fn test_covariance_growth() {
        let mut loc = make_localizer();

        let p_before = loc.covariance().data;

        let imu = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.0, 0.0, 0.0),
            dt: 0.01,
            timestamp_us: 10_000,
        };

        loc.predict(&imu);

        for i in [0, 7, 14, 21, 28, 35] { // diagonal indices
            assert!(loc.covariance().data[i] > p_before[i]);
        }
    }

    #[test]
    fn test_zero_gyro_orientation() {
        let mut loc = make_localizer();

        let imu = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.0, 0.0, 0.0),
            dt: 0.01,
            timestamp_us: 10_000,
        };

        loc.predict(&imu);

        assert_eq!(loc.orientation().w, 1.0);
        assert_eq!(loc.orientation().x, 0.0);
        assert_eq!(loc.orientation().y, 0.0);
        assert_eq!(loc.orientation().z, 0.0);
    }

    #[test]
    fn test_1000_updates() {
        let mut loc = make_localizer();

        let imu = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.1, 0.0, 0.0),
            dt: 0.005,
            timestamp_us: 5_000,
        };

        for _ in 0..1000 {
            loc.predict(&imu);
        }

        assert!(loc.orientation().w.is_finite());
        assert!(loc.position().x.is_finite());
    }

    #[test]
    fn test_quaternion_stability() {
        let mut loc = make_localizer();

        let imu = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.1, 0.0, 0.0),
            dt: 0.005,
            timestamp_us: 5_000,
        };

        for _ in 0..1000 {
            loc.predict(&imu);
        }

        let norm = {
            let q = loc.orientation();
            (q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z).sqrt()
        };
        assert!((norm - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_gyro_change() {
        let mut loc = make_localizer();

        let imu = ImuMeasurement {
            accel: Vector3::new(0.0, 0.0, 9.81),
            gyro: Vector3::new(0.0, 0.0, 1.0), // yaw rotation
            dt: 0.01,
            timestamp_us: 10_000,
        };

        loc.predict(&imu);

        assert!(loc.orientation().w < 1.0); // no longer identity
    }

    #[test]
    fn test_1000_predicts_position() {
        let mut loc = make_localizer();

        let imu = ImuMeasurement {
            accel: Vector3::new(1.0, 0.0, 11.81),
            gyro: Vector3::new(0.0, 0.0, 0.0), // yaw rotation
            dt: 0.01,
            timestamp_us: 10_000,
        };

        for _ in 0..1000 {
            loc.predict(&imu);
        }

        assert!((loc.position().x - 50.0) < 1e-9);
        assert!((loc.position().y).abs() < 1e-9);
        assert!((loc.position().z - 100.0) < 1e-9);
        assert!((loc.velocity().x - 10.0) < 1e-9);
        assert!((loc.velocity().y).abs() < 1e-9);
        assert!((loc.velocity().z - 20.0) < 1e-9);

    }

    // Issue #12: zero IMU, constant accel, covariance growth, quaternion normalization
}

// -- Localizer::update_vio ----------------------------------------------------

#[cfg(test)]
mod update_vio_tests {
    use super::*;

    #[test]
    fn test_perfect_measurement() {

        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let mut r = Matrix6x6::zeros();
        r.set(0, 0, 1e-6);
        r.set(1, 1, 1e-6);
        r.set(2, 2, 1e-6);
        r.set(3, 3, 1e-6);
        r.set(4, 4, 1e-6);
        r.set(5, 5, 1e-6);
        let vio = VioMeasurement{
            position: Vector3::new(0.0, 0.0, 0.0),
            covariance: r,
            timestamp_us: 100
        };

        loc.update_vio(&vio);
        assert!(loc.position().x.abs() < 0.01);
    }

    #[test]
    fn test_covariance_shrink() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let mut r = Matrix6x6::zeros();
        r.set(0, 0, 1e-6);
        r.set(1, 1, 1e-6);
        r.set(2, 2, 1e-6);
        r.set(3, 3, 1e-6);
        r.set(4, 4, 1e-6);
        r.set(5, 5, 1e-6);
        let vio = VioMeasurement{
            position: Vector3::new(0.0, 0.0, 0.0),
            covariance: r,
            timestamp_us: 100
        };

        let p_before = loc.covariance().data;

        loc.update_vio(&vio);

        for i in [0, 7, 14] {
            assert!(loc.covariance().data[i] < p_before[i]);
        }
    }

    #[test]
    fn test_large_r() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let mut r = Matrix6x6::zeros();
        r.set(0, 0, 1e6);
        r.set(1, 1, 1e6);
        r.set(2, 2, 1e6);
        r.set(3, 3, 1e6);
        r.set(4, 4, 1e6);
        r.set(5, 5, 1e6);
        let vio = VioMeasurement{
            position: Vector3::new(0.0, 0.0, 0.0),
            covariance: r,
            timestamp_us: 100
        };

        loc.update_vio(&vio);

        assert!(loc.position().x > 9.9);
    }

    #[test]
    fn test_symmetric_covariance() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let mut r = Matrix6x6::zeros();
        r.set(0, 0, 1e6);
        r.set(1, 1, 1e6);
        r.set(2, 2, 1e6);
        r.set(3, 3, 1e6);
        r.set(4, 4, 1e6);
        r.set(5, 5, 1e6);
        let vio = VioMeasurement{
            position: Vector3::new(0.0, 0.0, 0.0),
            covariance: r,
            timestamp_us: 100
        };

        loc.update_vio(&vio);

        for i in 0..6 {
            for j in 0..6 {
                let a = loc.covariance().data[i * 6 + j];
                let b = loc.covariance().data[j * 6 + i];
                assert!((a-b).abs() < 1e-10, "P[{},{}]={} != P[{},{}]={}", i, j, a, j, i, b);
            }
        }
    }

    #[test]
    fn test_positive_semi_definite_covariance() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let mut r = Matrix6x6::zeros();
        r.set(0, 0, 1e6);
        r.set(1, 1, 1e6);
        r.set(2, 2, 1e6);
        r.set(3, 3, 1e6);
        r.set(4, 4, 1e6);
        r.set(5, 5, 1e6);
        let vio = VioMeasurement{
            position: Vector3::new(0.0, 0.0, 0.0),
            covariance: r,
            timestamp_us: 100
        };

        loc.update_vio(&vio);

        for i in [0, 7, 14, 21, 28, 35] {
            assert!(loc.covariance().data[i] >= 0.0, "diagonal[{}] is negative: {}", i, loc.covariance().data[i]);
        }
    }

    #[test]
    fn test_100_updates() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let mut r = Matrix6x6::zeros();
        r.set(0, 0, 1e6);
        r.set(1, 1, 1e6);
        r.set(2, 2, 1e6);
        r.set(3, 3, 1e6);
        r.set(4, 4, 1e6);
        r.set(5, 5, 1e6);
        let vio = VioMeasurement{
            position: Vector3::new(0.0, 0.0, 0.0),
            covariance: r,
            timestamp_us: 100
        };

        for _ in 0..100 {
            loc.update_vio(&vio);
        }

        for i in 0..36 {
            assert!(loc.covariance().data[i].is_finite(), "covariance[{}] is not finite", i);
        }
        for i in [0, 7, 14, 21, 28, 35] {
            assert!(loc.covariance().data[i] >= 0.0, "diagonal[{}] is negative", i);
        }
    }

}

// -- Localizer::update_range --------------------------------------------------

#[cfg(test)]
mod update_range_tests {
    use super::*;

    #[test]
    fn test_correction_toward_anchor() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let measurement = RangeMeasurement{
            neighbor_id: 0,
            distance: 8.0,
            sigma: 0.1,
            timestamp_us: 100,
        };

        let anchor = Vector3::new(0.0, 0.0, 0.0);

        loc.update_range(&measurement, &anchor);

        assert!(loc.position().x < 10.0);
    }

    #[test]
    fn test_large_sigma() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let measurement = RangeMeasurement{
            neighbor_id: 0,
            distance: 8.0,
            sigma: 1e6,
            timestamp_us: 100,
        };

        let anchor = Vector3::new(0.0, 0.0, 0.0);

        loc.update_range(&measurement, &anchor);

        assert!(loc.position().x > 9.9);
    }

    #[test]
    fn test_zero_distance() {
        let pose = Pose3D::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let anchor = Vector3::new(0.0, 0.0, 0.0);
        let meas = RangeMeasurement {
            neighbor_id: 0,
            distance: 0.0,
            sigma: 0.1,
            timestamp_us: 100,
        };

        loc.update_range(&meas, &anchor);

        assert_eq!(loc.position().x, 0.0);
    }

    #[test]
    fn test_covariance_shrink() {
        let pose = Pose3D::new(
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Quaternion::identity(),
        );
        let mut loc = Localizer::new(1, pose, Matrix6x6::identity(), 0, Matrix6x6::identity());

        let measurement = RangeMeasurement{
            neighbor_id: 0,
            distance: 10.0,
            sigma: 0.1,
            timestamp_us: 100,
        };

        let anchor = Vector3::new(0.0, 0.0, 0.0);

        let p_before = loc.covariance().data;

        loc.update_range(&measurement, &anchor);

        assert!(loc.covariance().data[0] < p_before[0]);
    }

    // Issue #14: corrects toward anchor, large sigma, zero-distance edge case
}

// -- State accessors ----------------------------------------------------------

#[cfg(test)]
mod accessor_tests {
    use super::*;

    // Issue #15: state, position, velocity, orientation, covariance
}
