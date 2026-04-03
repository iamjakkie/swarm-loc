use swarm_loc::math::*;

#[cfg(test)]
mod measurements_types {

    use swarm_loc::measurements::{ImuMeasurement, RangeMeasurement, VioMeasurement};

    use super::*;

    #[test]
    fn test_imu_measurement() {
        let accel = Vector3::new(10.0, 5.0, 2.5);
        let gyro = Vector3::new(110.0, 180.0, 360.0);
        let dt = 100.0;
        let timestamp_us = 1775128764;

        let imu = ImuMeasurement {
            accel,
            gyro,
            dt,
            timestamp_us
        };

        assert_eq!(imu.accel.x, accel.x);
        assert_eq!(imu.accel.y, accel.y);
        assert_eq!(imu.accel.z, accel.z);
        assert_eq!(imu.gyro.x, gyro.x);
        assert_eq!(imu.gyro.y, gyro.y);
        assert_eq!(imu.gyro.z, gyro.z);
        assert_eq!(imu.dt, dt);
        assert_eq!(imu.timestamp_us, timestamp_us);
    }

    #[test]
    fn test_vio_measurement() {
        let position = Vector3::new(0.0, 0.0, 10.0);
        let covariance = Matrix6x6::identity();
        let timestamp_us = 1775128764;

        let vio = VioMeasurement {
            position,
            covariance: covariance.clone(),
            timestamp_us
        };

        assert_eq!(vio.position.x, position.x);
        assert_eq!(vio.position.y, position.y);
        assert_eq!(vio.position.z, position.z);
        assert_eq!(vio.covariance.data, covariance.data);
        assert_eq!(vio.timestamp_us, timestamp_us);
    }

    #[test]
    fn test_range_measurement() {
        let neighbor_id = 0;
        let distance = 40.5;
        let sigma = 0.05;
        let timestamp_us = 1775128764;

        let range = RangeMeasurement {
            neighbor_id,
            distance,
            sigma,
            timestamp_us
        };

        assert_eq!(range.neighbor_id, neighbor_id);
        assert_eq!(range.distance, distance);
        assert_eq!(range.sigma, sigma);
        assert_eq!(range.timestamp_us, timestamp_us);
        
    }
}
