use swarm_loc::ekf::Localizer;
use swarm_loc::measurements::{ImuMeasurement, RangeMeasurement, VioMeasurement};
use swarm_loc::math::Vector3;

// -- Localizer::new + predict -------------------------------------------------

#[cfg(test)]
mod predict_tests {
    use super::*;

    // Issue #12: zero IMU, constant accel, covariance growth, quaternion normalization
}

// -- Localizer::update_vio ----------------------------------------------------

#[cfg(test)]
mod update_vio_tests {
    use super::*;

    // Issue #13: perfect measurement, covariance shrinks, large R barely changes state
}

// -- Localizer::update_range --------------------------------------------------

#[cfg(test)]
mod update_range_tests {
    use super::*;

    // Issue #14: corrects toward anchor, large sigma, zero-distance edge case
}

// -- State accessors ----------------------------------------------------------

#[cfg(test)]
mod accessor_tests {
    use super::*;

    // Issue #15: state, position, velocity, orientation, covariance
}
