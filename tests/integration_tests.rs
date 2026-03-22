#[cfg(feature = "std")]
mod integration {
    use swarm_loc::ekf::Localizer;
    use swarm_loc::sim::{metrics, ImuSimulator, RangeSimulator, TrajectoryGenerator, VioSimulator};

    // Issue #21: hover — ATE < 0.5m

    // Issue #22: square trajectory — EKF ATE < 2.0m, EKF ATE < VIO-only ATE

    // Issue #23: range anchors — ATE with ranges < ATE without ranges

    // Issue #24: NIS consistency — mean NIS between 2.0 and 12.0
}
