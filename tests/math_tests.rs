use swarm_loc::math::{Matrix3x3, Matrix6x6, Quaternion, Vector3};

// -- Vector3 ------------------------------------------------------------------

#[cfg(test)]
mod vector3_tests {
    use super::*;

    // Issue #2: add, sub, scale, norm, dot
}

// -- Matrix3x3 ----------------------------------------------------------------

#[cfg(test)]
mod matrix3x3_tests {
    use super::*;

    // Issue #3: identity, mul_vector, transpose
}

// -- Quaternion basic ---------------------------------------------------------

#[cfg(test)]
mod quaternion_basic_tests {
    use super::*;

    // Issue #4: identity, normalize, conjugate, multiply
}

// -- Quaternion conversions ---------------------------------------------------

#[cfg(test)]
mod quaternion_conversion_tests {
    use super::*;

    // Issue #5: from_euler, to_rotation_matrix, rotate_vector, from_axis_angle
}

// -- Matrix6x6 basic ----------------------------------------------------------

#[cfg(test)]
mod matrix6x6_basic_tests {
    use super::*;

    // Issue #6: zeros, identity, from_diagonal, get/set, add, sub, scale, transpose
}

// -- Matrix6x6 multiplication -------------------------------------------------

#[cfg(test)]
mod matrix6x6_mul_tests {
    use super::*;

    // Issue #7: mul
}

// -- Matrix6x6 inverse --------------------------------------------------------

#[cfg(test)]
mod matrix6x6_inverse_tests {
    use super::*;

    // Issue #8: inverse, singular matrix returns None
}

// -- Matrix6x6 mul_vector -----------------------------------------------------

#[cfg(test)]
mod matrix6x6_mul_vector_tests {
    use super::*;

    // Issue #9: mul_vector
}
