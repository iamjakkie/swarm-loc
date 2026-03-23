use swarm_loc::math::{Matrix3x3, Matrix6x6, Quaternion, Vector3};

// -- Vector3 ------------------------------------------------------------------

#[cfg(test)]
mod vector3_tests {
    use super::*;

    #[test]
    fn test_vector3_add() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);

        let sum = v1.add(&v2);
        assert_eq!(sum.x, 5.0);
        assert_eq!(sum.y, 7.0);
        assert_eq!(sum.z, 9.0);
    }
    

    #[test]
    fn test_vector3_sub() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);

        let diff = v1.sub(&v2);
        assert_eq!(diff.x, -3.0);
        assert_eq!(diff.y, -3.0);
        assert_eq!(diff.z, -3.0);
    }

    #[test]
    fn test_vector3_scale() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let scaled = v.scale(2.0);
        assert_eq!(scaled.x, 2.0);
        assert_eq!(scaled.y, 4.0);
        assert_eq!(scaled.z, 6.0);
    }

    #[test]
    fn test_vector3_norm() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        assert_eq!(v.norm(), 5.0);
    }

    #[test]
    fn test_vector3_dot() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        assert_eq!(v1.dot(&v2), 32.0);
    }

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
