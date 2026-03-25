use swarm_loc::math::{Matrix3x3, Matrix6x6, Quaternion, Vector3};

// -- Vector3 ------------------------------------------------------------------

#[cfg(test)]
mod vector3_tests {
    use num_traits::Pow;

    use super::*;

    #[test]
    fn test_vector3_add() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);

        let sum = v1.add(&v2);
        assert_eq!(sum.x, 5.0);
        assert_eq!(sum.y, 7.0);
        assert_eq!(sum.z, 9.0);

        let v3 = Vector3::new(-4.0, -5.0, -6.0);

        let sum = v2.add(&v3);
        assert_eq!(sum.x, 0.0);
        assert_eq!(sum.y, 0.0);
        assert_eq!(sum.z, 0.0);
    }
    

    #[test]
    fn test_vector3_sub() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);

        let diff = v1.sub(&v2);
        assert_eq!(diff.x, -3.0);
        assert_eq!(diff.y, -3.0);
        assert_eq!(diff.z, -3.0);

        let v3 = Vector3::new(-4.0, -5.0, -6.0);

        let diff = v2.sub(&v3);
        assert_eq!(diff.x, 8.0);
        assert_eq!(diff.y, 10.0);
        assert_eq!(diff.z, 12.0);
    }

    #[test]
    fn test_vector3_scale() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let scaled = v.scale(2.0);
        assert_eq!(scaled.x, 2.0);
        assert_eq!(scaled.y, 4.0);
        assert_eq!(scaled.z, 6.0);

        let scaled = v.scale(0.0);
        assert_eq!(scaled.x, 0.0);
        assert_eq!(scaled.y, 0.0);
        assert_eq!(scaled.z, 0.0);

        let scaled = v.scale(1.0);
        assert_eq!(scaled.x, v.x);
        assert_eq!(scaled.y, v.y);
        assert_eq!(scaled.z, v.z);

        let scaled = v.scale(-1.0);
        assert_eq!(scaled.x, -v.x);
        assert_eq!(scaled.y, -v.y);
        assert_eq!(scaled.z, -v.z);
    }

    #[test]
    fn test_vector3_norm() {
        let v = Vector3::new(3.0, 4.0, 0.0);
        assert_eq!(v.norm(), 5.0);

        let zero = Vector3::new(0.0, 0.0, 0.0);
        assert_eq!(zero.norm(), 0.0);

        let unit = Vector3::new(1.0, 0.0, 0.0);
        assert_eq!(unit.norm(), 1.0);

        let neg = Vector3::new(-3.0, -4.0, 0.0);
        assert_eq!(v.norm(), 5.0);
    }

    #[test]
    fn test_vector3_dot() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        assert_eq!(v1.dot(&v2), 32.0);

        let u1 = Vector3::new(1.0, 0.0, 0.0);
        let u2 = Vector3::new(0.0, 1.0, 0.0);
        assert_eq!(u1.dot(&u2), 0.0);

        assert_eq!(v1.dot(&v1), v1.norm().powi(2));
    }

}

// -- Matrix3x3 ----------------------------------------------------------------

#[cfg(test)]
mod matrix3x3_tests {
    use numbrs::Matrix;
    use swarm_loc::math::{Matrix3x3, Vector3};

    #[test]
    fn test_matrix3x3_from_array() {
        let data = [1.0, 2.0, 3.0,
                1.0, 2.0, 3.0,
                1.0, 2.0, 3.0];
        let m = Matrix3x3::from_array(data);

        assert_eq!(m.data, data);
    }
    
    #[test]
    fn test_matrix3x3_identity() {
        let ind = Matrix3x3::identity();
        let ind_exp = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(ind.data, ind_exp);
    }

    #[test]
    fn test_matrix3x3_mul_vector() {
        let data = [1.0, 2.0, 3.0,
                1.0, 2.0, 3.0,
                1.0, 2.0, 3.0];
        let m = Matrix3x3::from_array(data);
        let v = Vector3::new(1.0, 2.0, 3.0);
        let ex_v = Vector3::new(14.0, 14.0, 14.0);
        
        let mul_v = m.mul_vector(&v);
        assert_eq!(mul_v.x, ex_v.x);
        assert_eq!(mul_v.y, ex_v.y);
        assert_eq!(mul_v.z, ex_v.z);

        let ind_m = Matrix3x3::identity();
        let mul_v = ind_m.mul_vector(&v);
        assert_eq!(mul_v.x, v.x);
        assert_eq!(mul_v.y, v.y);
        assert_eq!(mul_v.z, v.z);
    }

    #[test]
    fn test_matrix3x3_transpose() {
        let data = [5.0, 3.0, 2.0,
                            5.0, 4.0, 1.0,
                            2.0, 1.0, 0.0];
        let m = Matrix3x3::from_array(data);
        let ex_data = [5.0, 5.0, 2.0,
                            3.0, 4.0, 1.0,
                            2.0, 1.0, 0.0];
        
        assert_eq!(m.transpose().data, ex_data);

        assert_eq!(m.transpose().transpose().data, m.data);

        let ind_m = Matrix3x3::identity();
        assert_eq!(ind_m.transpose().data, ind_m.data);


    }



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
