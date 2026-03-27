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
        assert_eq!(neg.norm(), 5.0);
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
    use super::*;

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
}

// -- Quaternion basic ---------------------------------------------------------

#[cfg(test)]
mod quaternion_basic_tests {
    use super::*;

    #[test]
    fn test_quaternion_identity() {
        let q = Quaternion::identity();

        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
    }

    #[test]
    fn test_quaternion_normalize() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let norm = q.normalize();

        assert!((norm.w - 0.18257).abs() < 0.0001);
        assert!((norm.x - 0.36515).abs() < 0.0001);
        assert!((norm.y - 0.54772).abs() < 0.0001);
        assert!((norm.z - 0.7303).abs() < 0.0001);

        let ind = Quaternion::identity();
        let norm = ind.normalize();

        assert_eq!(norm.w, ind.w);
        assert_eq!(norm.x, ind.x);
        assert_eq!(norm.y, ind.y);
        assert_eq!(norm.z, ind.z);
    }

    #[test]
    fn test_quaternion_conjugate() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let conj = q.conjugate();

        assert_eq!(conj.w, q.w);
        assert_eq!(conj.x, -q.x);
        assert_eq!(conj.y, -q.y);
        assert_eq!(conj.z, -q.z);

        let conj = q.conjugate().conjugate();

        assert_eq!(conj.w, q.w);
        assert_eq!(conj.x, q.x);
        assert_eq!(conj.y, q.y);
        assert_eq!(conj.z, q.z);
    }

    #[test]
    fn test_quaternion_multiply() {
        let q1 = Quaternion::new(1.0, 2.0, 3.0, 4.0);
        let q2 = Quaternion::new(4.0, 3.0, 2.0, 1.0);
        let res = q1.multiply(&q2);

        let ex_q = Quaternion::new(-12.0, 6.0, 24.0, 12.0);
        assert_eq!(res.w, ex_q.w);
        assert_eq!(res.x, ex_q.x);
        assert_eq!(res.y, ex_q.y);
        assert_eq!(res.z, ex_q.z);

        let ind = Quaternion::identity();
        let res = q2.multiply(&ind);

        assert_eq!(res.w, q2.w);
        assert_eq!(res.x, q2.x);
        assert_eq!(res.y, q2.y);
        assert_eq!(res.z, q2.z);

        let res = q2.normalize().multiply(&q2.normalize().conjugate());
        assert!(res.w - ind.w < 0.0001);
        assert!(res.x - ind.x < 0.0001);
        assert!(res.y - ind.y < 0.0001);
        assert!(res.z - ind.z < 0.0001);

    }

}

// -- Quaternion conversions ---------------------------------------------------

#[cfg(test)]
mod quaternion_conversion_tests {
    use super::*;

    fn assert_matrix3x3_approx(a: &Matrix3x3, b: &Matrix3x3, eps: f64) {
        for i in 0..9 {
            assert!((a.data[i] - b.data[i]).abs() < eps, 
                "data[{}]: expected {}, got {}", i, b.data[i], a.data[i]);
        }
    }

    #[test]
    fn test_quaternion_from_euler() {
        let (roll, pitch, yaw) = (0.2, 1.12, 2.31);
        let q = Quaternion::from_euler(roll, pitch, yaw);
        let ex = Quaternion::new(0.389, -0.449, 0.291, 0.750);

        println!("{:?}", q);
        assert!((q.w - ex.w).abs() < 0.001);
        assert!((q.x - ex.x).abs() < 0.001);
        assert!((q.y - ex.y).abs() < 0.001);
        assert!((q.z - ex.z).abs() < 0.001);
    }

    #[test]
    fn test_quaternion_rotation_matrix() {
        let q = Quaternion::new(0.389, -0.449, 0.291, 0.750);
        let m = q.to_rotation_matrix();

        let ex = Matrix3x3 { data: [-0.2935191, -0.8447469, -0.4474921, 
                            0.3219716, -0.5281199,  0.7857631,
                            -0.9001005,  0.0865567,  0.4269978]};
        
        assert_matrix3x3_approx(&m, &ex, 1e-3);
    }

    #[test]
    fn test_quaternion_rotate_vector() {
        let q = Quaternion::new(1.0, 0.0, 1.0, 0.0);
        let v = Vector3::new(1.0, 1.0, 1.0);
        let res = q.normalize().rotate_vector(&v);

        let ex = Vector3::new(1.0, 1.0, -1.0);
        assert!((res.x - ex.x).abs() < 0.001);
        assert!((res.y - ex.y).abs() < 0.001);
        assert!((res.z - ex.z).abs() < 0.001);
    }

    #[test]
    fn test_quaternion_from_axis_angle() {
        let angle = 1.5708;
        let axis = Vector3::new(1.0, 0.0, 0.0);
        let q = Quaternion::from_axis_angle(&axis, angle);

        let ex = Quaternion::new(0.7071, 0.7071, 0.0, 0.0);
        assert!((q.w - ex.w).abs() < 0.001);
        assert!((q.x - ex.x).abs() < 0.001);
        assert!((q.y - ex.y).abs() < 0.001);
        assert!((q.z - ex.z).abs() < 0.001);

        let zero = Vector3::new(0.0, 0.0, 0.0);
        let q = Quaternion::from_axis_angle(&zero, 1.0);
        assert_eq!(q.w, 1.0);
        assert_eq!(q.x, 0.0);
        assert_eq!(q.y, 0.0);
        assert_eq!(q.z, 0.0);
    }



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
