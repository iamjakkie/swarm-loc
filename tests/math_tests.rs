use swarm_loc::math::{Matrix3x3, Matrix6x6, Quaternion, Vector3};

fn make_matrix6x6(seed: u64) -> Matrix6x6 {
        let mut data = [0.0; 36];
        let mut s = seed;
        for i in 0..36 {
            s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            data[i] = (s >> 33) as f64 / 1e9;
        }
        Matrix6x6 { data }
    }

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

}

// -- Matrix6x6 basic ----------------------------------------------------------

#[cfg(test)]
mod matrix6x6_basic_tests {

    use super::*;

    #[test]
    fn test_matrix6x6_zeros() {
        let m = Matrix6x6::zeros();

        let exp = [0.0;36];
        assert_eq!(m.data, exp);
    }

    #[test]
    fn test_matrix6x6_identity() {
        let m = Matrix6x6::identity();

        let exp = [
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        ];

        assert_eq!(m.data, exp);
    }

    #[test]
    fn test_matrix6x6_from_diagonal() {
        let diag = [2.0, 3.0, 1.0, 0.0, 0.0, 5.0];
        let m = Matrix6x6::from_diagonal(&diag);

        let exp = [
            2.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 3.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 5.0
        ];

        assert_eq!(m.data, exp);
    }

    #[test]
    fn test_matrix6x6_get() {
        let mut m = Matrix6x6::identity();
        let x = m.get(2, 2);

        assert_eq!(x, 1.0);
    }

    #[test]
    #[should_panic]
    fn test_matrix6x6_get_row_out_of_bounds() {
        let m = Matrix6x6::identity();
        m.get(6,0);
    }

    #[test]
    #[should_panic]
    fn test_matrix6x6_get_col_out_of_bounds() {
        let m = Matrix6x6::identity();
        m.get(6,0);
    }

    #[test]
    fn test_matrix6x6_set_val() {
        let mut m = Matrix6x6::identity();
        m.set(0,1, 12.0);

        let exp = [
            1.0, 12.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0
        ];

        assert_eq!(m.data, exp);
    }

    #[test]
    #[should_panic]
    fn test_matrix6x6_set_row_out_of_bounds() {
        let mut m = Matrix6x6::identity();
        m.set(6,0, 1.0);
    }

    #[test]
    #[should_panic]
    fn test_matrix6x6_set_col_out_of_bounds() {
        let mut m = Matrix6x6::identity();
        m.set(6,0, 1.0);
    }

    #[test]
    fn test_matrix6x6_add() {
        let mut m1 = Matrix6x6::zeros();
        m1.set(0, 0, 10.0);
        m1.set(1, 0, 10.0);
        m1.set(2, 0, 10.0);
        m1.set(3, 0, 10.0);
        m1.set(4, 0, 10.0);
        m1.set(5, 0, 10.0);

        let mut m2 = Matrix6x6::zeros();
        m2.set(5, 0, 10.0);
        m2.set(5, 1, 10.0);
        m2.set(5, 2, 10.0);
        m2.set(5, 3, 10.0);
        m2.set(5, 4, 10.0);
        m2.set(5, 5, 10.0);

        let m3 = Matrix6x6::identity();

        let res = m1.add(&m2);
        let ex: Vec<f64> = m1.data.iter()
            .zip(m2.data.iter())
            .map(|(a, b)| a + b)
            .collect();

        assert_eq!(res.data, ex.as_slice());

        let res = m1.add(&m3);
        let ex: Vec<f64> = m1.data.iter()
            .zip(m3.data.iter())
            .map(|(a, b)| a + b)
            .collect();

        assert_eq!(res.data, ex.as_slice());

        let res = m2.add(&m3);
        let ex: Vec<f64> = m2.data.iter()
            .zip(m3.data.iter())
            .map(|(a, b)| a + b)
            .collect();

        assert_eq!(res.data, ex.as_slice());

        let zeros = Matrix6x6::zeros();

        let res = m1.add(&zeros);

        assert_eq!(m1.data, res.data);
    }

    #[test]
    fn test_matrix6x6_sub() {
        let mut m1 = Matrix6x6::zeros();
        m1.set(0, 0, 10.0);
        m1.set(1, 0, 10.0);
        m1.set(2, 0, 10.0);
        m1.set(3, 0, 10.0);
        m1.set(4, 0, 10.0);
        m1.set(5, 0, 10.0);

        let mut m2 = Matrix6x6::zeros();
        m2.set(5, 0, 10.0);
        m2.set(5, 1, 10.0);
        m2.set(5, 2, 10.0);
        m2.set(5, 3, 10.0);
        m2.set(5, 4, 10.0);
        m2.set(5, 5, 10.0);

        let m3 = Matrix6x6::identity();

        let res = m1.sub(&m2);
        let ex: Vec<f64> = m1.data.iter()
            .zip(m2.data.iter())
            .map(|(a, b)| a - b)
            .collect();

        assert_eq!(res.data, ex.as_slice());

        let res = m1.sub(&m3);
        let ex: Vec<f64> = m1.data.iter()
            .zip(m3.data.iter())
            .map(|(a, b)| a - b)
            .collect();

        assert_eq!(res.data, ex.as_slice());

        let res = m2.sub(&m3);
        let ex: Vec<f64> = m2.data.iter()
            .zip(m3.data.iter())
            .map(|(a, b)| a - b)
            .collect();

        assert_eq!(res.data, ex.as_slice());

        let zeros = Matrix6x6::zeros();

        let res = m1.sub(&m1);

        assert_eq!(res.data, zeros.data);
    }

    #[test]
    fn test_matrix6x6_scale() {
        let m = make_matrix6x6(81);
        let s = 2.0;

        let res = m.scale(s);
        let ex: Vec<f64> = m.data.iter()
            .map(|a| a * s)
            .collect();
        
        assert_eq!(res.data, ex.as_slice());

        let s = 0.0;

        let res = m.scale(s);
        let ex = Matrix6x6::zeros();

        assert_eq!(res.data, ex.data);

        let s = -1.0;
        
        let res = m.scale(s);
        let ex: Vec<f64> = m.data.iter()
            .map(|a| a * s)
            .collect();

        assert_eq!(res.data, ex.as_slice());

        let s = 1.0;

        let res = m.scale(s);

        assert_eq!(res.data, m.data);
    }

    #[test]
    fn test_matrix6x6_transpose() {
        let m = Matrix6x6 {
            data: [
                1.0, 3.0, 5.0, 7.0, 9.0, 11.0,
                2.0, 1.0, 3.0, 5.0, 7.0, 9.0,
                3.0, 4.0, 1.0, 3.0, 5.0, 7.0,
                4.0, 5.0, 6.0, 1.0, 3.0, 5.0,
                5.0, 6.0, 7.0, 8.0, 1.0, 3.0,
                6.0, 7.0, 8.0, 9.0, 10.0, 1.0
            ]
        };

        let res = m.transpose();
        let ex = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
            3.0, 1.0, 4.0, 5.0, 6.0, 7.0,
            5.0, 3.0, 1.0, 6.0, 7.0, 8.0,
            7.0, 5.0, 3.0, 1.0, 8.0, 9.0,
            9.0, 7.0, 5.0, 3.0, 1.0, 10.0,
            11.0, 9.0, 7.0, 5.0, 3.0, 1.0
        ];

        assert_eq!(res.data, ex);

        let res = m.transpose().transpose();

        assert_eq!(m.data, res.data);

        let i = Matrix6x6::identity();

        let res = i.transpose();

        assert_eq!(res.data, i.data);

    }

}

// -- Matrix6x6 multiplication -------------------------------------------------

#[cfg(test)]
mod matrix6x6_mul_tests {
    use super::*;

    #[test]
    fn test_matrix6x6_mul() {
        let A = make_matrix6x6(81);
        let i = Matrix6x6::identity();

        let res = A.mul(&i);

        assert_eq!(res.data, A.data);

        let res = i.mul(&A);

        assert_eq!(res.data, A.data);

        let zeros = Matrix6x6::zeros();

        let res = A.mul(&zeros);

        assert_eq!(res.data, zeros.data);

        let m1 = Matrix6x6 {
            data: [
                1.0, 3.0, 5.0, 7.0, 9.0, 11.0,
                2.0, 1.0, 3.0, 5.0, 7.0, 9.0,
                3.0, 4.0, 1.0, 3.0, 5.0, 7.0,
                4.0, 5.0, 6.0, 1.0, 3.0, 5.0,
                5.0, 6.0, 7.0, 8.0, 1.0, 3.0,
                6.0, 7.0, 8.0, 9.0, 10.0, 1.0
            ]
        };

        let m2 = Matrix6x6 {
            data: [
                1.0, 2.0, 3.0, 4.0, 5.0, 6.0,
                -3.0, 1.0, 4.0, 5.0, 6.0, 7.0,
                -5.0, -3.0, 1.0, 6.0, 7.0, 8.0,
                -7.0, -5.0, -3.0, 1.0, 8.0, 9.0,
                -9.0, -7.0, -5.0, -3.0, 1.0, 10.0,
                -11.0, -9.0, -7.0, -5.0, -3.0, 1.0
            ]
        };

        let res = m1.mul(&m2);

        let ex = [
            -284.0, -207.0, -123.0, -26.0, 90.0, 231.0, 
            -213.0, -159.0, -100.0, -30.0, 57.0, 167.0,
            -157.0, -106.0, -57.0, -9.0, 54.0, 138.0,
            -130.0, -76.0, -15.0, 44.0, 88.0, 151.0,
            -146.0, -79.0, -4.0, 82.0, 166.0, 213.0,
            -219.0, -129.0, -30.0, 81.0, 207.0, 331.0
        ];

        for i in 0..36 {
            assert!((res.data[i] - ex[i]).abs() < 1e-6,
                "Data[{}]: expected {}, got {}", i, ex[i], res.data[i]);
        }

        let res1 = A.mul(&m1).mul(&m2); // (A*B)*C
        let res2 = A.mul(&(m1.mul(&m2))); // A*(B*C)

        for i in 0..36 {
            assert!((res1.data[i] - res2.data[i]).abs() < 1e-6,
                "Data[{}]: expected {}, got {}", i, res1.data[i], res2.data[i]);
        }
    }

}

// -- Matrix6x6 inverse --------------------------------------------------------

#[cfg(test)]
mod matrix6x6_inverse_tests {
    use super::*;

    #[test]
    fn test_matrix6x6_inverse_identity() {
        let m = Matrix6x6::identity();
        let res = m.inverse().unwrap();

        assert_eq!(m.data, res.data);
    }

    #[test]
    fn test_matrix6x6_self_inverse() {
        let m = make_matrix6x6(81);

        let res = m.mul(&m.inverse().unwrap());
        let ex = Matrix6x6::identity();

        for i in 0..36 {
            assert!((res.data[i] - ex.data[i]).abs() < 1e-6,
                "Data[{}]: expected {}, got {}", i, ex.data[i], res.data[i]);
        }
    }

    #[test]
    fn test_matrix6x6_double_inverse() {
        let m = make_matrix6x6(81);

        let res = m.inverse().unwrap().inverse().unwrap();

        for i in 0..36 {
            assert!((res.data[i] - m.data[i]).abs() < 1e-6,
                "Data[{}]: expected {}, got {}", i, m.data[i], res.data[i]);
        }
    }

    #[test]
    fn test_matrix6x6_diagonal_inverse() {
        let mut m = Matrix6x6::identity();
        m.set(0, 0, 3.0);
        m.set(1, 1, -5.0);
        m.set(2,2, 2.0);

        let res = m.inverse().unwrap();

        let ex: Vec<f64> = m.data.iter().map(|a| if *a != 0.0 { 1.0/a } else { 0.0 }).collect();

        for i in 0..36 {
            assert!((res.data[i] - ex.as_slice()[i]).abs() < 1e-6,
                "Data[{}]: expected {}, got {}", i, m.data[i], ex.as_slice()[i]);
        }
    }

    #[test]
    fn test_matrix6x6_singular_inverse() {
        let mut m = Matrix6x6::zeros();

        let res = m.inverse();

        assert!(res.is_none());
    }

    #[test]
    fn test_matrix6x6_near_singular_inverse() {
        let m = Matrix6x6 {
            data: [2.0,  1.0,  0.0,  0.0,  0.0,  0.0,
                1.0,  3.0,  1.0,  0.0,  0.0,  0.0,
                0.0,  1.0,  4.0,  1.0,  0.0,  0.0,
                0.0,  0.0,  1.0,  3.0,  1.0,  0.0,
                0.0,  0.0,  0.0,  1.0,  2.0,  1.0,
                0.0,  0.0,  0.0,  1.0,  2.0,  1.0 + 1e-13]
        };

        let res = m.inverse();

        assert!(res.is_none())
    }

    #[test]
    fn test_matrix6x6_inverse() {
        let m = Matrix6x6 {
            data: [
                1.0, 3.0, 5.0, 7.0, 9.0, 11.0,
                2.0, 1.0, 3.0, 5.0, 7.0, 9.0,
                3.0, 4.0, 1.0, 3.0, 5.0, 7.0,
                4.0, 5.0, 6.0, 1.0, 3.0, 5.0,
                5.0, 6.0, 7.0, 8.0, 1.0, 3.0,
                6.0, 7.0, 8.0, 9.0, 10.0, 1.0
            ]
        };

        let res = m.inverse().unwrap();

        let ex = [
            -0.3735153735, 0.3901653902, 0.04262404262, 0.03552003552, 0.03108003108, 0.02797202797,
            0.2, -0.4, 0.2, 0.0, 0.0, 0.0,
            0.05714285714, 0.02857142857, -0.2285714286, 0.1428571429, 0.0, 0.0,
            0.0253968254, 0.0126984127, 0.009523809524, -0.1587301587, 0.1111111111, 0.0,
            0.01385281385, 0.006926406926, 0.005194805195, 0.004329004329, -0.1212121212,0.09090909091,
            0.01684981685, 0.04688644689, 0.03516483516, 0.0293040293, 0.02564102564, -0.07692307692
        ];

        for i in 0..36 {
            assert!((res.data[i] - ex.as_slice()[i]).abs() < 1e-6,
                "Data[{}]: expected {}, got {}", i, m.data[i], ex.as_slice()[i]);
        }
    }

    // Issue #8: inverse, singular matrix returns None
}

// -- Matrix6x6 mul_vector -----------------------------------------------------

#[cfg(test)]
mod matrix6x6_mul_vector_tests {
    use super::*;

    // Issue #9: mul_vector
}
