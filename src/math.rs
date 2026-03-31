use num_traits::Float;

/// 3-dimensional vector (f64 components).
#[derive(Debug, Clone, Copy)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3 {
    /// Creates a new `Vector3`.
    ///
    /// # Examples
    /// ```
    /// use swarm_loc::math::Vector3;
    /// let v = Vector3::new(1.0, 2.0, 3.0);
    /// ```
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            x,
            y,
            z,
        }
    }

    /// Returns `self + other`.
    pub fn add(&self, other: &Vector3) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    /// Returns `self - other`.
    pub fn sub(&self, other: &Vector3) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z
        }
    }

    /// Returns each component multiplied by `s`.
    pub fn scale(&self, s: f64) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s
        }
    }

    /// Returns the Euclidean norm.
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Returns the dot product of `self` and `other`.
    pub fn dot(&self, other: &Vector3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

// ---------------------------------------------------------------------------

/// 3×3 matrix stored row-major as `[f64; 9]`.
#[derive(Debug, Clone, Copy)]
pub struct Matrix3x3 {
    pub data: [f64; 9],
}

impl Matrix3x3 {
    pub fn from_array(data: [f64;9]) -> Self {
        Self {
            data
        }
    }

    /// Returns the 3×3 identity matrix.
    pub fn identity() -> Self {
        Self {
            data: {
                [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
            }
        }
    }

    /// Returns `self * v`.
    pub fn mul_vector(&self, v: &Vector3) -> Vector3 {
        Vector3::new(
            self.data[0] * v.x + self.data[1] * v.y + self.data[2] * v.z,
            self.data[3] * v.x + self.data[4] * v.y + self.data[5] * v.z,
            self.data[6] * v.x + self.data[7] * v.y + self.data[8] * v.z,
        )
    }

    /// Returns the transpose of `self`.
    pub fn transpose(&self) -> Self {
        let mut new_data = [0.0;9];
        for i in 0..self.data.len() {
            let t_ind = (i % 3) * 3 + (i / 3);
            new_data[t_ind] = self.data[i];
        }
        Self {
            data: new_data
        }
    }
}

// ---------------------------------------------------------------------------

/// Unit quaternion in Hamilton scalar-first convention `[w, x, y, z]`.
#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Quaternion {
    
    pub fn new(w:f64, x:f64, y:f64, z:f64) -> Self {
        Self {
            w, x, y, z
        }
    }

    pub fn identity() -> Self {
        Self {
            w: 1.0, 
            x: 0.0, 
            y: 0.0, 
            z: 0.0
        }
    }

    /// Returns a unit-length copy of `self`.
    pub fn normalize(&self) -> Self {
        let d = (self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z).sqrt();
        Self {
            w: self.w / d,
            x: self.x / d,
            y: self.y / d,
            z: self.z / d
        }
    }

    /// Returns the conjugate `[w, -x, -y, -z]`.
    pub fn conjugate(&self) -> Self {
        Self {
            w: self.w,
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }

    /// Returns the Hamilton product `self ⊗ other`.
    pub fn multiply(&self, other: &Quaternion) -> Self {
        Self {
            w: self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
            x: self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
            y: self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
            z: self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
        }
    }

    /// Constructs a quaternion from Euler angles in radians (intrinsic ZYX order).
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        let sr = (roll/2.0).sin();
        let cr = (roll/2.0).cos();
        let sp = (pitch/2.0).sin();
        let cp = (pitch/2.0).cos();
        let sy = (yaw/2.0).sin();
        let cy = (yaw/2.0).cos();

        Self {
            w: cr*cp*cy + sr*sp*sy,
            x: sr*cp*cy - cr*sp*sy,
            y: cr*sp*cy + sr*cp*sy,
            z: cr*cp*sy - sr*sp*cy
        }
    }

    /// Converts `self` to an equivalent 3×3 rotation matrix.
    pub fn to_rotation_matrix(&self) -> Matrix3x3 {
        let data = [
            2.0 * (self.w * self.w + self.x * self.x) - 1.0,
            2.0 * (self.x * self.y - self.w * self.z),
            2.0 * (self.x * self.z + self.w * self.y),
            2.0 * (self.w * self.z + self.x * self.y),
            2.0 * (self.w * self.w + self.y * self.y) - 1.0,
            2.0 * (self.y * self.z - self.w * self.x),
            2.0 * (self.x * self.z - self.w * self.y),
            2.0 * (self.w * self.x + self.y * self.z),
            2.0 * (self.w * self.w + self.z * self.z) - 1.0
        ];

        Matrix3x3 { data }
    }

    /// Rotates vector `v` by this quaternion.
    pub fn rotate_vector(&self, v: &Vector3) -> Vector3 {
        let q = Quaternion::new(0.0, v.x, v.y, v.z);
        let rotated = self.multiply(&q).multiply(&self.conjugate());

        Vector3 {
            x: rotated.x,
            y: rotated.y,
            z: rotated.z
        }
    }

    /// Constructs a quaternion for a rotation of `angle` radians around `axis`.
    /// Returns identity if `axis` has near-zero norm.
    pub fn from_axis_angle(axis: &Vector3, angle: f64) -> Self {
        let norm = axis.norm();
        if norm < 1e-10 {
            Self::identity()
        }
        else {
            let axis_norm = axis.scale(1.0 / norm);
            let s = (angle/2.0).sin();
            Self {
                w: (angle/2.0).cos(),
                x: axis_norm.x * s,
                y: axis_norm.y * s,
                z: axis_norm.z * s
            }
        }
        
    }
}

// ---------------------------------------------------------------------------

/// 6×6 matrix stored row-major as `[f64; 36]`.
#[derive(Debug, Clone)]
pub struct Matrix6x6 {
    pub data: [f64; 36],
}

impl Matrix6x6 {
    /// Returns the all-zeros matrix.
    pub fn zeros() -> Self {
        Self {
            data: [0.0; 36]
        }
    }

    /// Returns the 6×6 identity matrix.
    pub fn identity() -> Self {
        Self {
            data: [
                    1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0
                ]
        }
    }

    /// Constructs a diagonal matrix from `diag`.
    pub fn from_diagonal(diag: &[f64; 6]) -> Self {
        Self {
            data: [
                    diag[0], 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, diag[1], 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, diag[2], 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, diag[3], 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, diag[4], 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, diag[5]
                ]
        }
    }

    /// Returns the element at (`row`, `col`).
    ///
    /// # Panics
    /// Panics if `row` or `col` >= 6.
    pub fn get(&self, row: usize, col: usize) -> f64 {
        assert!(row < 6 && col < 6);
        self.data[row * 6 + col]
    }

    /// Sets the element at (`row`, `col`) to `val`.
    ///
    /// # Panics
    /// Panics if `row` or `col` >= 6.
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        assert!(row < 6 && col < 6);
        self.data[row * 6 + col] = val;
    }

    /// Returns `self + other`.
    pub fn add(&self, other: &Matrix6x6) -> Self {
        let mut data = [0.0; 36];
        for i in 0..36 {
            data[i] = self.data[i] + other.data[i];
        }
        Self {
            data
        }
    }

    /// Returns `self - other`.
    pub fn sub(&self, other: &Matrix6x6) -> Self {
        let mut data = [0.0; 36];
        for i in 0..36 {
            data[i] = self.data[i] - other.data[i];
        }
        Self {
            data
        }
    }

    /// Returns each element multiplied by `s`.
    pub fn scale(&self, s: f64) -> Self {
        let mut data = [0.0; 36];
        for i in 0..36 {
            data[i] = self.data[i] * s;
        }

        Self { data }
    }

    /// Returns the transpose of `self`.
    pub fn transpose(&self) -> Self {
        let mut new_data = [0.0;36];
        for i in 0..self.data.len() {
            let t_ind = (i % 6) * 6 + (i / 6);
            new_data[t_ind] = self.data[i];
        }
        Self {
            data: new_data
        }
    }

    /// Returns `self * other` (matrix multiply).
    pub fn mul(&self, other: &Matrix6x6) -> Self {
        let mut data = [0.0; 36];

        for i in 0..6 {
            for j in 0..6 {
                for k in 0..6 {
                    data[i * 6 + j] += self.data[i * 6 + k] * other.data[k * 6 + j];
                }
            }
        }

        Self { data }
    }

    /// Returns the inverse of `self`, or `None` if the determinant is < 1e-12.
    pub fn inverse(&self) -> Option<Self> {
        todo!()
    }

    /// Returns `self * v` where `v` is a 6-element column vector.
    pub fn mul_vector(&self, v: &[f64; 6]) -> [f64; 6] {
        todo!()
    }
}
