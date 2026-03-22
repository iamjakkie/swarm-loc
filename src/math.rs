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

    }

    /// Returns `self + other`.
    pub fn add(&self, other: &Vector3) -> Self {
        todo!()
    }

    /// Returns `self - other`.
    pub fn sub(&self, other: &Vector3) -> Self {
        todo!()
    }

    /// Returns each component multiplied by `s`.
    pub fn scale(&self, s: f64) -> Self {
        todo!()
    }

    /// Returns the Euclidean norm.
    pub fn norm(&self) -> f64 {
        todo!()
    }

    /// Returns the dot product of `self` and `other`.
    pub fn dot(&self, other: &Vector3) -> f64 {
        todo!()
    }
}

// ---------------------------------------------------------------------------

/// 3×3 matrix stored row-major as `[f64; 9]`.
#[derive(Debug, Clone, Copy)]
pub struct Matrix3x3 {
    pub data: [f64; 9],
}

impl Matrix3x3 {
    /// Returns the 3×3 identity matrix.
    pub fn identity() -> Self {
        todo!()
    }

    /// Returns `self * v`.
    pub fn mul_vector(&self, v: &Vector3) -> Vector3 {
        todo!()
    }

    /// Returns the transpose of `self`.
    pub fn transpose(&self) -> Self {
        todo!()
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
    /// Returns the identity quaternion `[1, 0, 0, 0]`.
    pub fn identity() -> Self {
        todo!()
    }

    /// Returns a unit-length copy of `self`.
    pub fn normalize(&self) -> Self {
        todo!()
    }

    /// Returns the conjugate `[w, -x, -y, -z]`.
    pub fn conjugate(&self) -> Self {
        todo!()
    }

    /// Returns the Hamilton product `self ⊗ other`.
    pub fn multiply(&self, other: &Quaternion) -> Self {
        todo!()
    }

    /// Constructs a quaternion from Euler angles in radians (intrinsic ZYX order).
    pub fn from_euler(roll: f64, pitch: f64, yaw: f64) -> Self {
        todo!()
    }

    /// Converts `self` to an equivalent 3×3 rotation matrix.
    pub fn to_rotation_matrix(&self) -> Matrix3x3 {
        todo!()
    }

    /// Rotates vector `v` by this quaternion.
    pub fn rotate_vector(&self, v: &Vector3) -> Vector3 {
        todo!()
    }

    /// Constructs a quaternion for a rotation of `angle` radians around `axis`.
    /// Returns identity if `axis` has near-zero norm.
    pub fn from_axis_angle(axis: &Vector3, angle: f64) -> Self {
        todo!()
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
        todo!()
    }

    /// Returns the 6×6 identity matrix.
    pub fn identity() -> Self {
        todo!()
    }

    /// Constructs a diagonal matrix from `diag`.
    pub fn from_diagonal(diag: &[f64; 6]) -> Self {
        todo!()
    }

    /// Returns the element at (`row`, `col`).
    ///
    /// # Panics
    /// Panics if `row` or `col` >= 6.
    pub fn get(&self, row: usize, col: usize) -> f64 {
        todo!()
    }

    /// Sets the element at (`row`, `col`) to `val`.
    ///
    /// # Panics
    /// Panics if `row` or `col` >= 6.
    pub fn set(&mut self, row: usize, col: usize, val: f64) {
        todo!()
    }

    /// Returns `self + other`.
    pub fn add(&self, other: &Matrix6x6) -> Self {
        todo!()
    }

    /// Returns `self - other`.
    pub fn sub(&self, other: &Matrix6x6) -> Self {
        todo!()
    }

    /// Returns each element multiplied by `s`.
    pub fn scale(&self, s: f64) -> Self {
        todo!()
    }

    /// Returns the transpose of `self`.
    pub fn transpose(&self) -> Self {
        todo!()
    }

    /// Returns `self * other` (matrix multiply).
    pub fn mul(&self, other: &Matrix6x6) -> Self {
        todo!()
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
