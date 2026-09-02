//! Lightweight 2D vector math used by every simulation module.
//!
//! [`Vec2`] is a newtype over `[f32; 2]`. All operations are `#[inline]` and
//! perform no heap allocation, so the struct is safe to use in hot inner loops.

use core::ops::{Add, AddAssign, Mul, Neg, Sub, SubAssign};

/// A 2D vector backed by `[f32; 2]`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Vec2(pub [f32; 2]);

impl Vec2 {
    /// Constructs a vector from raw components.
    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self([x, y])
    }

    /// Returns the zero vector.
    #[inline]
    pub const fn zero() -> Self {
        Self([0.0, 0.0])
    }

    /// X component.
    #[inline]
    pub fn x(self) -> f32 {
        self.0[0]
    }

    /// Y component.
    #[inline]
    pub fn y(self) -> f32 {
        self.0[1]
    }

    /// Squared length. Use this in hot paths to avoid a `sqrt`.
    #[inline]
    pub fn length_sq(self) -> f32 {
        self.0[0] * self.0[0] + self.0[1] * self.0[1]
    }

    /// Euclidean length.
    #[inline]
    pub fn length(self) -> f32 {
        self.length_sq().sqrt()
    }

    /// Dot product.
    #[inline]
    pub fn dot(self, rhs: Self) -> f32 {
        self.0[0] * rhs.0[0] + self.0[1] * rhs.0[1]
    }

    /// Component-wise scalar multiply.
    #[inline]
    pub fn scale(self, k: f32) -> Self {
        Self([self.0[0] * k, self.0[1] * k])
    }

    /// Returns a unit-length copy, or the zero vector when `self` is
    /// (numerically) the zero vector. Avoids producing NaN.
    #[inline]
    pub fn normalize_or_zero(self) -> Self {
        let len_sq = self.length_sq();
        if len_sq > 0.0 && len_sq.is_finite() {
            let inv = 1.0 / len_sq.sqrt();
            Self([self.0[0] * inv, self.0[1] * inv])
        } else {
            Self::zero()
        }
    }

    /// Clamps the length to `max` without changing direction. The zero vector
    /// is returned unchanged.
    #[inline]
    pub fn limit(self, max: f32) -> Self {
        let len_sq = self.length_sq();
        if len_sq > max * max && len_sq > 0.0 {
            let k = max / len_sq.sqrt();
            Self([self.0[0] * k, self.0[1] * k])
        } else {
            self
        }
    }
}

impl Add for Vec2 {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self([self.0[0] + rhs.0[0], self.0[1] + rhs.0[1]])
    }
}

impl AddAssign for Vec2 {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.0[0] += rhs.0[0];
        self.0[1] += rhs.0[1];
    }
}

impl Sub for Vec2 {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self([self.0[0] - rhs.0[0], self.0[1] - rhs.0[1]])
    }
}

impl SubAssign for Vec2 {
    #[inline]
    fn sub_assign(&mut self, rhs: Self) {
        self.0[0] -= rhs.0[0];
        self.0[1] -= rhs.0[1];
    }
}

impl Mul<f32> for Vec2 {
    type Output = Self;
    #[inline]
    fn mul(self, k: f32) -> Self {
        self.scale(k)
    }
}

impl Neg for Vec2 {
    type Output = Self;
    #[inline]
    fn neg(self) -> Self {
        Self([-self.0[0], -self.0[1]])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-5
    }

    #[test]
    fn add_is_commutative() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, 4.0);
        assert_eq!(a + b, b + a);
        assert_eq!(a + b, Vec2::new(4.0, 6.0));
    }

    #[test]
    fn add_assign_works() {
        let mut a = Vec2::new(1.0, 2.0);
        a += Vec2::new(3.0, 4.0);
        assert_eq!(a, Vec2::new(4.0, 6.0));
    }

    #[test]
    fn sub_and_neg() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(4.0, 6.0);
        assert_eq!(a - b, Vec2::new(-3.0, -4.0));
        assert_eq!(-a, Vec2::new(-1.0, -2.0));
    }

    #[test]
    fn scale_and_mul() {
        let v = Vec2::new(1.0, -2.0);
        assert_eq!(v.scale(3.0), v * 3.0);
        assert_eq!(v * 3.0, Vec2::new(3.0, -6.0));
    }

    #[test]
    fn dot_product() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, 4.0);
        assert!(approx_eq(a.dot(b), 11.0));
        assert!(approx_eq(a.dot(Vec2::zero()), 0.0));
    }

    #[test]
    fn length_and_length_sq() {
        let v = Vec2::new(3.0, 4.0);
        assert!(approx_eq(v.length_sq(), 25.0));
        assert!(approx_eq(v.length(), 5.0));
    }

    #[test]
    fn normalize_or_zero_handles_zero() {
        let z = Vec2::zero();
        assert_eq!(z.normalize_or_zero(), Vec2::zero());
    }

    #[test]
    fn normalize_or_zero_handles_unit() {
        let v = Vec2::new(10.0, 0.0);
        let n = v.normalize_or_zero();
        assert!(approx_eq(n.x(), 1.0));
        assert!(approx_eq(n.y(), 0.0));
    }

    #[test]
    fn limit_clamps_length() {
        let v = Vec2::new(3.0, 4.0); // length 5
        let clamped = v.limit(2.0);
        assert!(approx_eq(clamped.length(), 2.0));
    }

    #[test]
    fn limit_passes_through_small_vectors() {
        let v = Vec2::new(0.5, 0.5);
        assert_eq!(v.limit(2.0), v);
    }

    #[test]
    fn limit_does_not_normalize_zero() {
        let z = Vec2::zero();
        assert_eq!(z.limit(1.0), Vec2::zero());
    }
}
