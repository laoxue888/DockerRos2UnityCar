/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Pose2.h
 * @brief 2D Pose
 * @author: Frank Dellaert
 * @author: Richard Roberts
 */

// \callgraph

#pragma once

#include <gtsam/geometry/BearingRange.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/base/Lie.h>
#include <gtsam/dllexport.h>
#include <gtsam/base/std_optional_serialization.h>

#include <optional>

namespace gtsam {

/**
 * A 2D pose (Point2,Rot2)
 * @ingroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Pose2: public LieGroup<Pose2, 3> {

public:

  /// Pose Concept requirements
  using Rotation = Rot2;
  using Translation = Point2;

  /// LieGroup Concept requirements
  using LieAlgebra = Matrix3;

private:

  Rot2 r_;
  Point2 t_;

public:

  /// @name Standard Constructors
  /// @{

  /** default constructor = origin */
  Pose2() :
      r_(traits<Rot2>::Identity()), t_(traits<Point2>::Identity()) {
  }

  /** copy constructor */
  Pose2(const Pose2& pose) = default;
  //  : r_(pose.r_), t_(pose.t_) {}

  Pose2& operator=(const Pose2& other) = default;

  /**
   * construct from (x,y,theta)
   * @param x x coordinate
   * @param y y coordinate
   * @param theta angle with positive X-axis
   */
  Pose2(double x, double y, double theta) :
    r_(Rot2::fromAngle(theta)), t_(x, y) {
  }

  /** construct from rotation and translation */
  Pose2(double theta, const Point2& t) :
    r_(Rot2::fromAngle(theta)), t_(t) {
  }

  /** construct from r,t */
  Pose2(const Rot2& r, const Point2& t) : r_(r), t_(t) {}

  /** Constructor from 3*3 matrix */
  Pose2(const Matrix &T)
      : r_(Rot2::atan2(T(1, 0), T(0, 0))), t_(T(0, 2), T(1, 2)) {
#ifndef NDEBUG
    if (T.rows() != 3 || T.cols() != 3) {
      throw;
    }
#endif
  }

  /// @}
  /// @name Advanced Constructors
  /// @{

  /** Construct from canonical coordinates \f$ [T_x,T_y,\theta] \f$ (Lie algebra) */
  Pose2(const Vector& v) : Pose2() {
    *this = Expmap(v);
  }

  /**
   *  Create Pose2 by aligning two point pairs
   *  A pose aTb is estimated between pairs (a_point, b_point) such that 
   *    a_point = aTb * b_point
   *  Note this allows for noise on the points but in that case the mapping 
   *  will not be exact.
   */
  static std::optional<Pose2> Align(const Point2Pairs& abPointPairs);

  // Version of Pose2::Align that takes 2 matrices.
  static std::optional<Pose2> Align(const Matrix& a, const Matrix& b);

  /// @}
  /// @name Testable
  /// @{

  /** print with optional string */
  void print(const std::string& s = "") const;

  /** assert equality up to a tolerance */
  bool equals(const Pose2& pose, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  inline static Pose2 Identity() { return Pose2(); }

  /// inverse
  Pose2 inverse() const;

  /// compose syntactic sugar
  inline Pose2 operator*(const Pose2& p2) const {
    return Pose2(r_*p2.r(), t_ + r_*p2.t());
  }

  /// @}
  /// @name Lie Group
  /// @{

  ///Exponential map at identity - create a rotation from canonical coordinates \f$ [T_x,T_y,\theta] \f$
  static Pose2 Expmap(const Vector3& xi, ChartJacobian H = {});

  ///Log map at identity - return the canonical coordinates \f$ [T_x,T_y,\theta] \f$ of this rotation
  static Vector3 Logmap(const Pose2& p, ChartJacobian H = {});

  /**
   * Calculate Adjoint map
   * Ad_pose is 3*3 matrix that when applied to twist xi \f$ [T_x,T_y,\theta] \f$, returns Ad_pose(xi)
   */
  Matrix3 AdjointMap() const;

  /// Apply AdjointMap to twist xi
  inline Vector3 Adjoint(const Vector3& xi) const {
    return AdjointMap()*xi;
  }

  /**
   * Compute the [ad(w,v)] operator for SE2 as in [Kobilarov09siggraph], pg 19
   */
  static Matrix3 adjointMap(const Vector3& v);

  /**
   * Action of the adjointMap on a Lie-algebra vector y, with optional derivatives
   */
  static Vector3 adjoint(const Vector3& xi, const Vector3& y) {
    return adjointMap(xi) * y;
  }

  /**
   * The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
   */
  static Vector3 adjointTranspose(const Vector3& xi, const Vector3& y) {
    return adjointMap(xi).transpose() * y;
  }

  // temporary fix for wrappers until case issue is resolved
  static Matrix3 adjointMap_(const Vector3 &xi) { return adjointMap(xi);}
  static Vector3 adjoint_(const Vector3 &xi, const Vector3 &y) { return adjoint(xi, y);}

  /// Derivative of Expmap
  static Matrix3 ExpmapDerivative(const Vector3& v);

  /// Derivative of Logmap
  static Matrix3 LogmapDerivative(const Pose2& v);

  // Chart at origin, depends on compile-time flag SLOW_BUT_CORRECT_EXPMAP
  struct GTSAM_EXPORT ChartAtOrigin {
    static Pose2 Retract(const Vector3& v, ChartJacobian H = {});
    static Vector3 Local(const Pose2& r, ChartJacobian H = {});
  };

  using LieGroup<Pose2, 3>::inverse; // version with derivative

  /// Hat maps from tangent vector to Lie algebra
  static Matrix3 Hat(const Vector3& xi);

  /// Vee maps from Lie algebra to tangent vector
  static Vector3 Vee(const Matrix3& X);

  /// @}
  /// @name Group Action on Point2
  /// @{

  /** Return point coordinates in pose coordinate frame */
  Point2 transformTo(const Point2& point,
      OptionalJacobian<2, 3> Dpose = {},
      OptionalJacobian<2, 2> Dpoint = {}) const;

  /**
   * @brief transform many points in world coordinates and transform to Pose.
   * @param points 2*N matrix in world coordinates
   * @return points in Pose coordinates, as 2*N Matrix
   */
  Matrix transformTo(const Matrix& points) const;

  /** Return point coordinates in global frame */
  Point2 transformFrom(const Point2& point,
      OptionalJacobian<2, 3> Dpose = {},
      OptionalJacobian<2, 2> Dpoint = {}) const;

  /**
   * @brief transform many points in Pose coordinates and transform to world.
   * @param points 2*N matrix in Pose coordinates
   * @return points in world coordinates, as 2*N Matrix
   */
  Matrix transformFrom(const Matrix& points) const;

  /** syntactic sugar for transformFrom */
  inline Point2 operator*(const Point2& point) const { 
    return transformFrom(point);
  }

  /// @}
  /// @name Standard Interface
  /// @{

  /// get x
  inline double x()     const { return t_.x(); }

  /// get y
  inline double y()     const { return t_.y(); }

  /// get theta
  inline double theta() const { return r_.theta(); }

  /// translation
  inline const Point2& t() const { return t_; }

  /// rotation
  inline const Rot2&   r() const { return r_; }

  /// translation
  inline const Point2& translation(OptionalJacobian<2, 3> Hself={}) const {
    if (Hself) {
      *Hself = Matrix::Zero(2, 3);
      (*Hself).block<2, 2>(0, 0) = rotation().matrix();
    }
    return t_;
  }

  /// rotation
  inline const Rot2&   rotation(OptionalJacobian<1, 3> Hself={}) const {
    if (Hself) *Hself << 0, 0, 1;
    return r_;
  }

  //// return transformation matrix
  Matrix3 matrix() const;

  /**
   * Calculate bearing to a landmark
   * @param point 2D location of landmark
   * @return 2D rotation \f$ \in SO(2) \f$
   */
  Rot2 bearing(const Point2& point,
               OptionalJacobian<1, 3> H1={}, OptionalJacobian<1, 2> H2={}) const;

  /**
   * Calculate bearing to another pose
   * @param point SO(2) location of other pose
   * @return 2D rotation \f$ \in SO(2) \f$
   */
  Rot2 bearing(const Pose2& pose,
               OptionalJacobian<1, 3> H1={}, OptionalJacobian<1, 3> H2={}) const;

  /**
   * Calculate range to a landmark
   * @param point 2D location of landmark
   * @return range (double)
   */
  double range(const Point2& point,
      OptionalJacobian<1, 3> H1={},
      OptionalJacobian<1, 2> H2={}) const;

  /**
   * Calculate range to another pose
   * @param point 2D location of other pose
   * @return range (double)
   */
  double range(const Pose2& point,
      OptionalJacobian<1, 3> H1={},
      OptionalJacobian<1, 3> H2={}) const;

  /// @}
  /// @name Advanced Interface
  /// @{

  /**
   * Return the start and end indices (inclusive) of the translation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  inline static std::pair<size_t, size_t> translationInterval() { return {0, 1}; }

  /**
   * Return the start and end indices (inclusive) of the rotation component of the
   * exponential map parameterization
   * @return a pair of [start, end] indices into the tangent space vector
   */
  static std::pair<size_t, size_t> rotationInterval() { return {2, 2}; }

  /// Return vectorized SE(2) matrix in column order.
  Vector9 vec(OptionalJacobian<9, 3> H = {}) const;

  /// Output stream operator
  GTSAM_EXPORT
  friend std::ostream &operator<<(std::ostream &os, const Pose2& p);

  /// @}
  /// @name deprecated
  /// @{

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
  /// @deprecated: use Hat
  static inline Matrix3 wedge(double vx, double vy, double w) {
    return Hat(TangentVector(vx, vy, w));
  }
#endif
  /// @}

 private:

#if GTSAM_ENABLE_BOOST_SERIALIZATION  //
  // Serialization function
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int /*version*/) {
    ar & BOOST_SERIALIZATION_NVP(t_);
    ar & BOOST_SERIALIZATION_NVP(r_);
  }
#endif

public:
  // Align for Point2, which is either derived from, or is typedef, of Vector2
  GTSAM_MAKE_ALIGNED_OPERATOR_NEW
}; // Pose2

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V43
/// @deprecated: use T::Hat
template <>
inline Matrix wedge<Pose2>(const Vector& xi) {
  // NOTE(chris): Need eval() as workaround for Apple clang + avx2.
  return Matrix(Pose2::Hat(xi)).eval();
}
#endif

// Convenience typedef
using Pose2Pair = std::pair<Pose2, Pose2>;
using Pose2Pairs = std::vector<Pose2Pair>;

template <>
struct traits<Pose2> : public internal::MatrixLieGroup<Pose2> {};

template <>
struct traits<const Pose2> : public internal::MatrixLieGroup<Pose2> {};

// bearing and range traits, used in RangeFactor
template <typename T>
struct Bearing<Pose2, T> : HasBearing<Pose2, T, Rot2> {};

template <typename T>
struct Range<Pose2, T> : HasRange<Pose2, T, double> {};

} // namespace gtsam

