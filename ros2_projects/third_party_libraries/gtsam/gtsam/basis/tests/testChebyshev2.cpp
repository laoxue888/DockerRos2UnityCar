/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

 /**
  * @file testChebyshev2.cpp
  * @date July 4, 2020
  * @author Varun Agrawal
  * @brief Unit tests for Chebyshev Basis Decompositions via pseudo-spectral
  *        methods
  */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/basis/Chebyshev2.h>
#include <gtsam/basis/FitBasis.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/factorTesting.h>

#include <cstddef>
#include <functional>

using namespace gtsam;

//******************************************************************************
TEST(Chebyshev2, Point) {
  static const size_t N = 5;
  auto points = Chebyshev2::Points(N);
  Vector expected(N);
  expected << -1., -sqrt(2.) / 2., 0., sqrt(2.) / 2., 1.;
  static const double tol = 1e-15;  // changing this reveals errors
  EXPECT_DOUBLES_EQUAL(expected(0), points(0), tol);
  EXPECT_DOUBLES_EQUAL(expected(1), points(1), tol);
  EXPECT_DOUBLES_EQUAL(expected(2), points(2), tol);
  EXPECT_DOUBLES_EQUAL(expected(3), points(3), tol);
  EXPECT_DOUBLES_EQUAL(expected(4), points(4), tol);

  // Check symmetry
  EXPECT_DOUBLES_EQUAL(Chebyshev2::Point(N, 0), -Chebyshev2::Point(N, 4), tol);
  EXPECT_DOUBLES_EQUAL(Chebyshev2::Point(N, 1), -Chebyshev2::Point(N, 3), tol);
}

//******************************************************************************
TEST(Chebyshev2, PointInInterval) {
  static const size_t N = 5;
  auto points = Chebyshev2::Points(N, 0, 20);
  Vector expected(N);
  expected << 0., 1. - sqrt(2.) / 2., 1., 1. + sqrt(2.) / 2., 2.;
  expected *= 10.0;
  static const double tol = 1e-15;  // changing this reveals errors
  EXPECT_DOUBLES_EQUAL(expected(0), points(0), tol);
  EXPECT_DOUBLES_EQUAL(expected(1), points(1), tol);
  EXPECT_DOUBLES_EQUAL(expected(2), points(2), tol);
  EXPECT_DOUBLES_EQUAL(expected(3), points(3), tol);
  EXPECT_DOUBLES_EQUAL(expected(4), points(4), tol);

  // all at once
  Vector actual = Chebyshev2::Points(N, 0, 20);
  CHECK(assert_equal(expected, actual));
}

//******************************************************************************
// InterpolatingPolynomial[{{-1, 4}, {0, 2}, {1, 6}}, 0.5]
TEST(Chebyshev2, Interpolate2) {
  const size_t N = 3;
  Chebyshev2::EvaluationFunctor fx(N, 0.5);
  Vector f(N);
  f << 4, 2, 6;
  EXPECT_DOUBLES_EQUAL(3.25, fx(f), 1e-9);
}

//******************************************************************************
// InterpolatingPolynomial[{{0, 4}, {1, 2}, {2, 6}}, 1.5]
TEST(Chebyshev2, Interpolate2_Interval) {
  Chebyshev2::EvaluationFunctor fx(3, 1.5, 0, 2);
  Vector3 f(4, 2, 6);
  EXPECT_DOUBLES_EQUAL(3.25, fx(f), 1e-9);
}

//******************************************************************************
// InterpolatingPolynomial[{{-1, 4}, {-Sqrt[2]/2, 2}, {0, 6}, {Sqrt[2]/2,3}, {1,
// 3}}, 0.5]
TEST(Chebyshev2, Interpolate5) {
  Chebyshev2::EvaluationFunctor fx(5, 0.5);
  Vector f(5);
  f << 4, 2, 6, 3, 3;
  EXPECT_DOUBLES_EQUAL(4.34283, fx(f), 1e-5);
}

//******************************************************************************
// Interpolating vectors
TEST(Chebyshev2, InterpolateVector) {
  double t = 30, a = 0, b = 100;
  const size_t N = 3;
  // Create 2x3 matrix with Vectors at Chebyshev points
  Matrix X = Matrix::Zero(2, N);
  X.row(0) = Chebyshev2::Points(N, a, b);  // slope 1 ramp

  // Check value
  Vector expected(2);
  expected << t, 0;
  Eigen::Matrix<double, /*2x2N*/ -1, -1> actualH(2, 2 * N);

  Chebyshev2::VectorEvaluationFunctor fx(2, N, t, a, b);
  EXPECT(assert_equal(expected, fx(X, actualH), 1e-9));

  // Check derivative
  std::function<Vector2(Matrix)> f =
    std::bind(&Chebyshev2::VectorEvaluationFunctor::operator(), fx,
      std::placeholders::_1, nullptr);
  Matrix numericalH =
    numericalDerivative11<Vector2, Matrix, 2 * N>(f, X);
  EXPECT(assert_equal(numericalH, actualH, 1e-9));
}

//******************************************************************************
// Interpolating poses using the exponential map
TEST(Chebyshev2, InterpolatePose2) {
  const size_t N = 32;
  double t = 30, a = 0, b = 100;

  Matrix X(3, N);
  X.row(0) = Chebyshev2::Points(N, a, b);  // slope 1 ramp
  X.row(1) = Vector::Zero(N);
  X.row(2) = 0.1 * Vector::Ones(N);

  Vector xi(3);
  xi << t, 0, 0.1;
  Eigen::Matrix<double, /*3x3N*/ -1, -1> actualH(3, 3 * N);

  Chebyshev2::ManifoldEvaluationFunctor<Pose2> fx(N, t, a, b);
  // We use xi as canonical coordinates via exponential map
  Pose2 expected = Pose2::ChartAtOrigin::Retract(xi);
  EXPECT(assert_equal(expected, fx(X, actualH)));

  // Check derivative
  std::function<Pose2(Matrix)> f =
    std::bind(&Chebyshev2::ManifoldEvaluationFunctor<Pose2>::operator(), fx,
      std::placeholders::_1, nullptr);
  Matrix numericalH =
    numericalDerivative11<Pose2, Matrix, 3 * N>(f, X);
  EXPECT(assert_equal(numericalH, actualH, 1e-9));
}

#ifdef GTSAM_POSE3_EXPMAP
//******************************************************************************
// Interpolating poses using the exponential map
TEST(Chebyshev2, InterpolatePose3) {
  const size_t N = 32;
  double a = 10, b = 100;
  double t = Chebyshev2::Points(N, a, b)(11);

  Rot3 R = Rot3::Ypr(-2.21366492e-05, -9.35353636e-03, -5.90463598e-04);
  Pose3 pose(R, Point3(1, 2, 3));

  Vector6 xi = Pose3::ChartAtOrigin::Local(pose);
  Eigen::Matrix<double, /*6x6N*/ -1, -1> actualH(6, 6 * N);

  Matrix X = Matrix::Zero(6, N);
  X.col(11) = xi;

  Chebyshev2::ManifoldEvaluationFunctor<Pose3> fx(N, t, a, b);
  // We use xi as canonical coordinates via exponential map
  Pose3 expected = Pose3::ChartAtOrigin::Retract(xi);
  EXPECT(assert_equal(expected, fx(X, actualH)));

  // Check derivative
  std::function<Pose3(Matrix)> f =
    std::bind(&Chebyshev2::ManifoldEvaluationFunctor<Pose3>::operator(), fx,
      std::placeholders::_1, nullptr);
  Matrix numericalH =
    numericalDerivative11<Pose3, Matrix, 6 * N>(f, X);
  EXPECT(assert_equal(numericalH, actualH, 1e-8));
}
#endif

//******************************************************************************
TEST(Chebyshev2, Decomposition) {
  // Create example sequence
  Sequence sequence;
  for (size_t i = 0; i < 16; i++) {
    double x = (1.0 / 16) * i - 0.99, y = x;
    sequence[x] = y;
  }

  // Do Chebyshev Decomposition
  FitBasis<Chebyshev2> actual(sequence, nullptr, 3);

  // Check
  Vector expected(3);
  expected << -1, 0, 1;
  EXPECT(assert_equal(expected, actual.parameters(), 1e-4));
}

//******************************************************************************
TEST(Chebyshev2, DifferentiationMatrix3) {
  // Trefethen00book, p.55
  const size_t N = 3;
  Matrix expected(N, N);
  // Differentiation matrix computed from chebfun
  expected << 1.5000, -2.0000, 0.5000,  //
    0.5000, -0.0000, -0.5000,         //
    -0.5000, 2.0000, -1.5000;
  // multiply by -1 since the chebyshev points have a phase shift wrt Trefethen
  // This was verified with chebfun
  expected = -expected;

  Matrix actual = Chebyshev2::DifferentiationMatrix(N);
  EXPECT(assert_equal(expected, actual, 1e-4));
}

//******************************************************************************
TEST(Chebyshev2, DerivativeMatrix6) {
  // Trefethen00book, p.55
  const size_t N = 6;
  Matrix expected(N, N);
  expected << 8.5000, -10.4721, 2.8944, -1.5279, 1.1056, -0.5000,  //
    2.6180, -1.1708, -2.0000, 0.8944, -0.6180, 0.2764,           //
    -0.7236, 2.0000, -0.1708, -1.6180, 0.8944, -0.3820,          //
    0.3820, -0.8944, 1.6180, 0.1708, -2.0000, 0.7236,            //
    -0.2764, 0.6180, -0.8944, 2.0000, 1.1708, -2.6180,           //
    0.5000, -1.1056, 1.5279, -2.8944, 10.4721, -8.5000;
  // multiply by -1 since the chebyshev points have a phase shift wrt Trefethen
  // This was verified with chebfun
  expected = -expected;

  Matrix actual = Chebyshev2::DifferentiationMatrix(N);
  EXPECT(assert_equal((Matrix)expected, actual, 1e-4));
}

// test function for CalculateWeights and DerivativeWeights
double f(double x) {
  // return 3*(x**3) - 2*(x**2) + 5*x - 11
  return 3.0 * pow(x, 3) - 2.0 * pow(x, 2) + 5.0 * x - 11;
}

// its derivative
double fprime(double x) {
  // return 9*(x**2) - 4*(x) + 5
  return 9.0 * pow(x, 2) - 4.0 * x + 5.0;
}

//******************************************************************************
TEST(Chebyshev2, CalculateWeights) {
  const size_t N = 32;
  Vector fvals = Chebyshev2::vector(f, N);
  double x1 = 0.7, x2 = -0.376;
  Weights weights1 = Chebyshev2::CalculateWeights(N, x1);
  Weights weights2 = Chebyshev2::CalculateWeights(N, x2);
  EXPECT_DOUBLES_EQUAL(f(x1), weights1 * fvals, 1e-8);
  EXPECT_DOUBLES_EQUAL(f(x2), weights2 * fvals, 1e-8);
}

TEST(Chebyshev2, CalculateWeights2) {
  const size_t N = 32;
  double a = 0, b = 10, x1 = 7, x2 = 4.12;
  Vector fvals = Chebyshev2::vector(f, N, a, b);

  Weights weights1 = Chebyshev2::CalculateWeights(N, x1, a, b);
  EXPECT_DOUBLES_EQUAL(f(x1), weights1 * fvals, 1e-8);

  Weights weights2 = Chebyshev2::CalculateWeights(N, x2, a, b);
  double expected2 = f(x2);  // 185.454784
  double actual2 = weights2 * fvals;
  EXPECT_DOUBLES_EQUAL(expected2, actual2, 1e-8);
}

// Test CalculateWeights when a point coincides with a Chebyshev point
TEST(Chebyshev2, CalculateWeights_CoincidingPoint) {
  const size_t N = 5;
  const double coincidingPoint = Chebyshev2::Point(N, 1);  // Pick the 2nd point

  // Generate weights for the coinciding point
  Weights weights = Chebyshev2::CalculateWeights(N, coincidingPoint);

  // Verify that the weights are zero everywhere except at the coinciding point
  for (size_t j = 0; j < N; ++j) {
    EXPECT_DOUBLES_EQUAL(j == 1 ? 1.0 : 0.0, weights(j), 1e-9);
  }
}

TEST(Chebyshev2, DerivativeWeights) {
  const size_t N = 32;
  Vector fvals = Chebyshev2::vector(f, N);
  std::vector<double> testPoints = { 0.7, -0.376, 0.0 };
  for (double x : testPoints) {
    Weights dWeights = Chebyshev2::DerivativeWeights(N, x);
    EXPECT_DOUBLES_EQUAL(fprime(x), dWeights * fvals, 1e-9);
  }

  // test if derivative calculation at Chebyshev point is correct
  double x4 = Chebyshev2::Point(N, 3);
  Weights dWeights4 = Chebyshev2::DerivativeWeights(N, x4);
  EXPECT_DOUBLES_EQUAL(fprime(x4), dWeights4 * fvals, 1e-9);
}

TEST(Chebyshev2, DerivativeWeights2) {
  const size_t N = 32;
  double x1 = 5, x2 = 4.12, a = 0, b = 10;
  Vector fvals = Chebyshev2::vector(f, N, a, b);

  Weights dWeights1 = Chebyshev2::DerivativeWeights(N, x1, a, b);
  EXPECT_DOUBLES_EQUAL(fprime(x1), dWeights1 * fvals, 1e-8);

  Weights dWeights2 = Chebyshev2::DerivativeWeights(N, x2, a, b);
  EXPECT_DOUBLES_EQUAL(fprime(x2), dWeights2 * fvals, 1e-8);

  // test if derivative calculation at Chebyshev point is correct
  double x3 = Chebyshev2::Point(N, 3, a, b);
  Weights dWeights3 = Chebyshev2::DerivativeWeights(N, x3, a, b);
  EXPECT_DOUBLES_EQUAL(fprime(x3), dWeights3 * fvals, 1e-8);
}


//******************************************************************************
// Check two different ways to calculate the derivative weights
TEST(Chebyshev2, DerivativeWeightsDifferentiationMatrix) {
  const size_t N6 = 6;
  double x1 = 0.311;
  Matrix D6 = Chebyshev2::DifferentiationMatrix(N6);
  Weights expected = Chebyshev2::CalculateWeights(N6, x1) * D6;
  Weights actual = Chebyshev2::DerivativeWeights(N6, x1);
  EXPECT(assert_equal(expected, actual, 1e-12));

  double a = -3, b = 8, x2 = 5.05;
  Matrix D6_2 = Chebyshev2::DifferentiationMatrix(N6, a, b);
  Weights expected1 = Chebyshev2::CalculateWeights(N6, x2, a, b) * D6_2;
  Weights actual1 = Chebyshev2::DerivativeWeights(N6, x2, a, b);
  EXPECT(assert_equal(expected1, actual1, 1e-12));
}

//******************************************************************************
// Check two different ways to calculate the derivative weights
TEST(Chebyshev2, DerivativeWeights6) {
  const size_t N6 = 6;
  Matrix D6 = Chebyshev2::DifferentiationMatrix(N6);
  Chebyshev2::Parameters x = Chebyshev2::Points(N6);  // ramp with slope 1
  EXPECT(assert_equal(Vector::Ones(N6), Vector(D6 * x)));
}

//******************************************************************************
// Check two different ways to calculate the derivative weights
TEST(Chebyshev2, DerivativeWeights7) {
  const size_t N7 = 7;
  Matrix D7 = Chebyshev2::DifferentiationMatrix(N7);
  Chebyshev2::Parameters x = Chebyshev2::Points(N7);  // ramp with slope 1
  EXPECT(assert_equal(Vector::Ones(N7), Vector(D7 * x)));
}

//******************************************************************************
// Check derivative in two different ways: numerical and using D on f
Vector6 f3_at_6points = (Vector6() << 4, 2, 6, 2, 4, 3).finished();
double proxy3(double x) {
  return Chebyshev2::EvaluationFunctor(6, x)(f3_at_6points);
}

// Check Derivative evaluation at point x=0.2
TEST(Chebyshev2, Derivative6) {
  // calculate expected values by numerical derivative of synthesis
  const double x = 0.2;
  Matrix numeric_dTdx = numericalDerivative11<double, double>(proxy3, x);

  // Calculate derivatives at Chebyshev points using D3, interpolate
  Matrix D6 = Chebyshev2::DifferentiationMatrix(6);
  Vector derivative_at_points = D6 * f3_at_6points;
  Chebyshev2::EvaluationFunctor fx(6, x);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), fx(derivative_at_points), 1e-8);

  // Do directly
  Chebyshev2::DerivativeFunctor dfdx(6, x);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), dfdx(f3_at_6points), 1e-8);
}

//******************************************************************************
// Assert that derivative also works in non-standard interval [0,3]
double proxy4(double x) {
  return Chebyshev2::EvaluationFunctor(6, x, 0, 3)(f3_at_6points);
}

TEST(Chebyshev2, Derivative6_03) {
  // Check Derivative evaluation at point x=0.2, in interval [0,3]

  // calculate expected values by numerical derivative of synthesis
  const double x = 0.2;
  Matrix numeric_dTdx = numericalDerivative11<double, double>(proxy4, x);

  // Calculate derivatives at Chebyshev points using D3, interpolate
  Matrix D6 = Chebyshev2::DifferentiationMatrix(6, 0, 3);
  Vector derivative_at_points = D6 * f3_at_6points;
  Chebyshev2::EvaluationFunctor fx(6, x, 0, 3);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), fx(derivative_at_points), 1e-8);

  // Do directly
  Chebyshev2::DerivativeFunctor dfdx(6, x, 0, 3);
  EXPECT_DOUBLES_EQUAL(numeric_dTdx(0, 0), dfdx(f3_at_6points), 1e-8);
}

//******************************************************************************
// Test VectorDerivativeFunctor
TEST(Chebyshev2, VectorDerivativeFunctor) {
  const size_t N = 3, M = 2;
  const double x = 0.2;
  using VecD = Chebyshev2::VectorDerivativeFunctor;
  VecD fx(M, N, x, 0, 3);
  Matrix X = Matrix::Zero(M, N);
  Matrix actualH(M, M * N);
  EXPECT(assert_equal(Vector::Zero(M), (Vector)fx(X, actualH), 1e-8));

  // Test Jacobian
  Matrix expectedH = numericalDerivative11<Vector2, Matrix, M* N>(
    std::bind(&VecD::operator(), fx, std::placeholders::_1, nullptr), X);
  EXPECT(assert_equal(expectedH, actualH, 1e-7));
}

//******************************************************************************
// Test VectorDerivativeFunctor with polynomial function
TEST(Chebyshev2, VectorDerivativeFunctor2) {
  const size_t N = 4, M = 1, T = 15;
  using VecD = Chebyshev2::VectorDerivativeFunctor;

  const Vector points = Chebyshev2::Points(N, 0, T);

  // Assign the parameter matrix 1xN
  Matrix X(1, N);
  for (size_t i = 0; i < N; ++i) {
    X(i) = f(points(i));
  }

  // Evaluate the derivative at the chebyshev points using
  // VectorDerivativeFunctor.
  for (size_t i = 0; i < N; ++i) {
    VecD d(M, N, points(i), 0, T);
    Vector1 Dx = d(X);
    EXPECT_DOUBLES_EQUAL(fprime(points(i)), Dx(0), 1e-6);
  }

  // Test Jacobian at the first chebyshev point.
  Matrix actualH(M, M * N);
  VecD vecd(M, N, points(0), 0, T);
  vecd(X, actualH);
  Matrix expectedH = numericalDerivative11<Vector1, Matrix, M* N>(
    std::bind(&VecD::operator(), vecd, std::placeholders::_1, nullptr), X);
  EXPECT(assert_equal(expectedH, actualH, 1e-6));
}

//******************************************************************************
// Test ComponentDerivativeFunctor
TEST(Chebyshev2, ComponentDerivativeFunctor) {
  const size_t N = 6, M = 2;
  const double x = 0.2;
  using CompFunc = Chebyshev2::ComponentDerivativeFunctor;
  size_t row = 1;
  CompFunc fx(M, N, row, x, 0, 3);
  Matrix X = Matrix::Zero(M, N);
  Matrix actualH(1, M * N);
  EXPECT_DOUBLES_EQUAL(0, fx(X, actualH), 1e-8);

  Matrix expectedH = numericalDerivative11<double, Matrix, M* N>(
    std::bind(&CompFunc::operator(), fx, std::placeholders::_1, nullptr), X);
  EXPECT(assert_equal(expectedH, actualH, 1e-7));
}

//******************************************************************************
TEST(Chebyshev2, IntegrationMatrix) {
  const size_t N = 10;  // number of intervals => N+1 nodes
  const double a = 0, b = 10;

  // Create integration matrix
  Matrix P = Chebyshev2::IntegrationMatrix(N, a, b);

  // Let's check that integrating a constant yields
  // the sum of the lengths of the intervals:
  Vector F = P * Vector::Ones(N);
  EXPECT_DOUBLES_EQUAL(0, F(0), 1e-9); // check first value is 0
  Vector points = Chebyshev2::Points(N, a, b);
  Vector ramp(N);
  for (size_t i = 0; i < N; ++i) ramp(i) = points(i) - a;
  EXPECT(assert_equal(ramp, F, 1e-9));

  // Get values of the derivative (fprime) at the Chebyshev nodes
  Vector fp = Chebyshev2::vector(fprime, N, a, b);

  // Integrate to get back f, using the integration matrix.
  // Since there is a constant term, we need to add it back.
  Vector F_est = P * fp;
  EXPECT_DOUBLES_EQUAL(0, F_est(0), 1e-9); // check first value is 0

  // For comparison, get actual function values at the nodes
  Vector F_true = Chebyshev2::vector(f, N, a, b);

  // Verify the integration matrix worked correctly, after adding back the
  // constant term
  F_est.array() += f(a);
  EXPECT(assert_equal(F_true, F_est, 1e-9));

  // Differentiate the result to get back to our derivative function
  Matrix D = Chebyshev2::DifferentiationMatrix(N, a, b);
  Vector ff_est = D * F_est;

  // Verify the round trip worked
  EXPECT(assert_equal(fp, ff_est, 1e-9));
}

//******************************************************************************
TEST(Chebyshev2, IntegrationWeights7) {
  const size_t N = 7;
  Weights actual = Chebyshev2::IntegrationWeights(N, -1, 1);

  // Expected values were calculated using chebfun:
  Weights expected = (Weights(N) << 0.0285714285714286, 0.253968253968254,
    0.457142857142857, 0.520634920634921, 0.457142857142857,
    0.253968253968254, 0.0285714285714286)
    .finished();
  EXPECT(assert_equal(expected, actual));

  // Assert that multiplying with all ones gives the correct integral (2.0)
  EXPECT_DOUBLES_EQUAL(2.0, actual.array().sum(), 1e-9);

  // Integrating f' over [-1,1] should give f(1) - f(-1)
  Vector fp = Chebyshev2::vector(fprime, N);
  double expectedF = f(1) - f(-1);
  double actualW = actual * fp;
  EXPECT_DOUBLES_EQUAL(expectedF, actualW, 1e-9);

  // We can calculate an alternate set of weights using the integration matrix:
  Matrix P = Chebyshev2::IntegrationMatrix(N);
  Weights p7 = P.row(N-1);

  // Check that the two sets of weights give the same results
  EXPECT_DOUBLES_EQUAL(expectedF, p7 * fp, 1e-9);

  // And same for integrate f itself:
  Vector fvals = Chebyshev2::vector(f, N);
  EXPECT_DOUBLES_EQUAL(p7*fvals, actual * fvals, 1e-9);
}

// Check N=8
TEST(Chebyshev2, IntegrationWeights8) {
  const size_t N = 8;
  Weights actual = Chebyshev2::IntegrationWeights(N, -1, 1);
  Weights expected = (Weights(N) << 0.0204081632653061, 0.190141007218208,
    0.352242423718159, 0.437208405798326, 0.437208405798326,
    0.352242423718159, 0.190141007218208, 0.0204081632653061)
    .finished();
  EXPECT(assert_equal(expected, actual));
  EXPECT_DOUBLES_EQUAL(2.0, actual.array().sum(), 1e-9);
}

//******************************************************************************
TEST(Chebyshev2, DoubleIntegrationWeights) {
  const size_t N = 7;
  const double a = 0, b = 10;
  // Let's integrate constant twice get a test case:
  Matrix P = Chebyshev2::IntegrationMatrix(N, a, b);
  auto ones = Vector::Ones(N);
  
  // Check the sum which should be 0.5*t^2 | [0,b] = b^2/2:
  Weights w = Chebyshev2::DoubleIntegrationWeights(N, a, b);
  EXPECT_DOUBLES_EQUAL(b*b/2, w * ones, 1e-9);
}

TEST(Chebyshev2, DoubleIntegrationWeights2) {
  const size_t N = 8;
  const double a = 0, b = 3;
  // Let's integrate constant twice get a test case:
  Matrix P = Chebyshev2::IntegrationMatrix(N, a, b);
  auto ones = Vector::Ones(N);
  
  // Check the sum which should be 0.5*t^2 | [0,b] = b^2/2:
  Weights w = Chebyshev2::DoubleIntegrationWeights(N, a, b);
  EXPECT_DOUBLES_EQUAL(b*b/2, w * ones, 1e-9);
}

//******************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
//******************************************************************************
