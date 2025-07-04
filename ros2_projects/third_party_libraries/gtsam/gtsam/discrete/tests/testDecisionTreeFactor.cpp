/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/*
 * testDecisionTreeFactor.cpp
 *
 *  @date Feb 5, 2012
 *  @author Frank Dellaert
 *  @author Duy-Nguyen Ta
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/serializationTestHelpers.h>
#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/discrete/DiscreteDistribution.h>
#include <gtsam/discrete/DiscreteFactor.h>
#include <gtsam/discrete/Signature.h>
#include <gtsam/inference/Key.h>
#include <gtsam/inference/Ordering.h>

using namespace std;
using namespace gtsam;

/** Convert Signature into CPT */
DecisionTreeFactor create(const Signature& signature) {
  DecisionTreeFactor p(signature.discreteKeys(), signature.cpt());
  return p;
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, ConstructorsMatch) {
  // Declare two keys
  DiscreteKey X(0, 2), Y(1, 3);

  // Create with vector and with string
  const std::vector<double> table{2, 5, 3, 6, 4, 7};
  DecisionTreeFactor f1({X, Y}, table);
  DecisionTreeFactor f2({X, Y}, "2 5 3 6 4 7");
  EXPECT(assert_equal(f1, f2));
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, constructors) {
  // Declare a bunch of keys
  DiscreteKey X(0, 2), Y(1, 3), Z(2, 2);

  // Create factors
  DecisionTreeFactor f1(X, {2, 8});
  DecisionTreeFactor f2(X & Y, "2 5 3 6 4 7");
  DecisionTreeFactor f3(X & Y & Z, "2 5 3 6 4 7 25 55 35 65 45 75");
  EXPECT_LONGS_EQUAL(1, f1.size());
  EXPECT_LONGS_EQUAL(2, f2.size());
  EXPECT_LONGS_EQUAL(3, f3.size());

  DiscreteValues x121{{0, 1}, {1, 2}, {2, 1}};
  EXPECT_DOUBLES_EQUAL(8, f1(x121), 1e-9);
  EXPECT_DOUBLES_EQUAL(7, f2(x121), 1e-9);
  EXPECT_DOUBLES_EQUAL(75, f3(x121), 1e-9);

  // Assert that error = -log(value)
  EXPECT_DOUBLES_EQUAL(-log(f1(x121)), f1.error(x121), 1e-9);

  // Construct from DiscreteConditional
  DiscreteConditional conditional(X | Y = "1/1 2/3 1/4");
  DecisionTreeFactor f4(conditional);
  EXPECT_DOUBLES_EQUAL(0.8, f4(x121), 1e-9);
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, Error) {
  // Declare a bunch of keys
  DiscreteKey X(0, 2), Y(1, 3), Z(2, 2);

  // Create factors
  DecisionTreeFactor f(X & Y & Z, "2 5 3 6 4 7 25 55 35 65 45 75");

  auto errors = f.errorTree();
  // regression
  AlgebraicDecisionTree<Key> expected(
      {X, Y, Z},
      vector<double>{-0.69314718, -1.6094379, -1.0986123, -1.7917595,
                     -1.3862944, -1.9459101, -3.2188758, -4.0073332, -3.5553481,
                     -4.1743873, -3.8066625, -4.3174881});
  EXPECT(assert_equal(expected, errors, 1e-6));
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, multiplication) {
  DiscreteKey v0(0, 2), v1(1, 2), v2(2, 2);

  // Multiply with a DiscreteDistribution, i.e., Bayes Law!
  DiscreteDistribution prior(v1 % "1/3");
  DecisionTreeFactor f1(v0 & v1, "1 2 3 4");
  DecisionTreeFactor expected(v0 & v1, "0.25 1.5 0.75 3");
  CHECK(assert_equal(expected, static_cast<DecisionTreeFactor>(prior) * f1));
  CHECK(assert_equal(expected, f1 * prior));

  // Multiply two factors
  DecisionTreeFactor f2(v1 & v2, "5 6 7 8");
  DecisionTreeFactor actual = f1 * f2;
  DecisionTreeFactor expected2(v0 & v1 & v2, "5 6 14 16 15 18 28 32");
  CHECK(assert_equal(expected2, actual));
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, Divide) {
  DiscreteKey A(0, 2), S(1, 2);
  DecisionTreeFactor pA = create(A % "99/1"), pS = create(S % "50/50");
  DecisionTreeFactor joint = pA * pS;

  DecisionTreeFactor s = joint / pA;

  // Factors are not equal due to difference in keys
  EXPECT(assert_inequal(pS, s));

  // The underlying data should be the same
#ifdef GTSAM_DT_MERGING
  using ADT = AlgebraicDecisionTree<Key>;
  EXPECT(assert_equal(ADT(pS), ADT(s)));
#endif

  KeySet keys(joint.keys());
  keys.insert(pA.keys().begin(), pA.keys().end());
  EXPECT(assert_inequal(KeySet(pS.keys()), keys));
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, sum_max) {
  DiscreteKey v0(0, 3), v1(1, 2);
  DecisionTreeFactor f1(v0 & v1, "1 2  3 4  5 6");

  DecisionTreeFactor expected(v1, "9 12");
  auto actual = std::dynamic_pointer_cast<DecisionTreeFactor>(f1.sum(1));
  CHECK(actual);
  CHECK(assert_equal(expected, *actual, 1e-5));

  DecisionTreeFactor expected2(v1, "5 6");
  auto actual2 = std::dynamic_pointer_cast<DecisionTreeFactor>(f1.max(1));
  CHECK(actual2);
  CHECK(assert_equal(expected2, *actual2));

  DecisionTreeFactor f2(v1 & v0, "1 2  3 4  5 6");
  auto actual22 = std::dynamic_pointer_cast<DecisionTreeFactor>(f2.sum(1));
  CHECK(actual22);
}

/* ************************************************************************* */
// Check enumerate yields the correct list of assignment/value pairs.
TEST(DecisionTreeFactor, enumerate) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
  auto actual = f.enumerate();
  std::vector<std::pair<DiscreteValues, double>> expected;
  DiscreteValues values;
  for (size_t a : {0, 1, 2}) {
    for (size_t b : {0, 1}) {
      values[12] = a;
      values[5] = b;
      expected.emplace_back(values, f(values));
    }
  }
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Test if restricting a factor based on DiscreteValues works.
TEST(DecisionTreeFactor, Restrict) {
  // Test for restricting a single value from multiple values.
  DiscreteKey A(12, 2), B(5, 3);
  DecisionTreeFactor f1(A & B, "1 2  3 4  5 6");
  DiscreteValues fixedValues = {{A.first, 1}};

  DecisionTreeFactor restricted_f1 =
      *std::static_pointer_cast<DecisionTreeFactor>(f1.restrict(fixedValues));

  DecisionTreeFactor expected_f1(B, "4 5 6");
  EXPECT(assert_equal(expected_f1, restricted_f1));

  // Test for restricting a multiple value from multiple values.
  DiscreteKey C(91, 2);
  DecisionTreeFactor f2(A & B & C, "1 2  3 4  5 6  7 8  9 10  11 12");
  fixedValues = {{A.first, 0}, {B.first, 2}};

  DecisionTreeFactor restricted_f2 =
      *std::static_pointer_cast<DecisionTreeFactor>(f2.restrict(fixedValues));

  DecisionTreeFactor expected_f2(C, "5 6");
  EXPECT(assert_equal(expected_f2, restricted_f2));

  // Edge case of restricting a single value when it is the only value.
  DecisionTreeFactor f3(A, "50 100");
  fixedValues = {{A.first, 1}};  // select 100

  DecisionTreeFactor restricted_f3 =
      *std::static_pointer_cast<DecisionTreeFactor>(f3.restrict(fixedValues));

  EXPECT_LONGS_EQUAL(0, restricted_f3.discreteKeys().size());
  // There should only be 1 value which is 100
  EXPECT_LONGS_EQUAL(1, restricted_f3.nrValues());
  EXPECT_LONGS_EQUAL(1, restricted_f3.nrLeaves());
  EXPECT_DOUBLES_EQUAL(100, restricted_f3.evaluate(DiscreteValues()), 1e-9);
}

namespace pruning_fixture {

DiscreteKey A(1, 2), B(2, 2), C(3, 2);
DecisionTreeFactor f(A& B& C, "1 5 3 7 2 6 4 8");

DiscreteKey D(4, 2);
DecisionTreeFactor factor(
    D& C & B & A,
    "0.0 0.0 0.0 0.60658897 0.61241912 0.61241969 0.61247685 0.61247742 0.0 "
    "0.0 0.0 0.99995287 1.0 1.0 1.0 1.0");

}  // namespace pruning_fixture

/* ************************************************************************* */
// Check if computing the correct threshold works.
TEST(DecisionTreeFactor, ComputeThreshold) {
  using namespace pruning_fixture;

  // Only keep the leaves with the top 5 values.
  double threshold = f.computeThreshold(5);
  EXPECT_DOUBLES_EQUAL(4.0, threshold, 1e-9);

  // Check for more extreme pruning where we only keep the top 2 leaves
  threshold = f.computeThreshold(2);
  EXPECT_DOUBLES_EQUAL(7.0, threshold, 1e-9);

  threshold = factor.computeThreshold(5);
  EXPECT_DOUBLES_EQUAL(0.99995287, threshold, 1e-9);

  threshold = factor.computeThreshold(3);
  EXPECT_DOUBLES_EQUAL(1.0, threshold, 1e-9);

  threshold = factor.computeThreshold(6);
  EXPECT_DOUBLES_EQUAL(0.61247742, threshold, 1e-9);
}

/* ************************************************************************* */
// Check pruning of the decision tree works as expected.
TEST(DecisionTreeFactor, Prune) {
  using namespace pruning_fixture;

  // Only keep the leaves with the top 5 values.
  size_t maxNrAssignments = 5;
  auto pruned5 = f.prune(maxNrAssignments);

  // Pruned leaves should be 0
  DecisionTreeFactor expected(A & B & C, "0 5 0 7 0 6 4 8");
  EXPECT(assert_equal(expected, pruned5));

  // Check for more extreme pruning where we only keep the top 2 leaves
  maxNrAssignments = 2;
  auto pruned2 = f.prune(maxNrAssignments);
  DecisionTreeFactor expected2(A & B & C, "0 0 0 7 0 0 0 8");
  EXPECT(assert_equal(expected2, pruned2));

  DecisionTreeFactor expected3(D & C & B & A,
                               "0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 "
                               "0.999952870000 1.0 1.0 1.0 1.0");
  maxNrAssignments = 5;
  auto pruned3 = factor.prune(maxNrAssignments);
  EXPECT(assert_equal(expected3, pruned3));

  // Edge case where the number of hypotheses are less than maxNrAssignments
  DecisionTreeFactor f(A, "0.50001 0.49999");
  auto pruned4 = f.prune(10);
  DecisionTreeFactor expected4(A, "0.50001 0.49999");
  EXPECT(assert_equal(expected4, pruned4));
}

/* ************************************************************************** */
// Asia Bayes Network
/* ************************************************************************** */

#define DISABLE_DOT

void maybeSaveDotFile(const DecisionTreeFactor& f, const string& filename) {
#ifndef DISABLE_DOT
  std::vector<std::string> names = {"A", "S", "T", "L", "B", "E", "X", "D"};
  auto formatter = [&](Key key) { return names[key]; };
  f.dot(filename, formatter, true);
#endif
}

/* ************************************************************************* */
// test Asia Joint
TEST(DecisionTreeFactor, joint) {
  DiscreteKey A(0, 2), S(1, 2), T(2, 2), L(3, 2), B(4, 2), E(5, 2), X(6, 2),
      D(7, 2);

  gttic_(asiaCPTs);
  DecisionTreeFactor pA = create(A % "99/1");
  DecisionTreeFactor pS = create(S % "50/50");
  DecisionTreeFactor pT = create(T | A = "99/1 95/5");
  DecisionTreeFactor pL = create(L | S = "99/1 90/10");
  DecisionTreeFactor pB = create(B | S = "70/30 40/60");
  DecisionTreeFactor pE = create((E | T, L) = "F T T T");
  DecisionTreeFactor pX = create(X | E = "95/5 2/98");
  DecisionTreeFactor pD = create((D | E, B) = "9/1 2/8 3/7 1/9");

  // Create joint
  gttic_(asiaJoint);
  DecisionTreeFactor joint = pA;
  maybeSaveDotFile(joint, "Asia-A");
  joint = joint * pS;
  maybeSaveDotFile(joint, "Asia-AS");
  joint = joint * pT;
  maybeSaveDotFile(joint, "Asia-AST");
  joint = joint * pL;
  maybeSaveDotFile(joint, "Asia-ASTL");
  joint = joint * pB;
  maybeSaveDotFile(joint, "Asia-ASTLB");
  joint = joint * pE;
  maybeSaveDotFile(joint, "Asia-ASTLBE");
  joint = joint * pX;
  maybeSaveDotFile(joint, "Asia-ASTLBEX");
  joint = joint * pD;
  maybeSaveDotFile(joint, "Asia-ASTLBEXD");

  // Check that discrete keys are as expected
  EXPECT(assert_equal(joint.discreteKeys(), {A, S, T, L, B, E, X, D}));

  // Check that summing out variables maintains the keys even if merged, as is
  // the case with S.
  auto noAB = joint.sum(Ordering{A.first, B.first});
  EXPECT(assert_equal(noAB->discreteKeys(), {S, T, L, E, X, D}));
}

/* ************************************************************************* */
TEST(DecisionTreeFactor, DotWithNames) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
  auto formatter = [](Key key) { return key == 12 ? "A" : "B"; };

  for (bool showZero : {true, false}) {
    string actual = f.dot(formatter, showZero);
    // pretty weak test, as ids are pointers and not stable across platforms.
    string expected = "digraph G {";
    EXPECT(actual.substr(0, 11) == expected);
  }
}

/* ************************************************************************* */
// Check markdown representation looks as expected.
TEST(DecisionTreeFactor, markdown) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
  string expected =
      "|A|B|value|\n"
      "|:-:|:-:|:-:|\n"
      "|0|0|1|\n"
      "|0|1|2|\n"
      "|1|0|3|\n"
      "|1|1|4|\n"
      "|2|0|5|\n"
      "|2|1|6|\n";
  auto formatter = [](Key key) { return key == 12 ? "A" : "B"; };
  string actual = f.markdown(formatter);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check markdown representation with a value formatter.
TEST(DecisionTreeFactor, markdownWithValueFormatter) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
  string expected =
      "|A|B|value|\n"
      "|:-:|:-:|:-:|\n"
      "|Zero|-|1|\n"
      "|Zero|+|2|\n"
      "|One|-|3|\n"
      "|One|+|4|\n"
      "|Two|-|5|\n"
      "|Two|+|6|\n";
  auto keyFormatter = [](Key key) { return key == 12 ? "A" : "B"; };
  DecisionTreeFactor::Names names{{12, {"Zero", "One", "Two"}},
                                  {5, {"-", "+"}}};
  string actual = f.markdown(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
// Check html representation with a value formatter.
TEST(DecisionTreeFactor, htmlWithValueFormatter) {
  DiscreteKey A(12, 3), B(5, 2);
  DecisionTreeFactor f(A & B, "1 2  3 4  5 6");
  string expected =
      "<div>\n"
      "<table class='DecisionTreeFactor'>\n"
      "  <thead>\n"
      "    <tr><th>A</th><th>B</th><th>value</th></tr>\n"
      "  </thead>\n"
      "  <tbody>\n"
      "    <tr><th>Zero</th><th>-</th><td>1</td></tr>\n"
      "    <tr><th>Zero</th><th>+</th><td>2</td></tr>\n"
      "    <tr><th>One</th><th>-</th><td>3</td></tr>\n"
      "    <tr><th>One</th><th>+</th><td>4</td></tr>\n"
      "    <tr><th>Two</th><th>-</th><td>5</td></tr>\n"
      "    <tr><th>Two</th><th>+</th><td>6</td></tr>\n"
      "  </tbody>\n"
      "</table>\n"
      "</div>";
  auto keyFormatter = [](Key key) { return key == 12 ? "A" : "B"; };
  DecisionTreeFactor::Names names{{12, {"Zero", "One", "Two"}},
                                  {5, {"-", "+"}}};
  string actual = f.html(keyFormatter, names);
  EXPECT(actual == expected);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
