/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   HybridNonlinearFactor.h
 * @brief  A set of nonlinear factors indexed by a set of discrete keys.
 * @author Varun Agrawal
 * @date   Sep 12, 2024
 */

#include <gtsam/discrete/DecisionTreeFactor.h>
#include <gtsam/hybrid/HybridNonlinearFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <memory>

namespace gtsam {

/* *******************************************************************************/
struct HybridNonlinearFactor::ConstructorHelper {
  KeyVector continuousKeys;   // Continuous keys extracted from factors
  DiscreteKeys discreteKeys;  // Discrete keys provided to the constructors
  FactorValuePairs factorTree;

  void copyOrCheckContinuousKeys(const NoiseModelFactor::shared_ptr& factor) {
    if (!factor) return;
    if (continuousKeys.empty()) {
      continuousKeys = factor->keys();
    } else if (factor->keys() != continuousKeys) {
      throw std::runtime_error(
          "HybridNonlinearFactor: all factors should have the same keys!");
    }
  }

  ConstructorHelper(const DiscreteKey& discreteKey,
                    const std::vector<NoiseModelFactor::shared_ptr>& factors)
      : discreteKeys({discreteKey}) {
    std::vector<NonlinearFactorValuePair> pairs;
    // Extract continuous keys from the first non-null factor
    for (const auto& factor : factors) {
      pairs.emplace_back(factor, 0.0);
      copyOrCheckContinuousKeys(factor);
    }
    factorTree = FactorValuePairs({discreteKey}, pairs);
  }

  ConstructorHelper(const DiscreteKey& discreteKey,
                    const std::vector<NonlinearFactorValuePair>& pairs)
      : discreteKeys({discreteKey}) {
    // Extract continuous keys from the first non-null factor
    for (const auto& pair : pairs) {
      copyOrCheckContinuousKeys(pair.first);
    }
    factorTree = FactorValuePairs({discreteKey}, pairs);
  }

  ConstructorHelper(const DiscreteKeys& discreteKeys,
                    const FactorValuePairs& factorPairs)
      : discreteKeys(discreteKeys), factorTree(factorPairs) {
    // Extract continuous keys from the first non-null factor
    factorPairs.visit([&](const NonlinearFactorValuePair& pair) {
      copyOrCheckContinuousKeys(pair.first);
    });
  }
};

/* *******************************************************************************/
HybridNonlinearFactor::HybridNonlinearFactor(const ConstructorHelper& helper)
    : Base(helper.continuousKeys, helper.discreteKeys),
      factors_(helper.factorTree) {}

HybridNonlinearFactor::HybridNonlinearFactor(
    const DiscreteKey& discreteKey,
    const std::vector<NoiseModelFactor::shared_ptr>& factors)
    : HybridNonlinearFactor(ConstructorHelper(discreteKey, factors)) {}

HybridNonlinearFactor::HybridNonlinearFactor(
    const DiscreteKey& discreteKey,
    const std::vector<NonlinearFactorValuePair>& pairs)
    : HybridNonlinearFactor(ConstructorHelper(discreteKey, pairs)) {}

HybridNonlinearFactor::HybridNonlinearFactor(const DiscreteKeys& discreteKeys,
                                             const FactorValuePairs& factors)
    : HybridNonlinearFactor(ConstructorHelper(discreteKeys, factors)) {}

/* *******************************************************************************/
AlgebraicDecisionTree<Key> HybridNonlinearFactor::errorTree(
    const Values& continuousValues) const {
  // functor to convert from sharedFactor to double error value.
  auto errorFunc =
      [continuousValues](const std::pair<sharedFactor, double>& f) {
        auto [factor, val] = f;
        return factor ? factor->error(continuousValues) + val
                      : std::numeric_limits<double>::infinity();
      };
  return {factors_, errorFunc};
}

/* *******************************************************************************/
double HybridNonlinearFactor::error(
    const Values& continuousValues,
    const DiscreteValues& discreteValues) const {
  // Retrieve the factor corresponding to the assignment in discreteValues.
  auto [factor, val] = factors_(discreteValues);
  // Compute the error for the selected factor
  const double factorError = factor->error(continuousValues);
  return factorError + val;
}

/* *******************************************************************************/
double HybridNonlinearFactor::error(const HybridValues& values) const {
  return error(values.nonlinear(), values.discrete());
}

/* *******************************************************************************/
size_t HybridNonlinearFactor::dim() const {
  const auto assignments = DiscreteValues::CartesianProduct(discreteKeys_);
  auto [factor, val] = factors_(assignments.at(0));
  return factor->dim();
}

/* *******************************************************************************/
void HybridNonlinearFactor::print(const std::string& s,
                                  const KeyFormatter& keyFormatter) const {
  std::cout << (s.empty() ? "" : s + " ");
  Base::print("", keyFormatter);
  std::cout << "\nHybridNonlinearFactor\n";
  auto valueFormatter = [&keyFormatter](const std::pair<sharedFactor, double>& v) {
    auto [factor, val] = v;
    if (factor) {
      RedirectCout rd;
      factor->print("", keyFormatter);
      return rd.str();
    } else {
      return std::string("nullptr");
    }
  };
  factors_.print("", keyFormatter, valueFormatter);
}

/* *******************************************************************************/
bool HybridNonlinearFactor::equals(const HybridFactor& other,
                                   double tol) const {
  // We attempt a dynamic cast from HybridFactor to HybridNonlinearFactor. If
  // it fails, return false.
  if (!dynamic_cast<const HybridNonlinearFactor*>(&other)) return false;

  // If the cast is successful, we'll properly construct a
  // HybridNonlinearFactor object from `other`
  const HybridNonlinearFactor& f(
      static_cast<const HybridNonlinearFactor&>(other));

  // Ensure that this HybridNonlinearFactor and `f` have the same `factors_`.
  auto compare = [tol](const std::pair<sharedFactor, double>& a,
                       const std::pair<sharedFactor, double>& b) {
    return a.first->equals(*b.first, tol) && (a.second == b.second);
  };
  if (!factors_.equals(f.factors_, compare)) return false;

  // If everything above passes, and the keys_ and discreteKeys_
  // member variables are identical, return true.
  return (std::equal(keys_.begin(), keys_.end(), f.keys().begin()) &&
          (discreteKeys_ == f.discreteKeys_));
}

/* *******************************************************************************/
GaussianFactor::shared_ptr HybridNonlinearFactor::linearize(
    const Values& continuousValues,
    const DiscreteValues& discreteValues) const {
  auto factor = factors_(discreteValues).first;
  return factor->linearize(continuousValues);
}

/* *******************************************************************************/
std::shared_ptr<HybridGaussianFactor> HybridNonlinearFactor::linearize(
    const Values& continuousValues) const {
  // functional to linearize each factor in the decision tree
  auto linearizeDT =
      [continuousValues](
          const std::pair<sharedFactor, double>& f) -> GaussianFactorValuePair {
    auto [factor, val] = f;
    // Check if valid factor. If not, return null and infinite error.
    if (!factor) {
      return {nullptr, std::numeric_limits<double>::infinity()};
    }

    if (auto gaussian = std::dynamic_pointer_cast<noiseModel::Gaussian>(
            factor->noiseModel())) {
      return {factor->linearize(continuousValues),
              val + gaussian->negLogConstant()};
    } else {
      throw std::runtime_error(
          "HybridNonlinearFactor: linearize() only supports NoiseModelFactors "
          "with Gaussian (or derived) noise models.");
    }
  };

  DecisionTree<Key, std::pair<GaussianFactor::shared_ptr, double>>
      linearized_factors(factors_, linearizeDT);

  return std::make_shared<HybridGaussianFactor>(discreteKeys_,
                                                linearized_factors);
}

/* *******************************************************************************/
HybridNonlinearFactor::shared_ptr HybridNonlinearFactor::prune(
    const DecisionTreeFactor& discreteProbs) const {
  // Find keys in discreteProbs.keys() but not in this->keys():
  std::set<Key> mine(this->keys().begin(), this->keys().end());
  std::set<Key> theirs(discreteProbs.keys().begin(),
                       discreteProbs.keys().end());
  std::vector<Key> diff;
  std::set_difference(theirs.begin(), theirs.end(), mine.begin(), mine.end(),
                      std::back_inserter(diff));

  // Find maximum probability value for every combination of our keys.
  Ordering keys(diff);
  auto max = discreteProbs.max(keys);

  // Check the max value for every combination of our keys.
  // If the max value is 0.0, we can prune the corresponding conditional.
  auto pruner =
      [&](const Assignment<Key>& choices,
          const NonlinearFactorValuePair& pair) -> NonlinearFactorValuePair {
    if (max->evaluate(choices) == 0.0)
      return {nullptr, std::numeric_limits<double>::infinity()};
    else
      return pair;
  };

  FactorValuePairs prunedFactors = factors().apply(pruner);
  return std::make_shared<HybridNonlinearFactor>(discreteKeys(), prunedFactors);
}

/* ************************************************************************ */
std::shared_ptr<Factor> HybridNonlinearFactor::restrict(
    const DiscreteValues& assignment) const {
  auto restrictedFactors = factors_.restrict(assignment);
  auto filtered = assignment.filter(discreteKeys_);
  if (filtered.size() == discreteKeys_.size()) {
    auto [nonlinearFactor, val] = factors_(filtered);
    return nonlinearFactor;
  } else {
    auto remainingKeys = assignment.missingKeys(discreteKeys());
    return std::make_shared<HybridNonlinearFactor>(remainingKeys,
                                                   factors_.restrict(filtered));
  }
}

/* ************************************************************************ */

}  // namespace gtsam
