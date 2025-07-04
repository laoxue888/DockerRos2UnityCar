//*************************************************************************
// nonlinear
//*************************************************************************

namespace gtsam {

#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3Fisheye.h>
#include <gtsam/geometry/Cal3Unified.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/CalibratedCamera.h>
#include <gtsam/geometry/EssentialMatrix.h>
#include <gtsam/geometry/FundamentalMatrix.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Similarity2.h>
#include <gtsam/geometry/Similarity3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/SO4.h>
#include <gtsam/geometry/SOn.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <gtsam/geometry/Unit3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <gtsam/nonlinear/GraphvizFormatting.h>
class GraphvizFormatting : gtsam::DotWriter {
  GraphvizFormatting();

  enum Axis { X, Y, Z, NEGX, NEGY, NEGZ };
  gtsam::GraphvizFormatting::Axis paperHorizontalAxis;
  gtsam::GraphvizFormatting::Axis paperVerticalAxis;

  double scale;
  bool mergeSimilarFactors;
};

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
class NonlinearFactorGraph {
  NonlinearFactorGraph();
  NonlinearFactorGraph(const gtsam::NonlinearFactorGraph& graph);

  // FactorGraph
  void print(string s = "NonlinearFactorGraph: ",
             const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  bool equals(const gtsam::NonlinearFactorGraph& other, double tol) const;
  size_t size() const;
  bool empty() const;
  void remove(size_t i);
  void replace(size_t i, gtsam::NonlinearFactor* factors);
  void resize(size_t size);
  size_t nrFactors() const;
  gtsam::NonlinearFactor* at(size_t idx) const;
  void push_back(const gtsam::NonlinearFactorGraph& factors);
  void push_back(gtsam::NonlinearFactor* factor);
  void add(gtsam::NonlinearFactor* factor);
  bool exists(size_t idx) const;
  gtsam::KeySet keys() const;
  gtsam::KeyVector keyVector() const;

  template <T = {double,
                 gtsam::Vector,
                 gtsam::Point2,
                 gtsam::StereoPoint2,
                 gtsam::Point3,
                 gtsam::Rot2,
                 gtsam::SO3,
                 gtsam::SO4,
                 gtsam::Rot3,
                 gtsam::Pose2,
                 gtsam::Pose3,
                 gtsam::Similarity2,
                 gtsam::Similarity3,
                 gtsam::Cal3_S2,
                 gtsam::Cal3f,
                 gtsam::Cal3Bundler,
                 gtsam::Cal3Fisheye,
                 gtsam::Cal3Unified,
                 gtsam::CalibratedCamera,
                 gtsam::EssentialMatrix,
                 gtsam::FundamentalMatrix,
                 gtsam::SimpleFundamentalMatrix,
                 gtsam::PinholeCamera<gtsam::Cal3_S2>,
                 gtsam::PinholeCamera<gtsam::Cal3f>,
                 gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                 gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                 gtsam::PinholeCamera<gtsam::Cal3Unified>,
                 gtsam::PinholeCamera<gtsam::CalibratedCamera>,
                 gtsam::imuBias::ConstantBias}>
  void addPrior(gtsam::Key key, const T& prior,
                const gtsam::noiseModel::Base* noiseModel);

  // NonlinearFactorGraph
  void printErrors(const gtsam::Values& values,
                   const string& str = "NonlinearFactorGraph: ",
                   const gtsam::KeyFormatter& keyFormatter =
                       gtsam::DefaultKeyFormatter) const;
  double error(const gtsam::Values& values) const;
  double probPrime(const gtsam::Values& values) const;
  gtsam::Ordering orderingCOLAMD() const;
  // Ordering* orderingCOLAMDConstrained(const gtsam::Values& c, const
  // std::map<gtsam::Key,int>& constraints) const;
  gtsam::GaussianFactorGraph* linearize(const gtsam::Values& linearizationPoint) const;
  gtsam::NonlinearFactorGraph clone() const;

  string dot(
      const gtsam::Values& values,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::GraphvizFormatting& writer = gtsam::GraphvizFormatting());
  void saveGraph(
      const string& s, const gtsam::Values& values,
      const gtsam::KeyFormatter& keyFormatter = gtsam::DefaultKeyFormatter,
      const gtsam::GraphvizFormatting& writer = gtsam::GraphvizFormatting()) const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NonlinearFactor : gtsam::Factor {
  // Factor base class
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  // NonlinearFactor
  bool equals(const gtsam::NonlinearFactor& f, double tol) const;
  double error(const gtsam::Values& c) const;
  double error(const gtsam::HybridValues& c) const;
  size_t dim() const;
  bool active(const gtsam::Values& c) const;
  gtsam::GaussianFactor* linearize(const gtsam::Values& c) const;
  gtsam::NonlinearFactor* clone() const;
  gtsam::NonlinearFactor* rekey(const gtsam::KeyVector& newKeys) const;
};

#include <gtsam/nonlinear/NonlinearFactor.h>
virtual class NoiseModelFactor : gtsam::NonlinearFactor {
  bool equals(const gtsam::NoiseModelFactor& f, double tol) const;
  gtsam::noiseModel::Base* noiseModel() const;
  gtsam::NoiseModelFactor* cloneWithNewNoiseModel(gtsam::noiseModel::Base* newNoise) const;
  gtsam::Vector unwhitenedError(const gtsam::Values& x) const;
  gtsam::Vector whitenedError(const gtsam::Values& c) const;
};

#include <gtsam/nonlinear/Marginals.h>
class Marginals {
  Marginals(const gtsam::NonlinearFactorGraph& graph,
            const gtsam::Values& solution);
  Marginals(const gtsam::GaussianFactorGraph& gfgraph,
            const gtsam::Values& solution);
  Marginals(const gtsam::GaussianFactorGraph& gfgraph,
            const gtsam::VectorValues& solutionvec);

  void print(string s = "Marginals: ", const gtsam::KeyFormatter& keyFormatter =
                                           gtsam::DefaultKeyFormatter) const;
  gtsam::Matrix marginalCovariance(size_t variable) const;
  gtsam::Matrix marginalInformation(size_t variable) const;
  gtsam::JointMarginal jointMarginalCovariance(
      const gtsam::KeyVector& variables) const;
  gtsam::JointMarginal jointMarginalInformation(
      const gtsam::KeyVector& variables) const;
};

class JointMarginal {
  gtsam::Matrix at(size_t iVariable, size_t jVariable) const;
  gtsam::Matrix fullMatrix() const;
  void print(string s = "", gtsam::KeyFormatter keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/nonlinear/LinearContainerFactor.h>
virtual class LinearContainerFactor : gtsam::NonlinearFactor {
  LinearContainerFactor(gtsam::GaussianFactor* factor,
                        const gtsam::Values& linearizationPoint);
  LinearContainerFactor(gtsam::GaussianFactor* factor);

  gtsam::GaussianFactor* factor() const;
  //  const std::optional<Values>& linearizationPoint() const;

  bool isJacobian() const;
  gtsam::JacobianFactor* toJacobian() const;
  gtsam::HessianFactor* toHessian() const;

  static gtsam::NonlinearFactorGraph ConvertLinearGraph(
      const gtsam::GaussianFactorGraph& linear_graph,
      const gtsam::Values& linearizationPoint);

  static gtsam::NonlinearFactorGraph ConvertLinearGraph(
      const gtsam::GaussianFactorGraph& linear_graph);

  // enabling serialization functionality
  void serializable() const;
};  // \class LinearContainerFactor

// Summarization functionality
//#include <gtsam/nonlinear/summarization.h>
//
//// Uses partial QR approach by default
// gtsam::GaussianFactorGraph summarize(
//    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
//    const gtsam::KeySet& saved_keys);
//
// gtsam::NonlinearFactorGraph summarizeAsNonlinearContainer(
//    const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values,
//    const gtsam::KeySet& saved_keys);

//*************************************************************************
// Nonlinear optimizers
//*************************************************************************
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
virtual class NonlinearOptimizerParams {
  NonlinearOptimizerParams();
  void print(string str = "") const;

  int getMaxIterations() const;
  double getRelativeErrorTol() const;
  double getAbsoluteErrorTol() const;
  double getErrorTol() const;
  string getVerbosity() const;

  void setMaxIterations(int value);
  void setRelativeErrorTol(double value);
  void setAbsoluteErrorTol(double value);
  void setErrorTol(double value);
  void setVerbosity(string src);

  string getLinearSolverType() const;
  void setLinearSolverType(string solver);

  void setIterativeParams(gtsam::IterativeOptimizationParameters* params);
  void setOrdering(const gtsam::Ordering& ordering);
  string getOrderingType() const;
  void setOrderingType(string ordering);

  bool isMultifrontal() const;
  bool isSequential() const;
  bool isCholmod() const;
  bool isIterative() const;

  // This only applies to python since matlab does not have lambda machinery.
  gtsam::NonlinearOptimizerParams::IterationHook iterationHook;
};

bool checkConvergence(double relativeErrorTreshold,
                      double absoluteErrorTreshold, double errorThreshold,
                      double currentError, double newError);
bool checkConvergence(const gtsam::NonlinearOptimizerParams& params,
                      double currentError, double newError);

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
virtual class GaussNewtonParams : gtsam::NonlinearOptimizerParams {
  GaussNewtonParams();
};

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
virtual class LevenbergMarquardtParams : gtsam::NonlinearOptimizerParams {
  LevenbergMarquardtParams();

  bool getDiagonalDamping() const;
  double getlambdaFactor() const;
  double getlambdaInitial() const;
  double getlambdaLowerBound() const;
  double getlambdaUpperBound() const;
  bool getUseFixedLambdaFactor();
  string getLogFile() const;
  string getVerbosityLM() const;

  void setDiagonalDamping(bool flag);
  void setlambdaFactor(double value);
  void setlambdaInitial(double value);
  void setlambdaLowerBound(double value);
  void setlambdaUpperBound(double value);
  void setUseFixedLambdaFactor(bool flag);
  void setLogFile(string s);
  void setVerbosityLM(string s);

  static gtsam::LevenbergMarquardtParams LegacyDefaults();
  static gtsam::LevenbergMarquardtParams CeresDefaults();

  static gtsam::LevenbergMarquardtParams EnsureHasOrdering(
      gtsam::LevenbergMarquardtParams params,
      const gtsam::NonlinearFactorGraph& graph);
  static gtsam::LevenbergMarquardtParams ReplaceOrdering(
      gtsam::LevenbergMarquardtParams params, const gtsam::Ordering& ordering);
};

#include <gtsam/nonlinear/DoglegOptimizer.h>
virtual class DoglegParams : gtsam::NonlinearOptimizerParams {
  DoglegParams();

  double getDeltaInitial() const;
  string getVerbosityDL() const;

  void setDeltaInitial(double deltaInitial) const;
  void setVerbosityDL(string verbosityDL) const;
};

#include <gtsam/nonlinear/GncParams.h>
enum GncLossType {
  GM /*Geman McClure*/,
  TLS /*Truncated least squares*/
};

template<PARAMS>
virtual class GncParams {
  GncParams(const PARAMS& baseOptimizerParams);
  GncParams();
  PARAMS baseOptimizerParams;
  gtsam::GncLossType lossType;
  size_t maxIterations;
  double muStep;
  double relativeCostTol;
  double weightsTol;
  gtsam::This::Verbosity verbosity;
  gtsam::This::IndexVector knownInliers;
  gtsam::This::IndexVector knownOutliers;

  void setLossType(const gtsam::GncLossType type);
  void setMaxIterations(const size_t maxIter);
  void setMuStep(const double step);
  void setRelativeCostTol(double value);
  void setWeightsTol(double value);
  void setVerbosityGNC(const gtsam::This::Verbosity value);
  void setKnownInliers(const gtsam::This::IndexVector& knownIn);
  void setKnownOutliers(const gtsam::This::IndexVector& knownOut);
  void print(const string& str = "GncParams: ") const;
  
  enum Verbosity {
    SILENT,
    SUMMARY,
    MU,
    WEIGHTS,
    VALUES
  };
};

typedef gtsam::GncParams<gtsam::GaussNewtonParams> GncGaussNewtonParams;
typedef gtsam::GncParams<gtsam::LevenbergMarquardtParams> GncLMParams;
  
#include <gtsam/nonlinear/NonlinearOptimizer.h>
virtual class NonlinearOptimizer {
  gtsam::Values optimize();
  gtsam::Values optimizeSafely();
  double error() const;
  int iterations() const;
  gtsam::Values values() const;
  gtsam::NonlinearFactorGraph graph() const;
  gtsam::GaussianFactorGraph* iterate() const;
};

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
virtual class GaussNewtonOptimizer : gtsam::NonlinearOptimizer {
  GaussNewtonOptimizer(const gtsam::NonlinearFactorGraph& graph,
                       const gtsam::Values& initialValues);
  GaussNewtonOptimizer(const gtsam::NonlinearFactorGraph& graph,
                       const gtsam::Values& initialValues,
                       const gtsam::GaussNewtonParams& params);
};

#include <gtsam/nonlinear/DoglegOptimizer.h>
virtual class DoglegOptimizer : gtsam::NonlinearOptimizer {
  DoglegOptimizer(const gtsam::NonlinearFactorGraph& graph,
                  const gtsam::Values& initialValues);
  DoglegOptimizer(const gtsam::NonlinearFactorGraph& graph,
                  const gtsam::Values& initialValues,
                  const gtsam::DoglegParams& params);
  double getDelta() const;
};
  
// TODO(dellaert): This will only work when GTSAM_USE_BOOST_FEATURES is true.
#include <gtsam/nonlinear/GncOptimizer.h>
template<PARAMS>
virtual class GncOptimizer {
  GncOptimizer(const gtsam::NonlinearFactorGraph& graph,
               const gtsam::Values& initialValues,
               const PARAMS& params);
  void setInlierCostThresholds(const double inth);
  const gtsam::Vector& getInlierCostThresholds();
  void setInlierCostThresholdsAtProbability(const double alpha);
  void setWeights(const gtsam::Vector w);
  const gtsam::Vector& getWeights();
  gtsam::Values optimize();
};

typedef gtsam::GncOptimizer<gtsam::GncParams<gtsam::GaussNewtonParams>> GncGaussNewtonOptimizer;
typedef gtsam::GncOptimizer<gtsam::GncParams<gtsam::LevenbergMarquardtParams>> GncLMOptimizer;

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
virtual class LevenbergMarquardtOptimizer : gtsam::NonlinearOptimizer {
  LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph,
                              const gtsam::Values& initialValues,
                              const gtsam::LevenbergMarquardtParams& params =
                                  gtsam::LevenbergMarquardtParams());
  LevenbergMarquardtOptimizer(const gtsam::NonlinearFactorGraph& graph,
                              const gtsam::Values& initialValues,
                              const gtsam::Ordering& ordering,
                              const gtsam::LevenbergMarquardtParams& params =
                                  gtsam::LevenbergMarquardtParams());

  double lambda() const;
  void print(string str = "") const;
};

#include <gtsam/nonlinear/ISAM2.h>
class ISAM2GaussNewtonParams {
  ISAM2GaussNewtonParams(double _wildfireThreshold = 0.001);

  void print(string str = "") const;

  /** Getters and Setters for all properties */
  double getWildfireThreshold() const;
  void setWildfireThreshold(double wildfireThreshold);
};

class ISAM2DoglegParams {
  ISAM2DoglegParams();

  void print(string str = "") const;

  /** Getters and Setters for all properties */
  double getWildfireThreshold() const;
  void setWildfireThreshold(double wildfireThreshold);
  double getInitialDelta() const;
  void setInitialDelta(double initialDelta);
  string getAdaptationMode() const;
  void setAdaptationMode(string adaptationMode);
  bool isVerbose() const;
  void setVerbose(bool verbose);
};

class ISAM2ThresholdMapValue {
  ISAM2ThresholdMapValue(char c, gtsam::Vector thresholds);
  ISAM2ThresholdMapValue(const gtsam::ISAM2ThresholdMapValue& other);
};

class ISAM2ThresholdMap {
  ISAM2ThresholdMap();
  ISAM2ThresholdMap(const gtsam::ISAM2ThresholdMap& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(const gtsam::ISAM2ThresholdMapValue& value) const;
};

class ISAM2Params {
  ISAM2Params();

  void print(string str = "") const;

  /** Getters and Setters for all properties */
  void setOptimizationParams(
      const gtsam::ISAM2GaussNewtonParams& gauss_newton__params);
  void setOptimizationParams(const gtsam::ISAM2DoglegParams& optimizationParams);
  void setRelinearizeThreshold(double relinearizeThreshold);
  void setRelinearizeThreshold(const gtsam::ISAM2ThresholdMap& threshold_map);
  string getFactorization() const;
  void setFactorization(string factorization);

  int relinearizeSkip;
  bool enableRelinearization;
  bool evaluateNonlinearError;
  bool cacheLinearizedFactors;
  bool enableDetailedResults;
  bool enablePartialRelinearizationCheck;
  bool findUnusedFactorSlots;

  enum Factorization { CHOLESKY, QR };
  gtsam::ISAM2Params::Factorization factorization;
};

class ISAM2Clique {
  // Constructors
  ISAM2Clique();

  // Standard Interface
  gtsam::Vector gradientContribution() const;
  void print(string s = "",
             gtsam::KeyFormatter keyFormatter = gtsam::DefaultKeyFormatter);
};

class ISAM2Result {
  ISAM2Result();

  void print(string str = "") const;

  /** Getters and Setters for all properties */
  size_t getVariablesRelinearized() const;
  size_t getVariablesReeliminated() const;
  gtsam::FactorIndices getNewFactorsIndices() const;
  size_t getCliques() const;
  double getErrorBefore() const;
  double getErrorAfter() const;
};

class ISAM2 {
  ISAM2();
  ISAM2(const gtsam::ISAM2Params& params);
  ISAM2(const gtsam::ISAM2& other);

  bool equals(const gtsam::ISAM2& other, double tol) const;
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  void printStats() const;
  void saveGraph(string s) const;

  gtsam::ISAM2Result update();
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices,
                            const gtsam::KeyGroupMap& constrainedKeys);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices,
                            const gtsam::KeyGroupMap& constrainedKeys,
                            const gtsam::KeyList& noRelinKeys);
  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::FactorIndices& removeFactorIndices,
                            gtsam::KeyGroupMap& constrainedKeys,
                            const gtsam::KeyList& noRelinKeys,
                            const gtsam::KeyList& extraReelimKeys,
                            bool force_relinearize = false);

  gtsam::ISAM2Result update(const gtsam::NonlinearFactorGraph& newFactors,
                            const gtsam::Values& newTheta,
                            const gtsam::ISAM2UpdateParams& updateParams);

  double error(const gtsam::VectorValues& x) const;

  gtsam::Values getLinearizationPoint() const;
  bool valueExists(gtsam::Key key) const;
  gtsam::Values calculateEstimate() const;
  template <VALUE = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
                     gtsam::Rot3, gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Cal3_S2, gtsam::Cal3DS2,
                     gtsam::Cal3f, gtsam::Cal3Bundler, gtsam::imuBias::ConstantBias,
                     gtsam::EssentialMatrix, gtsam::FundamentalMatrix, gtsam::SimpleFundamentalMatrix,
                     gtsam::PinholeCamera<gtsam::Cal3_S2>,
                     gtsam::PinholeCamera<gtsam::Cal3Bundler>,
                     gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
                     gtsam::PinholeCamera<gtsam::Cal3Unified>, gtsam::Vector, gtsam::Matrix}>
  VALUE calculateEstimate(gtsam::Key key) const;
  gtsam::Matrix marginalCovariance(gtsam::Key key) const;
  gtsam::Values calculateBestEstimate() const;
  gtsam::VectorValues getDelta() const;
  double error(const gtsam::VectorValues& x) const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
  gtsam::VariableIndex getVariableIndex() const;
  const gtsam::KeySet& getFixedVariables() const;
  gtsam::ISAM2Params params() const;

  void printStats() const;
  gtsam::VectorValues gradientAtZero() const;

  string dot(const gtsam::KeyFormatter& keyFormatter =
                 gtsam::DefaultKeyFormatter) const;
  void saveGraph(string s, const gtsam::KeyFormatter& keyFormatter =
                               gtsam::DefaultKeyFormatter) const;
};

#include <gtsam/nonlinear/NonlinearISAM.h>
class NonlinearISAM {
  NonlinearISAM();
  NonlinearISAM(int reorderInterval);
  void print(string s = "", const gtsam::KeyFormatter& keyFormatter =
                                gtsam::DefaultKeyFormatter) const;
  void printStats() const;
  void saveGraph(string s) const;
  gtsam::Values estimate() const;
  gtsam::Matrix marginalCovariance(gtsam::Key key) const;
  int reorderInterval() const;
  int reorderCounter() const;
  void update(const gtsam::NonlinearFactorGraph& newFactors,
              const gtsam::Values& initialValues);
  void reorder_relinearize();

  // These might be expensive as instead of a reference the wrapper will make a
  // copy
  gtsam::GaussianISAM bayesTree() const;
  gtsam::Values getLinearizationPoint() const;
  gtsam::NonlinearFactorGraph getFactorsUnsafe() const;
};

//*************************************************************************
// Nonlinear factor types
//*************************************************************************
#include <gtsam/nonlinear/PriorFactor.h>
template <T = {double,
               gtsam::Vector,
               gtsam::Point2,
               gtsam::StereoPoint2,
               gtsam::Point3,
               gtsam::Rot2,
               gtsam::SO3,
               gtsam::SO4,
               gtsam::SOn,
               gtsam::Rot3,
               gtsam::Pose2,
               gtsam::Pose3,
               gtsam::Similarity2,
               gtsam::Similarity3,
               gtsam::Unit3,
               gtsam::Cal3_S2,
               gtsam::Cal3DS2,
               gtsam::Cal3Bundler,
               gtsam::Cal3Fisheye,
               gtsam::Cal3Unified,
               gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::NavState,
               gtsam::imuBias::ConstantBias}>
virtual class PriorFactor : gtsam::NoiseModelFactor {
  PriorFactor(gtsam::Key key, const T& prior,
              const gtsam::noiseModel::Base* noiseModel);
  T prior() const;

  // enabling serialization functionality
  void serialize() const;
};

#include <gtsam/nonlinear/NonlinearEquality.h>
template <T = {gtsam::Point2, gtsam::StereoPoint2, gtsam::Point3, gtsam::Rot2,
               gtsam::SO3, gtsam::SO4, gtsam::SOn, gtsam::Rot3, gtsam::Pose2,
               gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Cal3_S2, gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::imuBias::ConstantBias}>
virtual class NonlinearEquality : gtsam::NoiseModelFactor {
  // Constructor - forces exact evaluation
  NonlinearEquality(size_t j, const T& feasible);
  // Constructor - allows inexact evaluation
  NonlinearEquality(size_t j, const T& feasible, double error_gain);

  // enabling serialization functionality
  void serialize() const;
};

template <T = {gtsam::Point2, gtsam::StereoPoint2, gtsam::Point3, gtsam::Rot2,
               gtsam::SO3, gtsam::SO4, gtsam::SOn, gtsam::Rot3, gtsam::Pose2,
               gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Cal3_S2, gtsam::CalibratedCamera,
               gtsam::PinholeCamera<gtsam::Cal3_S2>,
               gtsam::PinholeCamera<gtsam::Cal3Bundler>,
               gtsam::PinholeCamera<gtsam::Cal3Fisheye>,
               gtsam::PinholeCamera<gtsam::Cal3Unified>,
               gtsam::imuBias::ConstantBias}>
virtual class NonlinearEquality2 : gtsam::NoiseModelFactor {
  NonlinearEquality2(gtsam::Key key1, gtsam::Key key2, double mu = 1e4);
  gtsam::Vector evaluateError(const T& x1, const T& x2);
};

#include <gtsam/nonlinear/FixedLagSmoother.h>
// This class is not available in python, just use a dictionary
class FixedLagSmootherKeyTimestampMapValue {
  FixedLagSmootherKeyTimestampMapValue(gtsam::Key key, double timestamp);
  FixedLagSmootherKeyTimestampMapValue(const gtsam::FixedLagSmootherKeyTimestampMapValue& other);
};

// This class is not available in python, just use a dictionary
class FixedLagSmootherKeyTimestampMap {
  FixedLagSmootherKeyTimestampMap();
  FixedLagSmootherKeyTimestampMap(const gtsam::FixedLagSmootherKeyTimestampMap& other);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  double at(const gtsam::Key key) const;
  void insert(const gtsam::FixedLagSmootherKeyTimestampMapValue& value);
};

class FixedLagSmootherResult {
  size_t getIterations() const;
  size_t getNonlinearVariables() const;
  size_t getLinearVariables() const;
  double getError() const;
};

virtual class FixedLagSmoother {
  void print(string s) const;
  bool equals(const gtsam::FixedLagSmoother& rhs, double tol) const;

  gtsam::FixedLagSmootherKeyTimestampMap timestamps() const;
  double smootherLag() const;

  gtsam::FixedLagSmootherResult update(const gtsam::NonlinearFactorGraph &newFactors,
                                       const gtsam::Values &newTheta,
                                       const gtsam::FixedLagSmootherKeyTimestampMap &timestamps);
  gtsam::FixedLagSmootherResult update(const gtsam::NonlinearFactorGraph &newFactors,
                                       const gtsam::Values &newTheta,
                                       const gtsam::FixedLagSmootherKeyTimestampMap &timestamps,
                                       const gtsam::FactorIndices &factorsToRemove);
  gtsam::Values calculateEstimate() const;
};

#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
virtual class BatchFixedLagSmoother : gtsam::FixedLagSmoother {
  BatchFixedLagSmoother();
  BatchFixedLagSmoother(double smootherLag);
  BatchFixedLagSmoother(double smootherLag, const gtsam::LevenbergMarquardtParams& parameters);

  void print(string s = "BatchFixedLagSmoother:\n") const;

  gtsam::LevenbergMarquardtParams params() const;

  gtsam::NonlinearFactorGraph getFactors() const;

  template <VALUE = {gtsam::Point2, gtsam::Rot2, gtsam::Pose2, gtsam::Point3,
                     gtsam::Rot3, gtsam::Pose3, gtsam::Similarity2, gtsam::Similarity3, gtsam::Cal3_S2, gtsam::Cal3DS2,
                     gtsam::Vector, gtsam::Matrix}>
  VALUE calculateEstimate(gtsam::Key key) const;
};

#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
virtual class IncrementalFixedLagSmoother : gtsam::FixedLagSmoother {
  IncrementalFixedLagSmoother();
  IncrementalFixedLagSmoother(double smootherLag);
  IncrementalFixedLagSmoother(double smootherLag, const gtsam::ISAM2Params& parameters);

  void print(string s = "IncrementalFixedLagSmoother:\n") const;

  gtsam::Matrix marginalCovariance(gtsam::Key key) const;
  gtsam::ISAM2Params params() const;

  gtsam::NonlinearFactorGraph getFactors() const;
  gtsam::ISAM2 getISAM2() const;
};

#include <gtsam/nonlinear/ExtendedKalmanFilter.h>
template <T = {gtsam::Point2,
               gtsam::Point3,
               gtsam::Rot2,
               gtsam::Rot3,
               gtsam::Pose2,
               gtsam::Pose3,
               gtsam::Similarity2,
               gtsam::Similarity3,
               gtsam::NavState,
               gtsam::imuBias::ConstantBias}>
virtual class ExtendedKalmanFilter {
  ExtendedKalmanFilter(gtsam::Key key_initial, const T& x_initial, const gtsam::noiseModel::Gaussian* P_initial);
  
  T predict(const gtsam::NoiseModelFactor& motionFactor);
  T update(const gtsam::NoiseModelFactor& measurementFactor);
  
  gtsam::JacobianFactor::shared_ptr Density() const;
};

}  // namespace gtsam
