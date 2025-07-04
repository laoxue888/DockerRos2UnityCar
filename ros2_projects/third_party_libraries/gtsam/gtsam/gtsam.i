/**

 * GTSAM Wrap Module Definition
 *
 * These are the current classes available through the matlab and python
 wrappers,
 * add more functions/classes as they are available.
 *
 * Please refer to the wrapping docs:
 https://github.com/borglab/wrap/blob/master/README.md
 */

namespace gtsam {

#include <gtsam/inference/Key.h>

const KeyFormatter DefaultKeyFormatter;

// Actually a FastList<Key>
class KeyList {
  KeyList();
  KeyList(const gtsam::KeyList& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t front() const;
  size_t back() const;
  void push_back(gtsam::Key key);
  void push_front(gtsam::Key key);
  void pop_back();
  void pop_front();
  void sort();
  void remove(gtsam::Key key);

  void serialize() const;

  // Special dunder methods for Python wrapping
  __len__();
  __contains__(gtsam::Key key);
  __iter__();
};

// Actually a FastSet<Key>
class KeySet {
  KeySet();
  KeySet(const gtsam::KeySet& set);
  KeySet(const gtsam::KeyVector& vector);
  KeySet(const gtsam::KeyList& list);

  // Testable
  void print(string s = "") const;
  bool equals(const gtsam::KeySet& other) const;

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(gtsam::Key key);
  void merge(const gtsam::KeySet& other);
  bool erase(gtsam::Key key);        // returns true if value was removed
  bool count(gtsam::Key key) const;  // returns true if value exists

  void serialize() const;

  // Special dunder methods for Python wrapping
  __len__();
  __contains__(gtsam::Key key);
  __iter__();
};

// Actually a vector<Key>, needed for Matlab
class KeyVector {
  KeyVector();
  KeyVector(const gtsam::KeyVector& other);

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t at(size_t i) const;
  size_t front() const;
  size_t back() const;
  void push_back(gtsam::Key key) const;

  void serialize() const;

  // Special dunder methods for Python wrapping
  __len__();
  __contains__(gtsam::Key key);
  __iter__();
};

// Actually a FastMap<Key,int>
class KeyGroupMap {
  KeyGroupMap();

  // Note: no print function

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t at(gtsam::Key key) const;
  int erase(gtsam::Key key);
  bool insert2(gtsam::Key key, int val);
};

// Actually a FastSet<FactorIndex>
// Used in Matlab wrapper
class FactorIndexSet {
  FactorIndexSet();
  FactorIndexSet(const gtsam::FactorIndexSet& set);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  void insert(size_t factorIndex);
  bool erase(size_t factorIndex);        // returns true if value was removed
  bool count(size_t factorIndex) const;  // returns true if value exists
};

// Actually a vector<FactorIndex>
// Used in Matlab wrapper
class FactorIndices {
  FactorIndices();
  FactorIndices(const gtsam::FactorIndices& other);

  // common STL methods
  size_t size() const;
  bool empty() const;
  void clear();

  // structure specific methods
  size_t at(size_t i) const;
  size_t front() const;
  size_t back() const;
  void push_back(size_t factorIndex) const;
};

//*************************************************************************
// Utilities
//*************************************************************************

namespace utilities {

#include <gtsam/nonlinear/utilities.h>
gtsam::KeyList createKeyList(gtsam::Vector I);
gtsam::KeyList createKeyList(string s, gtsam::Vector I);
gtsam::KeyVector createKeyVector(gtsam::Vector I);
gtsam::KeyVector createKeyVector(string s, gtsam::Vector I);
gtsam::KeySet createKeySet(gtsam::Vector I);
gtsam::KeySet createKeySet(string s, gtsam::Vector I);
gtsam::Matrix extractPoint2(const gtsam::Values& values);
gtsam::Matrix extractPoint3(const gtsam::Values& values);
gtsam::Values allPose2s(gtsam::Values& values);
gtsam::Matrix extractPose2(const gtsam::Values& values);
gtsam::Values allPose3s(gtsam::Values& values);
gtsam::Matrix extractPose3(const gtsam::Values& values);
gtsam::Matrix extractVectors(const gtsam::Values& values, char c);
void perturbPoint2(gtsam::Values& values, double sigma, int seed = 42u);
void perturbPose2(gtsam::Values& values, double sigmaT, double sigmaR,
                  int seed = 42u);
void perturbPoint3(gtsam::Values& values, double sigma, int seed = 42u);
void insertBackprojections(gtsam::Values& values,
                           const gtsam::PinholeCamera<gtsam::Cal3_S2>& c,
                           gtsam::Vector J, gtsam::Matrix Z, double depth);
void insertProjectionFactors(
    gtsam::NonlinearFactorGraph& graph, size_t i, gtsam::Vector J, gtsam::Matrix Z,
    const gtsam::noiseModel::Base* model, const gtsam::Cal3_S2* K,
    const gtsam::Pose3& body_P_sensor = gtsam::Pose3());
gtsam::Matrix reprojectionErrors(const gtsam::NonlinearFactorGraph& graph,
                          const gtsam::Values& values);
gtsam::Values localToWorld(const gtsam::Values& local,
                           const gtsam::Pose2& base);
gtsam::Values localToWorld(const gtsam::Values& local, const gtsam::Pose2& base,
                           const gtsam::KeyVector& keys);

}  // namespace utilities

class RedirectCout {
  RedirectCout();
  string str();
};

}  // namespace gtsam
