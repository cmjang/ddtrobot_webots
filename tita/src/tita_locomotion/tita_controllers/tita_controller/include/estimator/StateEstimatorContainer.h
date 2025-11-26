/*!
 * @file StateEstimator.h
 * @brief Implementation of State Estimator Interface
 *
 * Each StateEstimator object contains a number of estimators
 *
 * When the state estimator is run, it runs all estimators.
 */

#ifndef PROJECT_STATEESTIMATOR_H
#define PROJECT_STATEESTIMATOR_H
#include "common/Math/orientation_tools.h"
#include "common/RobotParameters.h"
#include "common/enumClass.h"

/*!
 * Inputs for state estimation.
 * If robot code needs to inform the state estimator of something,
 * it should be added here. (You should also a setter method to
 * StateEstimatorContainer)
 */
// template <typename T>
struct StateEstimatorData
{
  std::shared_ptr<StateEstimate> result;  // where to write the output to
  std::shared_ptr<LowlevelState> lowState;
  std::shared_ptr<WheelLeggedData> legWheelsData;
  std::shared_ptr<RobotControlParameters> parameters;
};

/*!
 * All Estimators should inherit from this class
 */
class GenericEstimator
{
public:
  virtual void run() = 0;
  virtual void setup() = 0;

  void setData(StateEstimatorData data) { _stateEstimatorData = data; }

  virtual ~GenericEstimator() = default;
  StateEstimatorData _stateEstimatorData;
};

/*!
 * Main State Estimator Class
 * Contains all GenericEstimators, and can run them
 * Also updates visualizations
 */
// template <typename T>
class StateEstimatorContainer
{
public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /*!
   * Construct a new state estimator container
   */
  StateEstimatorContainer(
    std::shared_ptr<RobotControlParameters> parameters, std::shared_ptr<LowlevelState> lowState,
    std::shared_ptr<WheelLeggedData> legWheelsData, std::shared_ptr<StateEstimate> stateEstimate)
  {
    _data.result = stateEstimate;
    _data.lowState = lowState;
    _data.legWheelsData = legWheelsData;
    _data.parameters = parameters;
  }

  /*!
   * Run all estimators
   */
  void run(/*CheetahVisualization* visualization = nullptr*/)
  {
    for (auto estimator : _estimators) {
      estimator->run();
    }
    // if (visualization) {
    //   visualization->quat = _data.result->orientation.template cast<float>();
    //   visualization->p = _data.result->position.template cast<float>();
    //   // todo contact!
    // }
  }

  /*!
   * Get the result
   */
  const StateEstimate & getResult() { return *_data.result; }
  // StateEstimate * getResultHandle() { return _data.result; }

  /*!
   * Set the contact phase
   */
  // void setContactPhase(Vec4<scalar_t>& phase) {
  //   *_data.contactPhase = phase;
  // }

  /*!
   * Add an estimator of the given type
   * @tparam EstimatorToAdd
   */
  template <typename EstimatorToAdd>
  void addEstimator()
  {
    auto * estimator = new EstimatorToAdd();
    estimator->setData(_data);
    estimator->setup();
    _estimators.push_back(estimator);
  }

  /*!
   * Remove all estimators of a given type
   * @tparam EstimatorToRemove
   */
  template <typename EstimatorToRemove>
  void removeEstimator()
  {
    int nRemoved = 0;
    _estimators.erase(
      std::remove_if(
        _estimators.begin(), _estimators.end(),
        [&nRemoved](GenericEstimator * e) {
          if (dynamic_cast<EstimatorToRemove *>(e)) {
            delete e;
            nRemoved++;
            return true;
          } else {
            return false;
          }
        }),
      _estimators.end());
  }

  /*!
   * Remove all estimators
   */
  void removeAllEstimators()
  {
    for (auto estimator : _estimators) {
      delete estimator;
    }
    _estimators.clear();
  }

  ~StateEstimatorContainer()
  {
    for (auto estimator : _estimators) {
      delete estimator;
    }
  }

private:
  StateEstimatorData _data;
  std::vector<GenericEstimator *> _estimators;
  Vec4<scalar_t> _phase;
};

#endif  // PROJECT_STATEESTIMATOR_H
