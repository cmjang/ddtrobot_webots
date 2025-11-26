#include "estimator/OrientationEstimator.h"

/*!
 * Get quaternion, rotation matrix, angular velocity (body and world),
 * rpy, acceleration (world, body) from vector nav IMU
 */
void VectorNavOrientationEstimator::run()
{
  this->_stateEstimatorData.result->aBody = this->_stateEstimatorData.lowState->accelerometer;
  this->_stateEstimatorData.result->omegaBody = this->_stateEstimatorData.lowState->gyro;
  this->_stateEstimatorData.result->orientation = this->_stateEstimatorData.lowState->quat;

  this->_stateEstimatorData.result->rpy =
    ori::quatToRPY(this->_stateEstimatorData.result->orientation);
  this->_stateEstimatorData.result->rBody =
    ori::quaternionToRotationMatrix(this->_stateEstimatorData.result->orientation);
  this->_stateEstimatorData.result->omegaWorld =
    this->_stateEstimatorData.result->rBody.transpose() *
    this->_stateEstimatorData.result->omegaBody;
  this->_stateEstimatorData.result->aWorld =
    this->_stateEstimatorData.result->rBody.transpose() * this->_stateEstimatorData.result->aBody;

  this->_stateEstimatorData.result->rCtrl =
    ori::coordinateRotation(ori::CoordinateAxis::Z, this->_stateEstimatorData.result->rpy(2));
  // this->_stateEstimatorData.result->omegaCtrl =
  //   this->_stateEstimatorData.result->rCtrl.transpose() *
  //   this->_stateEstimatorData.result->omegaBody;
  // this->_stateEstimatorData.result->aCtrl =
  //   this->_stateEstimatorData.result->rCtrl.transpose() * this->_stateEstimatorData.result->aBody;
}