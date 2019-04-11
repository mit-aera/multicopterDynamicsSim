#include "inertialMeasurementSim.hpp"
#include <chrono>

inertialMeasurementSim::inertialMeasurementSim(double accMeasNoiseVariance, double gyroMeasNoiseVariance,
                        double accBiasProcessNoiseAutoCorrelation, double gyroBiasProcessNoiseAutoCorrelation){

    randomNumberGenerator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
                            
    accMeasNoiseVariance_ = accMeasNoiseVariance;
    gyroMeasNoiseVariance_ = gyroMeasNoiseVariance;
    accBiasProcessNoiseAutoCorrelation_ = accBiasProcessNoiseAutoCorrelation;
    gyroBiasProcessNoiseAutoCorrelation_ = gyroBiasProcessNoiseAutoCorrelation;
}

void inertialMeasurementSim::setBias(const Eigen::Vector3d & accBias, const Eigen::Vector3d & gyroBias,
                                     double accBiasProcessNoiseAutoCorrelation,
                                     double gyroBiasProcessNoiseAutoCorrelation){
    accBias_ = accBias;
    gyroBias_ = gyroBias;
    accBiasProcessNoiseAutoCorrelation_ = accBiasProcessNoiseAutoCorrelation;
    gyroBiasProcessNoiseAutoCorrelation_ = gyroBiasProcessNoiseAutoCorrelation;
}

void inertialMeasurementSim::setBias(double accBiasVariance, double gyroBiasVariance,
                                     double accBiasProcessNoiseAutoCorrelation,
                                     double gyroBiasProcessNoiseAutoCorrelation){

    accBias_ << sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accBiasVariance)*standardNormalDistribution_(randomNumberGenerator_);

    gyroBias_ << sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroBiasVariance)*standardNormalDistribution_(randomNumberGenerator_);

    accBiasProcessNoiseAutoCorrelation_ = accBiasProcessNoiseAutoCorrelation;
    gyroBiasProcessNoiseAutoCorrelation_ = gyroBiasProcessNoiseAutoCorrelation;
}

void inertialMeasurementSim::setNoiseVariance(double accMeasNoiseVariance, double gyroMeasNoiseVariance){
    accMeasNoiseVariance_ = accMeasNoiseVariance;
    gyroMeasNoiseVariance_ = gyroMeasNoiseVariance;
}

void inertialMeasurementSim::setOrientation(const Eigen::Quaterniond & imuOrient){
    imuOrient_ = imuOrient;
}

void inertialMeasurementSim::getMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput,
                const Eigen::Vector3d specificForce, const Eigen::Vector3d angularVelocity){
    // specificForce and angularVelocity in body-frame
    // accOutput and gyroOutput in IMU frame

    Eigen::Vector3d accNoise;

    accNoise << sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                sqrt(accMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

    Eigen::Vector3d gyroNoise;

    gyroNoise << sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_),
                 sqrt(gyroMeasNoiseVariance_)*standardNormalDistribution_(randomNumberGenerator_);

    accOutput = imuOrient_.inverse()*specificForce + accBias_ + accNoise;
    gyroOutput = imuOrient_.inverse()*angularVelocity + gyroBias_ + gyroNoise;
}

void inertialMeasurementSim::proceedBiasDynamics(double dt_secs){
    Eigen::Vector3d accBiasDerivative;

    accBiasDerivative << sqrt(accBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                         sqrt(accBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                         sqrt(accBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    Eigen::Vector3d gyroBiasDerivative;

    gyroBiasDerivative << sqrt(gyroBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                          sqrt(gyroBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                          sqrt(gyroBiasProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    accBias_ += dt_secs*accBiasDerivative;
    gyroBias_ += dt_secs*gyroBiasDerivative;
}