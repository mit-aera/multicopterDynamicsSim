#ifndef INERTIALMEASUREMENTSIM_H
#define INERTIALMEASUREMENTSIM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

class inertialMeasurementSim
{
    public:
        inertialMeasurementSim(double accMeasNoiseVariance, double gyroMeasNoiseVariance,
                               double accBiasProcessNoiseAutoCorrelation, double gyroBiasProcessNoiseAutoCorrelation);

        void setBias(const Eigen::Vector3d & accBias, const Eigen::Vector3d & gyroBias,
                                     double accBiasProcessNoiseAutoCorrelation,
                                     double gyroBiasProcessNoiseAutoCorrelation);

        void setNoiseVariance(double accMeasNoiseVariance, double gyroMeasNoiseVariance);

        void setOrientation(const Eigen::Quaterniond & imuOrient);

        void getMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput,
                      const Eigen::Vector3d specificForce, const Eigen::Vector3d angularVelocity);

        void proceedBiasDynamics(double dt_secs);
    private:
        // Std normal rng
        std::default_random_engine randomNumberGenerator_;
        std::normal_distribution<double> standardNormalDistribution_ = std::normal_distribution<double>(0.0,1.0);

        // Measurement noise variance
        double accMeasNoiseVariance_ = 0.; // m^2/s^4
        double gyroMeasNoiseVariance_ = 0.; // rad^2/s^2

        // Bias dynamics process noise auto correlation
        double accBiasProcessNoiseAutoCorrelation_ = 0.;
        double gyroBiasProcessNoiseAutoCorrelation_ = 0.;

        // Bias states
        Eigen::Vector3d accBias_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d gyroBias_ = Eigen::Vector3d::Zero();

        // Orientation of IMU in body-fixed frame
        Eigen::Quaterniond imuOrient_ = Eigen::Quaterniond::Identity();
};

#endif // INERTIALMEASUREMENTSIM_H