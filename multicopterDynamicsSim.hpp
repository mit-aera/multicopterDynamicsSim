/**
 * @file multicopterDynamicsSim.hpp
 * @author Ezra Tal
 * @brief Multicopter dynamics simulator class header file
 * 
 */

#ifndef MULTICOPTERDYNAMICSSIM_H
#define MULTICOPTERDYNAMICSSIM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include "inertialMeasurementSim.hpp"

#include <iomanip>

/**
 * @brief Multicopter dynamics simulator class
 * 
 */
class MulticopterDynamicsSim{
    public:
        MulticopterDynamicsSim(int numCopter, double thrustCoefficient, double torqueCoefficient,
                               double minMotorSpeed, double maxMotorSpeed,
                               double motorTimeConstant, double motorRotationalInertia,
                               double vehicleMass,
                               const Eigen::Matrix3d & vehicleInertia, 
                               const Eigen::Matrix3d & aeroMomentCoefficient,
                               double dragCoefficient,
                               double momentProcessNoiseAutoCorrelation,
                               double forceProcessNoiseAutoCorrelation,
                               const Eigen::Vector3d & gravity);
        MulticopterDynamicsSim(int numCopter);

        int getNumCopter() { return numCopter_; }
        void setVehicleProperties(double vehicleMass, const Eigen::Matrix3d & vehicleInertia, 
                                  const Eigen::Matrix3d & aeroMomentCoefficient,
                                  double dragCoefficient,
                                  double momentProcessNoiseAutoCorrelation,
                                  double forceProcessNoiseAutoCorrelation);
        void setGravityVector(const Eigen::Vector3d & gravity);
        void setMotorFrame(const Eigen::Isometry3d & motorFrame, int motorDirection, int motorIndex);
        void setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                double minMotorSpeed, double maxMotorSpeed, double rotationalInertia, int motorIndex);
        void setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                double minMotorSpeed, double maxMotorSpeed, double rotationalInertia);
        void setMotorSpeed(double motorSpeed, int motorIndex);
        void setMotorSpeed(double motorSpeed);
        void resetMotorSpeeds(void);
        void setVehiclePosition(const Eigen::Vector3d & position,const Eigen::Quaterniond & attitude);
        void setVehicleState(const Eigen::Vector3d & position,
                             const Eigen::Vector3d & velocity,
                             const Eigen::Vector3d & angularVelocity,
                             const Eigen::Quaterniond & attitude,
                             const std::vector<double> & motorSpeed);
        void getVehicleState(Eigen::Vector3d & position,
                             Eigen::Vector3d & velocity,
                             Eigen::Vector3d & angularVelocity,
                             Eigen::Quaterniond & attitude,
                             std::vector<double> & motorSpeed);
        Eigen::Vector3d getVehiclePosition(void);
        Eigen::Quaterniond getVehicleAttitude(void);
        Eigen::Vector3d getVehicleVelocity(void);
        Eigen::Vector3d getVehicleAngularVelocity(void);
        std::vector<double> getMotorSpeed();

        Eigen::Vector3d getVehicleSpecificForce(void);

        void proceedState_ExplicitEuler(double dt_secs, const std::vector<double> & motorSpeedCommand);
        void proceedState_RK4(double dt_secs, const std::vector<double> & motorSpeedCommand);

        void getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput);

        /// @name IMU simulator
        inertialMeasurementSim imu_ = inertialMeasurementSim(0.,0.,0.,0.);
        
    private:
        /// @name  Number of rotors
        int numCopter_;

        /// @name Flag indicating whether to add stochastic forces
        //TODO add ROS parameter querying to set this.
        bool enableStochasticity_;

        /// @name Motor properties
        //@{

        // Transform from motor to vehicle (c.o.g.) frame
        /* Motor frame must have prop spinning around z-axis such that
         a positive motor speed corresponds to a positive thrust in 
         positive motor z-axis direction.*/
        std::vector<Eigen::Isometry3d> motorFrame_;

        /* +1 if positive motor speed corresponds to positive moment around the motor frame z-axis
           -1 if positive motor speed corresponds to negative moment around the motor frame z-axis
           i.e. -1 indicates a positive motor speed corresponds to a positive rotation rate around the motor z-axis

           Values: 
           motor0 -1 (front left)
           motor1 1 (back left)
           motor2 -1 (back right)
           motor3 1 (front right)
        */
        std::vector<int> motorDirection_;

        std::vector<double> thrustCoefficient_; // N/(rad/s)^2
        std::vector<double> torqueCoefficient_; // Nm/(rad/s)^2
        std::vector<double> motorTimeConstant_; // s
        std::vector<double> motorRotationalInertia_; // kg m^2
        std::vector<double> maxMotorSpeed_; // rad/s
        std::vector<double> minMotorSpeed_; // rad/s
        //@}

        /// @name Vehicle properties
        //@{
        double dragCoefficient_; // N/(m/s)
        Eigen::Matrix3d aeroMomentCoefficient_; // Nm/(rad/s)^2
        double vehicleMass_; // kg
        Eigen::Matrix3d vehicleInertia_; // kg m^2
        double momentProcessNoiseAutoCorrelation_ = 0.; // (Nm)^2s
        double forceProcessNoiseAutoCorrelation_ = 0.; // N^2s
        //@}

        /// @name Std normal RNG
        //@{
        std::default_random_engine randomNumberGenerator_;
        std::normal_distribution<double> standardNormalDistribution_ = std::normal_distribution<double>(0.0,1.0);
        //@}

        // Default reference frame is NED, but can be set by changing gravity direction
        /// @name Gravity vector
        Eigen::Vector3d gravity_; // m/s^2

        /// @name Vehicle state variables
        //@{
        std::vector<double> motorSpeed_; // rad/s
        Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero(); // m/s
        Eigen::Vector3d position_ = Eigen::Vector3d::Zero(); // m
        Eigen::Vector3d angularVelocity_ = Eigen::Vector3d::Zero(); // rad/s
        Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();
        //@}

        /* Vehicle stochastic force vector (in world frame) is maintained
        for accelerometer output, since it must include the same
        random linear acceleration noise as used for dynamics integration*/
        /// @name Vehicle stochastic force vector
        Eigen::Vector3d stochForce_ = Eigen::Vector3d::Zero(); // N

        void vectorAffineOp(const std::vector<double> & vec1, const std::vector<double> & vec2, 
                            std::vector<double> & vec3, double val);
        void vectorScalarProd(const std::vector<double> & vec1, std::vector<double> & vec2, double val);
        void vectorBoundOp(const std::vector<double> & vec1, std::vector<double> & vec2,
                           const std::vector<double> &  minvec, const std::vector<double> & maxvec);
};

#endif // MULTICOPTERDYNAMICSSIM_H