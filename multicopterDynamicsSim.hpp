#ifndef MULTICOPTERDYNAMICSSIM_H
#define MULTICOPTERDYNAMICSSIM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>



class MulticopterDynamics{
    public:
        MulticopterDynamics(int numCopter, double thrustCoefficient, double torqueCoefficient,
                            double minMotorSpeed, double maxMotorSpeed,
                            double motorTimeConstant, double vehicleMass,
                            const Eigen::Matrix3d & vehicleInertia, 
                            const Eigen::Matrix3d & aeroMomentCoefficient,
                            double dragCoefficient,
                            double angAccelProcessNoiseAutoCorrelation,
                            double linAccelProcessNoiseAutoCorrelation,
                            const Eigen::Vector3d & gravity);
        MulticopterDynamics(int numCopter);
        void setVehicleProperties(double vehicleMass, const Eigen::Matrix3d & vehicleInertia, 
                                            const Eigen::Matrix3d & aeroMomentCoefficient,
                                            double dragCoefficient,
                                            double angAccelProcessNoiseAutoCorrelation,
                                            double linAccelProcessNoiseAutoCorrelation);
        void setGravityVector(const Eigen::Vector3d & gravity);
        void setMotorFrame(const Eigen::Isometry3d & motorFrame, int motorDirection, int motorIndex);
        void setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                             double minMotorSpeed, double maxMotorSpeed, int motorIndex);
        void setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                             double minMotorSpeed, double maxMotorSpeed);
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
        Eigen::Vector3d getVehicleSpecificForce(void);
        void proceedState_ExplicitEuler(double dt_secs, const std::vector<double> & motorSpeedCommand);
        void proceedState_RK4(double dt_secs, const std::vector<double> & motorSpeedCommand);

    private:
        // Number of rotors
        int numCopter_;

        // Transform from motor to vehicle (c.o.g.) frame
        // Motor frame must have prop spinning around z-axis with positive thrust in positive z axis direction
        std::vector<Eigen::Isometry3d> motorFrame_;

        // -1 if positive rotation rate corresponds to negative moment around motor z axis
        std::vector<int> motorDirection_;

        std::vector<double> thrustCoefficient_;
        std::vector<double> torqueCoefficient_;
        std::vector<double> motorTimeConstant_;
        std::vector<double> maxMotorSpeed_;
        std::vector<double> minMotorSpeed_;

        double dragCoefficient_;
        Eigen::Matrix3d aeroMomentCoefficient_;
        double vehicleMass_;
        Eigen::Matrix3d vehicleInertia_;
        double angAccelProcessNoiseAutoCorrelation_ = 0.; // N^2s
        double linAccelProcessNoiseAutoCorrelation_ = 0.; // (Nm)^2s

        std::default_random_engine randomNumberGenerator_;
        std::normal_distribution<double> standardNormalDistribution_ = std::normal_distribution<double>(0.0,1.0);

        // Default is NED, but can be set by changing gravity direction
        Eigen::Vector3d gravity_;

        // Vehicle state variables
        std::vector<double> motorSpeed_;
        Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d angularVelocity_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond attitude_ = Eigen::Quaterniond::Identity();


        Eigen::Vector3d getThrust(const std::vector<double> & motorSpeed);
        Eigen::Vector3d getControlMoment(const std::vector<double> & motorSpeed);
        Eigen::Vector3d getAeroMoment(const Eigen::Vector3d & angularVelocity);
        Eigen::Vector3d getDragForce(const Eigen::Vector3d & velocity);
        void getMotorSpeedDerivative(std::vector<double> & motorSpeedDer,
                                                  const std::vector<double> & motorSpeed,
                                                  const std::vector<double> & motorSpeedCommand);
        Eigen::Vector3d getVelocityDerivative(const Eigen::Quaterniond & attitude, const Eigen::Vector3d & stochForce,
                                        const Eigen::Vector3d & velocity, const std::vector<double> & motorSpeed);
        Eigen::Vector3d getAngularVelocityDerivative(const std::vector<double> & motorSpeed,
                                const Eigen::Vector3d & angularVelocity, const Eigen::Vector3d & stochMoment);
        Eigen::Vector4d getAttitudeDerivative(const Eigen::Quaterniond & attitude, const Eigen::Vector3d & angularVelocity);
        void vectorAffineOp(const std::vector<double> & vec1, const std::vector<double> & vec2, 
                                  std::vector<double> & vec3, double val);
        void vectorScalarProd(const std::vector<double> & vec1, std::vector<double> & vec2, double val);
        void vectorBoundOp(const std::vector<double> & vec1, std::vector<double> & vec2,
                                         const std::vector<double> &  minvec, const std::vector<double> & maxvec);
};

#endif // MULTICOPTERDYNAMICSSIM_H