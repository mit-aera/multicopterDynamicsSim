#ifndef MULTICOPTERDYNAMICSSIM_H
#define MULTICOPTERDYNAMICSSIM_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>



class MulticopterDynamics{
    public:
    MulticopterDynamics(int numCopter);

    void setMotorFrame(Eigen::Transform3 & motorFrame, int motorIndex);
    void setThrustCoefficient(double thrustCoefficient, int motorIndex);
    void setTorqueCoefficient(double torqueCoefficient, int motorIndex);

    private:
        // Number of rotors
        int numCopter_;

        // Transform from motor to vehicle (c.o.g.) frame
        // Vehicle frame is x forward, y left, z up
        // Motor frame must have prop spinning around z-axis with positive thrust in z axis direction
        std::vector<Eigen::Isometry3d> motorFrame_;

        // -1 if positive rotation rate corresponds to positive rotation around z axis
        std::vector<int> motorDirection_;

        std::vector<double> thrustCoefficient_;
        std::vector<double> torqueCoefficient_;
        std::vector<double> motorTimeConstant_;

        double dragCoefficient_;
        Eigen::Matrix3d aeroMomentCoefficient_;


        // Vehicle state variables
        std::vector<double> motorSpeed_;
        Eigen::Vector3d position_;
        Eigen::Vector3d velocity_;
        Eigen::Vector3d angularVelocity_;
        Eigen::Quaterniond attitude_;
}

#endif // MULTICOPTERDYNAMICSSIM_H