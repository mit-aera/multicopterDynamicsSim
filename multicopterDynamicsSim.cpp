#include "multicopterDynamicsSim.hpp"


MulticopterDynamics::MulticopterDynamics(int numCopter,
double thrustCoefficient, double torqueCoefficient,
double motorTimeConstant, )
: numCopter_(numCopter)
, motorFrame_(numCopter)
, motorDirection_(numCopter)
, thrustCoefficient_(numCopter)
, torqueCoefficient_(numCopter)
{
    for (int indx = 0; indx < numCopter; indx++){
        motorFrame_.at(indx).setIdentity();
        thrustCoefficient_.at(indx) = thrustCoefficient;
        torqueCoefficient_.at(indx) = torqueCoefficient;
        motorTimeConstant_.at(indx) = motorTimeConstant;
        motorSpeed_.at(indx) = 0.;
    }

    position_.setZero();
    velocity_.setZero();
    angularVelocity_.setZero();
    attitude_.setIdentity();
}

void MulticopterDynamics::setmotorFrame(const Eigen::Isometry3d & motorFrame, int motorIndex){
    // TODO: Is this the correct manner to copy a value by reference into a actual value?
    motorFrame_.at(motorIndx) = motorFrame;
}

void MulticopterDynamics::setThrustCoefficient(double thrustCoefficient, int motorIndex){
    thrustCoefficient_.at(motorIndx) = thrustCoefficient;
}

void MulticopterDynamics::setTorqueCoefficient(double torqueCoefficient, int motorIndex){
    torqueCoefficient_.at(motorIndx) = torqueCoefficient;
}

void MulticopterDynamics::setMotorTimeConstant(double motorTimeConstant, int motorIndex){
    motorTimeConstant_.at(motorIndx) = motorTimeConstant;
}

void MulticopterDynamics::setMotorSpeed(double motorSpeed, int motorIndex){
    motorSpeed_.at(motorIndex) = motorSpeed;
}

void MulticopterDynamics::resetMotorSpeeds(void){
    for (int indx = 0; indx < numCopter_; indx++){
        motorSpeed_.at(indx) = 0.;
    }
}

void MulticopterDynamics::setVehicleState(const Eigen::Vector3d & position,
                                              const Eigen::Vector3d & velocity,
                                              const Eigen::Vector3d & angularVelocity,
                                              const Eigen::Quaterniond & attitude){
    position_ = position;
    velocity_ = velocity;
    angularVelocity_ = angularVelocity;
    attitude_ = attitude;
}

void MulticopterDynamics::getControlMomentThrust(Eigen::Vector3d & controlMoment, Eigen::Vector3d & thrust){
    controlMoment.setZero();
    thrust.setZero();

    for (int indx = 0; indx < numCopter_; indx++){
        if(motorSpeed_.at(indx) >= 0.){

            Eigen::Vector3d motorThrust(0.,0.,pow(motorSpeed_.at(indx),2.)*thrustCoefficient_.at(indx));
            thrust += motorFrame_.at(indx).linear()*motorThrust;

            // Moment due to thrust
            controlMoment += motorFrame_.at(indx).translation().cross(motorFrame_.at(indx).linear()*motorThrust);

            // Moment due to torque
            Eigen::Vector3d motorTorque(0.,0.,motorDirection_.at(indx)*pow(motorSpeed_.at(indx),2.)*torqueCoefficient_.at(indx));
            controlMoment += motorTorque;
        }
    }
}

void MulticopterDynamics::getAeroDragMoment(Eigen::Vector3d & dragForce, Eigen::Vector3d & aeroMoment){
    dragforce = -dragCoefficient_*velocity_.norm()*velocity_;
    aeroMoment = -angularVelocity_.norm()*aeroMomentCoefficient_*angularVelocity_;
}

void MulticopterDynamics::proceedState(double dt_secs, const std::vector<double> & motorSpeedCommand){


    attitude_.normalize();
}

void MulticopterDynamics::getStateDerivative(){
    
}
