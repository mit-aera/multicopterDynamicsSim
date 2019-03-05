#include "multicopterDynamicsSim.hpp"
#include <iostream>

MulticopterDynamicsSim::MulticopterDynamicsSim(int numCopter,
double thrustCoefficient, double torqueCoefficient,
double minMotorSpeed, double maxMotorSpeed,
double motorTimeConstant, double vehicleMass,
const Eigen::Matrix3d & vehicleInertia, 
const Eigen::Matrix3d & aeroMomentCoefficient,
double dragCoefficient,
double momentProcessNoiseAutoCorrelation,
double forceProcessNoiseAutoCorrelation,
const Eigen::Vector3d & gravity
)
: numCopter_(numCopter)
, motorFrame_(numCopter)
, motorDirection_(numCopter)
, thrustCoefficient_(numCopter)
, torqueCoefficient_(numCopter)
, motorSpeed_(numCopter)
, motorTimeConstant_(numCopter)
, maxMotorSpeed_(numCopter)
, minMotorSpeed_(numCopter)
{
    for (int indx = 0; indx < numCopter; indx++){
        motorFrame_.at(indx).setIdentity();
        thrustCoefficient_.at(indx) = thrustCoefficient;
        torqueCoefficient_.at(indx) = torqueCoefficient;
        motorTimeConstant_.at(indx) = motorTimeConstant;
        maxMotorSpeed_.at(indx) = maxMotorSpeed;
        minMotorSpeed_.at(indx) = minMotorSpeed;
        motorDirection_.at(indx) = 1;
        motorSpeed_.at(indx) = 0.;
    }

    vehicleMass_ = vehicleMass;
    vehicleInertia_ = vehicleInertia;
    aeroMomentCoefficient_ = aeroMomentCoefficient;
    dragCoefficient_ = dragCoefficient;
    momentProcessNoiseAutoCorrelation_ = momentProcessNoiseAutoCorrelation;
    forceProcessNoiseAutoCorrelation_ = forceProcessNoiseAutoCorrelation;

    gravity_ = gravity;
}

MulticopterDynamicsSim::MulticopterDynamicsSim(int numCopter)
: numCopter_(numCopter)
, motorFrame_(numCopter)
, motorDirection_(numCopter)
, thrustCoefficient_(numCopter)
, torqueCoefficient_(numCopter)
, motorSpeed_(numCopter)
, motorTimeConstant_(numCopter)
, maxMotorSpeed_(numCopter)
, minMotorSpeed_(numCopter)
{
    for (int indx = 0; indx < numCopter; indx++){
        motorFrame_.at(indx).setIdentity();
        thrustCoefficient_.at(indx) = 0.;
        torqueCoefficient_.at(indx) = 0.;
        motorTimeConstant_.at(indx) = 0.;
        maxMotorSpeed_.at(indx) = 0.;
        minMotorSpeed_.at(indx) = 0.;
        motorDirection_.at(indx) = 1;
        motorSpeed_.at(indx) = 0.;
    }

    // Default is NED, but can be set by changing gravity direction
    gravity_ << 0.,0.,9.81;
}

void MulticopterDynamicsSim::setVehicleProperties(double vehicleMass,
                                            const Eigen::Matrix3d & vehicleInertia, 
                                            const Eigen::Matrix3d & aeroMomentCoefficient,
                                            double dragCoefficient,
                                            double momentProcessNoiseAutoCorrelation,
                                            double forceProcessNoiseAutoCorrelation){
    vehicleMass_ = vehicleMass;
    vehicleInertia_ = vehicleInertia;
    aeroMomentCoefficient_ = aeroMomentCoefficient;
    dragCoefficient_ = dragCoefficient;
    momentProcessNoiseAutoCorrelation_ = momentProcessNoiseAutoCorrelation;
    forceProcessNoiseAutoCorrelation_ = forceProcessNoiseAutoCorrelation;
}

void MulticopterDynamicsSim::setGravityVector(const Eigen::Vector3d & gravity){
    gravity_ = gravity;
}

void MulticopterDynamicsSim::setMotorFrame(const Eigen::Isometry3d & motorFrame, int motorDirection, int motorIndex){
    motorFrame_.at(motorIndex) = motorFrame;
    motorDirection_.at(motorIndex) = motorDirection;
}

void MulticopterDynamicsSim::setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                             double minMotorSpeed, double maxMotorSpeed, int motorIndex){
    thrustCoefficient_.at(motorIndex) = thrustCoefficient;
    torqueCoefficient_.at(motorIndex) = torqueCoefficient;
    motorTimeConstant_.at(motorIndex) = motorTimeConstant;
    maxMotorSpeed_.at(motorIndex) = maxMotorSpeed;
    minMotorSpeed_.at(motorIndex) = minMotorSpeed;
}

void MulticopterDynamicsSim::setMotorProperties(double thrustCoefficient, double torqueCoefficient, double motorTimeConstant,
                                             double minMotorSpeed, double maxMotorSpeed){
    for (int motorIndex = 0; motorIndex < numCopter_; motorIndex++){
        setMotorProperties(thrustCoefficient, torqueCoefficient, motorTimeConstant,
                                             minMotorSpeed, maxMotorSpeed, motorIndex);
    }
}

void MulticopterDynamicsSim::setMotorSpeed(double motorSpeed, int motorIndex){
    motorSpeed_.at(motorIndex) = motorSpeed;
}

void MulticopterDynamicsSim::setMotorSpeed(double motorSpeed){
    for (int motorIndex = 0; motorIndex < numCopter_; motorIndex++){
        setMotorSpeed(motorSpeed, motorIndex);
    }
}

void MulticopterDynamicsSim::resetMotorSpeeds(void){
    for (int indx = 0; indx < numCopter_; indx++){
        motorSpeed_.at(indx) = 0.;
    }
}

void MulticopterDynamicsSim::setVehiclePosition(const Eigen::Vector3d & position,
                                             const Eigen::Quaterniond & attitude){
    position_ = position;
    attitude_ = attitude;

    angularVelocity_.setZero();
    velocity_.setZero();

    resetMotorSpeeds();
}

void MulticopterDynamicsSim::setVehicleState(const Eigen::Vector3d & position,
                                              const Eigen::Vector3d & velocity,
                                              const Eigen::Vector3d & angularVelocity,
                                              const Eigen::Quaterniond & attitude,
                                              const std::vector<double> & motorSpeed){
    position_ = position;
    velocity_ = velocity;
    angularVelocity_ = angularVelocity;
    attitude_ = attitude;
    for (int indx = 0; indx < numCopter_; indx++){
        motorSpeed_.at(indx) = motorSpeed.at(indx);
    }
}

void MulticopterDynamicsSim::getVehicleState(Eigen::Vector3d & position,
                                          Eigen::Vector3d & velocity,
                                          Eigen::Vector3d & angularVelocity,
                                          Eigen::Quaterniond & attitude,
                                          std::vector<double> & motorSpeed){
    position = position_;
    velocity = velocity_;
    angularVelocity = angularVelocity_;
    attitude = attitude_;
    motorSpeed = motorSpeed_;
}

Eigen::Vector3d MulticopterDynamicsSim::getVehiclePosition(void){
    return position_;
}

Eigen::Quaterniond MulticopterDynamicsSim::getVehicleAttitude(void){
    return attitude_;
}

Eigen::Vector3d MulticopterDynamicsSim::getVehicleVelocity(void){
    return velocity_;
}

Eigen::Vector3d MulticopterDynamicsSim::getVehicleAngularVelocity(void){
    return angularVelocity_;
}

// In body frame
Eigen::Vector3d MulticopterDynamicsSim::getVehicleSpecificForce(void){
    Eigen::Vector3d specificForce = getThrust(motorSpeed_);
    specificForce += attitude_.inverse()*getDragForce(velocity_)/vehicleMass_;
    specificForce += attitude_.inverse()*stochForce_/vehicleMass_;
    
    return specificForce;
}

Eigen::Vector3d MulticopterDynamicsSim::getThrust(const std::vector<double> & motorSpeed){
    Eigen::Vector3d thrust = Eigen::Vector3d::Zero();

    for (int indx = 0; indx < numCopter_; indx++){

        Eigen::Vector3d motorThrust(0.,0.,fabs(motorSpeed.at(indx))*motorSpeed.at(indx)*thrustCoefficient_.at(indx));
        thrust += motorFrame_.at(indx).linear()*motorThrust;
    }

    return thrust;
}

Eigen::Vector3d MulticopterDynamicsSim::getControlMoment(const std::vector<double> & motorSpeed){
    Eigen::Vector3d controlMoment = Eigen::Vector3d::Zero();

    for (int indx = 0; indx < numCopter_; indx++){
        // Moment due to thrust
        Eigen::Vector3d motorThrust(0.,0.,fabs(motorSpeed.at(indx))*motorSpeed.at(indx)*thrustCoefficient_.at(indx));
        controlMoment += motorFrame_.at(indx).translation().cross(motorFrame_.at(indx).linear()*motorThrust);

        // Moment due to torque
        Eigen::Vector3d motorTorque(0.,0.,motorDirection_.at(indx)*fabs(motorSpeed.at(indx))*motorSpeed.at(indx)*torqueCoefficient_.at(indx));
        controlMoment += motorFrame_.at(indx).linear()*motorTorque;
    }

    return controlMoment;
}

Eigen::Vector3d MulticopterDynamicsSim::getAeroMoment(const Eigen::Vector3d & angularVelocity){
    return (-angularVelocity.norm()*aeroMomentCoefficient_*angularVelocity);
}

Eigen::Vector3d MulticopterDynamicsSim::getDragForce(const Eigen::Vector3d & velocity){
    return (-dragCoefficient_*velocity.norm()*velocity);
}

void MulticopterDynamicsSim::getIMUMeasurement(Eigen::Vector3d & accOutput, Eigen::Vector3d & gyroOutput){
    imu_.getMeasurement(accOutput, gyroOutput, getVehicleSpecificForce(), angularVelocity_);
}

void MulticopterDynamicsSim::proceedState_ExplicitEuler(double dt_secs, const std::vector<double> & motorSpeedCommand){

    std::vector<double> motorSpeed(motorSpeed_);
    Eigen::Vector3d position = position_;
    Eigen::Vector3d velocity = velocity_;
    Eigen::Vector3d angularVelocity = angularVelocity_;
    Eigen::Quaterniond attitude = attitude_;

    stochForce_ /= sqrt(dt_secs);

    Eigen::Vector3d stochMoment;
    stochMoment << sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    std::vector<double> motorSpeedDer(numCopter_);
    getMotorSpeedDerivative(motorSpeedDer,motorSpeed,motorSpeedCommand);
    Eigen::Vector3d positionDer = velocity;
    Eigen::Vector3d velocityDer = getVelocityDerivative(attitude,stochForce_,velocity,motorSpeed);
    Eigen::Vector4d attitudeDer = getAttitudeDerivative(attitude,angularVelocity);
    Eigen::Vector3d angularVelocityDer = getAngularVelocityDerivative(motorSpeed,angularVelocity,stochMoment);

    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed_,dt_secs);
    vectorBoundOp(motorSpeed_,motorSpeed_,minMotorSpeed_,maxMotorSpeed_);
    position_ = position + positionDer*dt_secs;
    velocity_ = velocity + velocityDer*dt_secs;
    angularVelocity_ = angularVelocity + angularVelocityDer*dt_secs;
    attitude_.coeffs() = attitude.coeffs() + attitudeDer*dt_secs;

    attitude_.normalize();

    stochForce_ << sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_);

    imu_.proceedBiasDynamics(dt_secs);
}

void MulticopterDynamicsSim::proceedState_RK4(double dt_secs, const std::vector<double> & motorSpeedCommand){

    std::vector<double> motorSpeed(motorSpeed_);
    Eigen::Vector3d position = position_;
    Eigen::Vector3d velocity = velocity_;
    Eigen::Vector3d angularVelocity = angularVelocity_;
    Eigen::Quaterniond attitude = attitude_;

    stochForce_ /= sqrt(dt_secs);

    Eigen::Vector3d stochMoment;
    stochMoment << sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(momentProcessNoiseAutoCorrelation_/dt_secs)*standardNormalDistribution_(randomNumberGenerator_);

    // k1
    std::vector<double> motorSpeedDer(numCopter_);
    getMotorSpeedDerivative(motorSpeedDer,motorSpeed_,motorSpeedCommand);
    vectorScalarProd(motorSpeedDer,motorSpeedDer,dt_secs);
    Eigen::Vector3d positionDer = dt_secs*velocity_;
    Eigen::Vector3d velocityDer = dt_secs*getVelocityDerivative(attitude_,stochForce_,velocity_,motorSpeed_);
    Eigen::Vector4d attitudeDer = dt_secs*getAttitudeDerivative(attitude_,angularVelocity_);
    Eigen::Vector3d angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeed_,angularVelocity_,stochMoment);

    // x + 1/6*(k1)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed,(1./6.));
    position += (1./6.)*positionDer;
    velocity += (1./6.)*velocityDer;
    attitude.coeffs() += (1./6.)*attitudeDer;
    attitude.normalize();
    angularVelocity += (1./6.)*angularVelocityDer;

    // k2
    std::vector<double> motorSpeedIntermediate(numCopter_);
    Eigen::Quaterniond attitudeIntermediate;

    vectorAffineOp(motorSpeed_,motorSpeedDer,motorSpeedIntermediate,0.5);  // x + 0.5*(k1)
    vectorBoundOp(motorSpeedIntermediate,motorSpeedIntermediate,minMotorSpeed_,maxMotorSpeed_);
    attitudeIntermediate.coeffs() = attitude_.coeffs() + attitudeDer*0.5;
    attitudeIntermediate.normalize();

    getMotorSpeedDerivative(motorSpeedDer, motorSpeedIntermediate, motorSpeedCommand);
    vectorScalarProd(motorSpeedDer, motorSpeedDer, dt_secs);
    positionDer = dt_secs*(velocity_ + 0.5*velocityDer);
    velocityDer = dt_secs*getVelocityDerivative(attitudeIntermediate,stochForce_,(velocity_ + 0.5*velocityDer), motorSpeedIntermediate);
    attitudeDer = dt_secs*getAttitudeDerivative(attitudeIntermediate,(angularVelocity_ + 0.5*angularVelocityDer));
    angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeedIntermediate,(angularVelocity_ + 0.5*angularVelocityDer),stochMoment);

    // x + 1/6*(k1 + 2*k2)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed,(1./3.));
    position += (1./3.)*positionDer;
    velocity += (1./3.)*velocityDer;
    attitude.coeffs() += (1./3.)*attitudeDer;
    attitude.normalize();
    angularVelocity += (1./3.)*angularVelocityDer;

    // k3
    vectorAffineOp(motorSpeed_,motorSpeedDer,motorSpeedIntermediate,0.5);
    vectorBoundOp(motorSpeedIntermediate,motorSpeedIntermediate,minMotorSpeed_,maxMotorSpeed_);
    attitudeIntermediate.coeffs() = attitude_.coeffs() + attitudeDer*0.5;
    attitudeIntermediate.normalize();

    getMotorSpeedDerivative(motorSpeedDer, motorSpeedIntermediate, motorSpeedCommand);
    vectorScalarProd(motorSpeedDer, motorSpeedDer, dt_secs);
    positionDer = dt_secs*(velocity_ + 0.5*velocityDer);
    velocityDer = dt_secs*getVelocityDerivative(attitudeIntermediate,stochForce_,(velocity_ + 0.5*velocityDer), motorSpeedIntermediate);
    attitudeDer = dt_secs*getAttitudeDerivative(attitudeIntermediate,(angularVelocity_ + 0.5*angularVelocityDer));
    angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeedIntermediate,(angularVelocity_ + 0.5*angularVelocityDer),stochMoment);

    // x + 1/6*(k1 + 2*k2 + 2*k3)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed,(1./3.));
    position += (1./3.)*positionDer;
    velocity += (1./3.)*velocityDer;
    attitude.coeffs() += (1./3.)*attitudeDer;
    attitude.normalize();
    angularVelocity += (1./3.)*angularVelocityDer;

    // k4
    vectorAffineOp(motorSpeed_,motorSpeedDer,motorSpeedIntermediate,1.);
    vectorBoundOp(motorSpeedIntermediate,motorSpeedIntermediate,minMotorSpeed_,maxMotorSpeed_);
    attitudeIntermediate.coeffs() = attitude_.coeffs() + attitudeDer;
    attitudeIntermediate.normalize();

    getMotorSpeedDerivative(motorSpeedDer, motorSpeedIntermediate, motorSpeedCommand);
    vectorScalarProd(motorSpeedDer, motorSpeedDer, dt_secs);
    positionDer = dt_secs*(velocity_ + velocityDer);
    velocityDer = dt_secs*getVelocityDerivative(attitudeIntermediate,stochForce_,(velocity_ + velocityDer), motorSpeedIntermediate);
    attitudeDer = dt_secs*getAttitudeDerivative(attitudeIntermediate,(angularVelocity_ + angularVelocityDer));
    angularVelocityDer = dt_secs*getAngularVelocityDerivative(motorSpeedIntermediate,(angularVelocity_ + angularVelocityDer),stochMoment);

    // x + 1/6*(k1 + 2*k2 + 2*k3 + k4)
    vectorAffineOp(motorSpeed,motorSpeedDer,motorSpeed_,(1./6.));
    vectorBoundOp(motorSpeed_,motorSpeed_,minMotorSpeed_,maxMotorSpeed_);
    position_ = position + positionDer*(1./6.);
    velocity_ = velocity + velocityDer*(1./6.);
    attitude_.coeffs() = attitude.coeffs() + attitudeDer*(1./6.);
    attitude_.normalize();
    angularVelocity_ = angularVelocity + angularVelocityDer*(1./6.);

    stochForce_ << sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_),
                   sqrt(forceProcessNoiseAutoCorrelation_)*standardNormalDistribution_(randomNumberGenerator_);

    imu_.proceedBiasDynamics(dt_secs);
}

void MulticopterDynamicsSim::vectorAffineOp(const std::vector<double> & vec1, const std::vector<double> & vec2,
                                         std::vector<double> & vec3, double val){
    // vec3 = vec1 + val*vec2
    std::transform(vec1.begin(), vec1.end(), vec2.begin(), vec3.begin(), [val](const double & vec1val, const double & vec2val)->double{return (vec1val + val*vec2val);});
}

void MulticopterDynamicsSim::vectorBoundOp(const std::vector<double> & vec1, std::vector<double> & vec2,
                                         const std::vector<double> &  minvec, const std::vector<double> & maxvec){
    // vec2 = max(minvalue, min(maxvalue, vec1))
    std::transform(vec1.begin(), vec1.end(), maxvec.begin(), vec2.begin(), [](const double & vec1val, const double & maxvalue)->double{return fmin(vec1val,maxvalue);});
    std::transform(vec2.begin(), vec2.end(), minvec.begin(), vec2.begin(), [](const double & vec2val, const double & minvalue)->double{return fmax(vec2val,minvalue);});
}

void MulticopterDynamicsSim::vectorScalarProd(const std::vector<double> & vec1, std::vector<double> & vec2, double val){
    // vec2 = val*vec1
    std::transform(vec1.begin(), vec1.end(), vec2.begin(), [val](const double & vec1val){return vec1val*val;});
}

void MulticopterDynamicsSim::getMotorSpeedDerivative(std::vector<double> & motorSpeedDer,
                                                  const std::vector<double> & motorSpeed,
                                                  const std::vector<double> & motorSpeedCommand){
    for (int indx = 0; indx < numCopter_; indx++){
        motorSpeedDer.at(indx) = (motorSpeedCommand.at(indx) - motorSpeed.at(indx))/motorTimeConstant_.at(indx);
    }
}

Eigen::Vector3d MulticopterDynamicsSim::getVelocityDerivative(const Eigen::Quaterniond & attitude, const Eigen::Vector3d & stochForce,
                                        const Eigen::Vector3d & velocity, const std::vector<double> & motorSpeed){

    return (gravity_ + (attitude*getThrust(motorSpeed) + getDragForce(velocity) + stochForce)/vehicleMass_);
}

Eigen::Vector4d MulticopterDynamicsSim::getAttitudeDerivative(const Eigen::Quaterniond & attitude, const Eigen::Vector3d & angularVelocity){
    Eigen::Quaterniond angularVelocityQuad;
    angularVelocityQuad.w() = 0;
    angularVelocityQuad.vec() = angularVelocity;

    return (0.5*(attitude*angularVelocityQuad).coeffs());
}

Eigen::Vector3d MulticopterDynamicsSim::getAngularVelocityDerivative(const std::vector<double> & motorSpeed,
                                const Eigen::Vector3d & angularVelocity, const Eigen::Vector3d & stochMoment){

    return (vehicleInertia_.inverse()*(getControlMoment(motorSpeed) + getAeroMoment(angularVelocity) + stochMoment 
                                                       - angularVelocity.cross(vehicleInertia_*angularVelocity)));
}