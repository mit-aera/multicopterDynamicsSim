#include "pybind11/pybind11.h"
#include "pybind11/stl.h"
#include "pybind11/eigen.h"
#include "multicopterDynamicsSim.hpp"

namespace py = pybind11;

void bind_MulticopterDynamicsSim(py::module &m);

PYBIND11_MODULE(multicopter_dynamics_sim, m) {
    m.doc() = "Python package of multicopterDynamicsSim";
    bind_MulticopterDynamicsSim(m);
}

void bind_MulticopterDynamicsSim(py::module &m) {
    py::class_<Eigen::Isometry3d>(m, "Isometry3d", py::buffer_protocol())
        .def(py::init([](const Eigen::Matrix<double, 3, 4> & arg) {
            Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
            r.rotate(Eigen::Quaterniond(arg.topLeftCorner<3, 3>()));
            r.translation() = Eigen::Vector3d(arg.col(3));
            return r;}))
        .def(py::init([](Eigen::VectorXd arg) {
            Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
            r.rotate(Eigen::Quaterniond(arg[3],arg[4],arg[5],arg[6]));
            r.translation() = Eigen::Vector3d(arg.segment(0,3));
            return r;}))
        .def_buffer([](Eigen::Isometry3d& r)->py::buffer_info {
            return py::buffer_info(
                r.data(), // Pointer to buffer
                sizeof(double), // Size of one scalar
                py::format_descriptor<double>::format(), // Python struct-style format descriptor
                2, // Number of dimensions
                { 3, 4 }, // Buffer dimensions
                { sizeof(double) * 3, sizeof(double) } // Strides (in bytes) for each index
            );
        });

    py::class_<Eigen::Quaterniond>(m, "Quaterniond", py::buffer_protocol())
        .def(py::init([](const Eigen::Vector4d & arg) {
            Eigen::Quaterniond q(arg[0],arg[1],arg[2],arg[3]);
            return q;}))
        .def_buffer([](Eigen::Quaterniond& r)->py::buffer_info {
            return py::buffer_info(
                r.coeffs().data(), // Pointer to buffer
                sizeof(double), // Size of one scalar
                py::format_descriptor<double>::format(), // Python struct-style format descriptor
                1, // Number of dimensions
                { 4 }, // Buffer dimensions
                { sizeof(double) } // Strides (in bytes) for each index
            );
        });

    py::class_<MulticopterDynamicsSim>(m, "MulticopterDynamicsSim")
        .def(py::init<
            int, double, double, double, double, double, double, double,
            const Eigen::Matrix3d &, const Eigen::Matrix3d & ,
            double, double, double, const Eigen::Vector3d &>(),
            py::arg("numCopter")=4, py::arg("thrustCoefficient")=1.91e-6, py::arg("torqueCoefficient")=2.6e-7,
            py::arg("minMotorSpeed")=0, py::arg("maxMotorSpeed")=2200.,
            py::arg("motorTimeConstant")=0.02, py::arg("motorRotationalInertia")=6.62e-6,
            py::arg("vehicleMass")=1.0,
            py::arg("vehicleInertia")=Eigen::Vector3d(.0049,0.0049,0.0069).matrix().asDiagonal(), 
            py::arg("aeroMomentCoefficient")=Eigen::Matrix3d::Zero(),
            py::arg("dragCoefficient")=0.1,
            py::arg("momentProcessNoiseAutoCorrelation")=0.00025,
            py::arg("forceProcessNoiseAutoCorrelation")=0.0005,
            py::arg("gravity")=Eigen::Vector3d(0.,0.,9.81))
        .def(py::init<int>(), py::arg("numCopter"))
        .def("setRandomSeed", &MulticopterDynamicsSim::setRandomSeed, "set random seed",
            py::arg("multicopterSeed"), py::arg("imuSeed"))
        .def("setVehicleProperties", &MulticopterDynamicsSim::setVehicleProperties, "set vehicle properties",
            py::arg("vehicleMass"), py::arg("vehicleInertia"), py::arg("aeroMomentCoefficient"), 
            py::arg("dragCoefficient"), 
            py::arg("momentProcessNoiseAutoCorrelation"), 
            py::arg("forceProcessNoiseAutoCorrelation"))
        .def("setGravityVector", &MulticopterDynamicsSim::setGravityVector, "set gravity vector", py::arg("gravity"))

        .def("setMotorFrame", &MulticopterDynamicsSim::setMotorFrame, "set motor frame",
            py::arg("motorFrame"), py::arg("motorDirection"), py::arg("motorIndex"))
        .def("setMotorFrame", [](MulticopterDynamicsSim & multicopterSim, 
            const Eigen::Matrix<double, 3, 4> & motorFrame, int motorDirection, int motorIndex) {
            Eigen::Isometry3d r = Eigen::Isometry3d::Identity();
            r.rotate(Eigen::Quaterniond(motorFrame.topLeftCorner<3, 3>()));
            r.translation() = Eigen::Vector3d(motorFrame.col(3));
            multicopterSim.setMotorFrame(r, motorDirection, motorIndex);
            return;
            }, "set motor frame",
            py::arg("motorFrame"), py::arg("motorDirection"), py::arg("motorIndex"))

        .def("setMotorProperties", (void (MulticopterDynamicsSim::*)(double, double, double, double, double, double, int)) 
            &MulticopterDynamicsSim::setMotorProperties, "set motor properties",
            py::arg("thrustCoefficient"), py::arg("torqueCoefficient"), py::arg("motorTimeConstant"),
            py::arg("minMotorSpeed"), py::arg("maxMotorSpeed"), py::arg("rotationalInertia"), py::arg("motorIndex"))
        .def("setMotorProperties", (void (MulticopterDynamicsSim::*)(double, double, double, double, double, double)) 
            &MulticopterDynamicsSim::setMotorProperties, "set motor properties",
            py::arg("thrustCoefficient"), py::arg("torqueCoefficient"), py::arg("motorTimeConstant"),
            py::arg("minMotorSpeed"), py::arg("maxMotorSpeed"), py::arg("rotationalInertia"))
        .def("setMotorSpeed", (void (MulticopterDynamicsSim::*)(double))
            &MulticopterDynamicsSim::setMotorSpeed, "set motor speed", py::arg("motorSpeed"))
        .def("setMotorSpeed", (void (MulticopterDynamicsSim::*)(double, int))
            &MulticopterDynamicsSim::setMotorSpeed, "set motor speed", py::arg("motorSpeed"), py::arg("motorIndex"))
        .def("resetMotorSpeeds", &MulticopterDynamicsSim::resetMotorSpeeds, "reset motor speed to zero")

        .def("setVehiclePosition", &MulticopterDynamicsSim::setVehiclePosition, "set vehicle position",
            py::arg("position"), py::arg("attitude"))
        .def("setVehiclePosition", [](MulticopterDynamicsSim & multicopterSim, 
            const Eigen::Vector3d & position, const Eigen::Vector4d & attitude) {
            Eigen::Quaterniond q(attitude[0],attitude[1],attitude[2],attitude[3]);
            multicopterSim.setVehiclePosition(position, q);
            return;
            }, "set vehicle position",
            py::arg("position"), py::arg("attitude"))
        .def("setVehicleState", &MulticopterDynamicsSim::setVehicleState, "set vehicle state",
            py::arg("position"), py::arg("velocity"), py::arg("angularVelocity"),
            py::arg("attitude"), py::arg("motorSpeed"))
        .def("setVehicleState", [](MulticopterDynamicsSim & multicopterSim, 
            const Eigen::Vector3d & position,
            const Eigen::Vector3d & velocity,
            const Eigen::Vector3d & angularVelocity,
            const Eigen::Vector4d & attitude,
            const std::vector<double> & motorSpeed) {
            Eigen::Quaterniond q(attitude[0],attitude[1],attitude[2],attitude[3]);
            multicopterSim.setVehicleState(position, velocity, angularVelocity, q, motorSpeed);
            }, "set vehicle state",
            py::arg("position"), py::arg("velocity"), py::arg("angularVelocity"),
            py::arg("attitude"), py::arg("motorSpeed"))
        
        // .def("getVehicleState", &MulticopterDynamicsSim::getVehicleState, "get vehicle state")
        .def("getVehicleState", [](MulticopterDynamicsSim & multicopterSim) {
            Eigen::Vector3d position, velocity, angularVelocity;
            Eigen::Quaterniond attitude;
            std::vector<double> motorSpeed;
            multicopterSim.getVehicleState(position, velocity, angularVelocity, attitude, motorSpeed);
            Eigen::Vector4d attitude_vec;
            attitude_vec << attitude.w(), attitude.x(), attitude.y(), attitude.z();
            Eigen::VectorXd motorSpeed_vec = Eigen::Map<Eigen::VectorXd>(motorSpeed.data(), motorSpeed.size());
            // std::list<Eigen::VectorXd> vec;
            // vec.push_back(position);
            // vec.push_back(velocity);
            // vec.push_back(angularVelocity);
            // vec.push_back(attitude_vec);
            // vec.push_back(motorSpeed_vec);
            std::map<std::string, Eigen::VectorXd> vec;
            vec.insert(std::pair<std::string, Eigen::VectorXd>("position",position));
            vec.insert(std::pair<std::string, Eigen::VectorXd>("velocity",velocity));
            vec.insert(std::pair<std::string, Eigen::VectorXd>("angularVelocity",angularVelocity));
            vec.insert(std::pair<std::string, Eigen::VectorXd>("attitude",attitude_vec));
            vec.insert(std::pair<std::string, Eigen::VectorXd>("motorSpeed",motorSpeed_vec));
            return vec;
            }, "get vehicle state")

        .def("getVehiclePosition", &MulticopterDynamicsSim::getVehiclePosition, "get vehicle position")
        .def("getVehicleVelocity", &MulticopterDynamicsSim::getVehicleVelocity, "get vehicle velocity")
        .def("getVehicleAttitude", &MulticopterDynamicsSim::getVehicleAttitude, "get vehicle attitude")
        .def("getVehicleAngularVelocity", &MulticopterDynamicsSim::getVehicleAngularVelocity, "get vehicle angular velocity")

        .def("proceedState", [](MulticopterDynamicsSim & multicopterSim,
            double dt_secs, const std::vector<double> & motorSpeedCommand) {
            multicopterSim.proceedState_RK4(dt_secs, motorSpeedCommand);
            }, "proceed state",
            py::arg("dt_secs"), py::arg("motorSpeedCommand"))
        .def("proceedState_ExplicitEuler", &MulticopterDynamicsSim::proceedState_ExplicitEuler, "proceed state", 
            py::arg("dt_secs"), py::arg("motorSpeedCommand"))
        .def("proceedState_RK4", &MulticopterDynamicsSim::proceedState_RK4, "proceed state", 
            py::arg("dt_secs"), py::arg("motorSpeedCommand"))

        .def("getIMUMeasurement", [](MulticopterDynamicsSim & multicopterSim) {
            Eigen::Vector3d accOutput;
            Eigen::Vector3d gyroOutput;
            multicopterSim.getIMUMeasurement(accOutput, gyroOutput);
            std::map<std::string, Eigen::VectorXd> vec;
            vec.insert(std::pair<std::string, Eigen::VectorXd>("acc",accOutput));
            vec.insert(std::pair<std::string, Eigen::VectorXd>("gyro",gyroOutput));
            return vec;
            }, "get imu measurement")

        .def("setIMUBias", [](MulticopterDynamicsSim & multicopterSim,
            const Eigen::Vector3d & accBias, const Eigen::Vector3d & gyroBias,
            double accBiasProcessNoiseAutoCorrelation,
            double gyroBiasProcessNoiseAutoCorrelation) {
            multicopterSim.imu_.setBias(accBias, gyroBias, 
                accBiasProcessNoiseAutoCorrelation, 
                gyroBiasProcessNoiseAutoCorrelation);
            }, "set IMU bias",
            py::arg("accBias"), py::arg("gyroBias"), 
            py::arg("accBiasProcessNoiseAutoCorrelation"),
            py::arg("gyroBiasProcessNoiseAutoCorrelation"))
        .def("setIMUBias", [](MulticopterDynamicsSim & multicopterSim,
            double accBiasVariance, double gyroBiasVariance,
            double accBiasProcessNoiseAutoCorrelation,
            double gyroBiasProcessNoiseAutoCorrelation) {
            multicopterSim.imu_.setBias(accBiasVariance, gyroBiasVariance, 
                accBiasProcessNoiseAutoCorrelation, 
                gyroBiasProcessNoiseAutoCorrelation);
            }, "set IMU bias",
            py::arg("accBiasVariance"), py::arg("gyroBiasVariance"), 
            py::arg("accBiasProcessNoiseAutoCorrelation"),
            py::arg("gyroBiasProcessNoiseAutoCorrelation"))
        .def("setIMUBias", [](MulticopterDynamicsSim & multicopterSim,
            double accBiasVariance, double gyroBiasVariance) {
            multicopterSim.imu_.setBias(accBiasVariance, gyroBiasVariance);
            }, "set IMU bias",
            py::arg("accBiasVariance"), py::arg("gyroBiasVariance"))
        .def("setIMUNoiseVariance", [](MulticopterDynamicsSim & multicopterSim,
            double accMeasNoiseVariance, double gyroMeasNoiseVariance) {
            multicopterSim.imu_.setNoiseVariance(accMeasNoiseVariance, gyroMeasNoiseVariance);
            }, "set IMU noise variance",
            py::arg("accMeasNoiseVariance"), py::arg("gyroMeasNoiseVariance"))
        .def("setIMUOrientation", [](MulticopterDynamicsSim & multicopterSim,
            const Eigen::Vector4d & imuOrient) {
            Eigen::Quaterniond q(imuOrient[0],imuOrient[1],imuOrient[2],imuOrient[3]);
            multicopterSim.imu_.setOrientation(q);
            }, "set IMU orientation",
            py::arg("imuOrient"));
}
