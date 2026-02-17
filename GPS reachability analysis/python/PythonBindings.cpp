#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <gps/kin/InverseKinematicsGps.h>
#include <gps/kin/ISolutionValidator.h>
#include <gps/kin/GpsIKSolutionValidator.h>
#include <gps/mctrl/kin/ForwardKinematicsGps.h>
#include <gps/mctrl/kin/AxisConfigGroup.h>
#include <gps/mctrl/kin/AxisConfigFromIniFactory.h>
#include <gps/mctrl/kin/DHConfigFromIniFactory.h>
#include <gps/mctrl/kin/IDHConfig.h>
#include <gps/mctrl/kin/IAxisConfig.h>
#include <gps/mctrl/geom.h>

namespace py = pybind11;

PYBIND11_MODULE(gps_ik_python_bindings_renjie, m) {
    m.doc() = "GPS Kinematics Python bindings";

    // Bind Angle first (needed by Joints)
    py::class_<gps::mctrl::geom::Angle>(m, "Angle")
        .def_static("fromDegrees", &gps::mctrl::geom::Angle::fromDegrees)
        .def_static("fromRadians", &gps::mctrl::geom::Angle::fromRadians)
        .def("degrees", &gps::mctrl::geom::Angle::degrees)
        .def("radians", &gps::mctrl::geom::Angle::radians);

    // Bind Joints class
    py::class_<gps::mctrl::geom::Joints>(m, "Joints")
        .def(py::init<double, gps::mctrl::geom::Angle, gps::mctrl::geom::Angle, 
                      gps::mctrl::geom::Angle, gps::mctrl::geom::Angle>())
        .def_readwrite("vertical", &gps::mctrl::geom::Joints::vertical)
        .def_readwrite("shoulder", &gps::mctrl::geom::Joints::shoulder)
        .def_readwrite("elbow", &gps::mctrl::geom::Joints::elbow)
        .def_readwrite("roll", &gps::mctrl::geom::Joints::roll)
        .def_readwrite("pitch", &gps::mctrl::geom::Joints::pitch);

    // Bind Point3d
    py::class_<gps::mctrl::geom::Point3d>(m, "Point3d")
        .def(py::init<double, double, double>())
        .def("x", &gps::mctrl::geom::Point3d::x)
        .def("y", &gps::mctrl::geom::Point3d::y)
        .def("z", &gps::mctrl::geom::Point3d::z);

    // Bind Vector3d
    py::class_<gps::mctrl::geom::Vector3d>(m, "Vector3d")
        .def(py::init<>())
        .def(py::init<double, double, double>())
        .def(py::init<const gps::mctrl::geom::UnitVector3d&>())
        .def_static("zero", &gps::mctrl::geom::Vector3d::zero)
        .def("x", &gps::mctrl::geom::Vector3d::x)
        .def("y", &gps::mctrl::geom::Vector3d::y)
        .def("z", &gps::mctrl::geom::Vector3d::z)
        .def("setX", &gps::mctrl::geom::Vector3d::setX)
        .def("setY", &gps::mctrl::geom::Vector3d::setY)
        .def("setZ", &gps::mctrl::geom::Vector3d::setZ)
        .def("magnitude", &gps::mctrl::geom::Vector3d::magnitude)
        .def("magnitudeSquared", &gps::mctrl::geom::Vector3d::magnitudeSquared)
        .def("unitVector", &gps::mctrl::geom::Vector3d::unitVector)
        .def("cross", &gps::mctrl::geom::Vector3d::cross)
        .def("dot", &gps::mctrl::geom::Vector3d::dot);

    // Bind UnitVector3d
    py::class_<gps::mctrl::geom::UnitVector3d>(m, "UnitVector3d")
        .def_static("unitX", &gps::mctrl::geom::UnitVector3d::unitX)
        .def_static("unitY", &gps::mctrl::geom::UnitVector3d::unitY)
        .def_static("unitZ", &gps::mctrl::geom::UnitVector3d::unitZ)
        .def("x", &gps::mctrl::geom::UnitVector3d::x)
        .def("y", &gps::mctrl::geom::UnitVector3d::y)
        .def("z", &gps::mctrl::geom::UnitVector3d::z)
        .def_static("magnitude", &gps::mctrl::geom::UnitVector3d::magnitude)
        .def_static("magnitudeSquared", &gps::mctrl::geom::UnitVector3d::magnitudeSquared)
        .def("dot", py::overload_cast<const gps::mctrl::geom::UnitVector3d&>(&gps::mctrl::geom::UnitVector3d::dot, py::const_))
        .def("angleWith", &gps::mctrl::geom::UnitVector3d::angleWith);

    // Bind Ray3d
    py::class_<gps::mctrl::geom::Ray3d>(m, "Ray3d")
        .def(py::init<>())
        .def(py::init<const gps::mctrl::geom::Point3d&, const gps::mctrl::geom::UnitVector3d&>())
        .def("point", py::overload_cast<>(&gps::mctrl::geom::Ray3d::point, py::const_))
        .def("dir", py::overload_cast<>(&gps::mctrl::geom::Ray3d::dir, py::const_))
        .def("setPoint", &gps::mctrl::geom::Ray3d::setPoint)
        .def("setDir", &gps::mctrl::geom::Ray3d::setDir)
        .def("translated", py::overload_cast<double>(&gps::mctrl::geom::Ray3d::translated, py::const_))
        .def("pointAtDistance", &gps::mctrl::geom::Ray3d::pointAtDistance);

    // Bind Affine3d
    py::class_<gps::mctrl::geom::Affine3d>(m, "Affine3d")
        .def(py::init<>())
        .def(py::init<const gps::mctrl::geom::UnitVector3d&,
                      const gps::mctrl::geom::UnitVector3d&,
                      const gps::mctrl::geom::UnitVector3d&,
                      const gps::mctrl::geom::Point3d&>(),
             py::arg("x"), py::arg("y"), py::arg("z"), py::arg("origin"))
        .def_static("identity", &gps::mctrl::geom::Affine3d::identity)
        .def("origin", &gps::mctrl::geom::Affine3d::origin, "Get origin point")
        .def("setOrigin", &gps::mctrl::geom::Affine3d::setOrigin,
             py::arg("origin"), "Set origin point")
        .def("xAxis", &gps::mctrl::geom::Affine3d::xAxis,
             "Get X axis unit vector")
        .def("yAxis", &gps::mctrl::geom::Affine3d::yAxis,
             "Get Y axis unit vector")
        .def("zAxis", &gps::mctrl::geom::Affine3d::zAxis,
             "Get Z axis unit vector")
        .def("setXaxis", &gps::mctrl::geom::Affine3d::setXaxis,
             py::arg("xAxis"))
        .def("setYaxis", &gps::mctrl::geom::Affine3d::setYaxis,
             py::arg("yAxis"))
        .def("setZaxis", &gps::mctrl::geom::Affine3d::setZaxis,
             py::arg("zAxis"))
        .def("inverse", &gps::mctrl::geom::Affine3d::inverse,
             "Get inverse transform")
        .def_static("fromRotationX", &gps::mctrl::geom::Affine3d::fromRotationX,
                    py::arg("angle"))
        .def_static("fromRotationY", &gps::mctrl::geom::Affine3d::fromRotationY,
                    py::arg("angle"))
        .def_static("fromRotationZ", &gps::mctrl::geom::Affine3d::fromRotationZ,
                    py::arg("angle"))
        .def("__mul__",
             py::overload_cast<const gps::mctrl::geom::Affine3d&>(
                 &gps::mctrl::geom::Affine3d::operator*, py::const_),
             "Multiply two transforms")
        .def("__mul__",
             py::overload_cast<const gps::mctrl::geom::Point3d&>(
                 &gps::mctrl::geom::Affine3d::operator*, py::const_),
             "Transform a point")
        .def("__mul__",
             py::overload_cast<const gps::mctrl::geom::Vector3d&>(
                 &gps::mctrl::geom::Affine3d::operator*, py::const_),
             "Transform a vector");

    // Bind IDHConfig (opaque - created via factory)
    py::class_<gps::mctrl::kin::IDHConfig, std::shared_ptr<gps::mctrl::kin::IDHConfig>>(m, "IDHConfig");

    // Bind DHConfigFromIniFactory
    py::class_<gps::mctrl::kin::DHConfigFromIniFactory>(m, "DHConfigFromIniFactory")
        .def_static("createInstance", &gps::mctrl::kin::DHConfigFromIniFactory::createInstance,
                    py::arg("iniFilePath") = "./DHConfig.ini",
                    "Create a DH config from an INI file");

    // Bind IAxisConfig (opaque - created via factory)
    py::class_<gps::mctrl::kin::IAxisConfig, std::shared_ptr<gps::mctrl::kin::IAxisConfig>>(m, "IAxisConfig");

    // Bind AxisConfigFromIniFactory
    py::class_<gps::mctrl::kin::AxisConfigFromIniFactory>(m, "AxisConfigFromIniFactory")
        .def_static("createInstance", &gps::mctrl::kin::AxisConfigFromIniFactory::createInstance,
                    py::arg("iniFilePath") = "./config.ini",
                    "Create an axis config from an INI file");

    // Bind ForwardKinematicsGps (requires DHConfig)
    py::class_<gps::mctrl::kin::ForwardKinematicsGps,
               std::shared_ptr<gps::mctrl::kin::ForwardKinematicsGps>>(
        m, "ForwardKinematicsGps")
        .def(py::init<std::shared_ptr<gps::mctrl::kin::IDHConfig>>(),
             py::arg("dhConfig"),
             "Create forward kinematics solver with DH config")
        .def("eeTip_base", &gps::mctrl::kin::ForwardKinematicsGps::eeTip_base,
             py::arg("joints"), py::arg("robotTool_X_eeTip"),
             "Compute end effector tip pose in base frame")
        .def("base_T_vertical",
             &gps::mctrl::kin::ForwardKinematicsGps::base_T_vertical,
             py::arg("vertical"), "Transform from base to vertical frame")
        .def("vertical_T_shoulder",
             &gps::mctrl::kin::ForwardKinematicsGps::vertical_T_shoulder,
             py::arg("shoulder"), "Transform from vertical to shoulder frame")
        .def("shoulder_T_elbow",
             &gps::mctrl::kin::ForwardKinematicsGps::shoulder_T_elbow,
             py::arg("elbow"), "Transform from shoulder to elbow frame")
        .def("elbow_T_roll",
             &gps::mctrl::kin::ForwardKinematicsGps::elbow_T_roll,
             py::arg("roll"), "Transform from elbow to roll frame")
        .def("roll_T_pitch",
             &gps::mctrl::kin::ForwardKinematicsGps::roll_T_pitch,
             py::arg("pitch"), "Transform from roll to pitch frame")
        .def("base_T_tool", &gps::mctrl::kin::ForwardKinematicsGps::base_T_tool,
             py::arg("joints"), "Transform from base to tool frame")
        .def("getBaseOffsets",
             &gps::mctrl::kin::ForwardKinematicsGps::getBaseOffsets,
             "Get DH base offset transform");

    // Bind AxisConfigGroup (CONCRETE - takes IAxisConfig in constructor)
    py::class_<gps::mctrl::kin::AxisConfigGroup, std::shared_ptr<gps::mctrl::kin::AxisConfigGroup>>(m, "AxisConfigGroup")
        .def(py::init<std::shared_ptr<gps::mctrl::kin::IAxisConfig>>(),
             py::arg("config"),
             "Create axis config group from IAxisConfig")
        .def("allAxisPositionsAreReachable", &gps::mctrl::kin::AxisConfigGroup::allAxisPositionsAreReachable,
             py::arg("axes"),
             py::arg("useKinematicRangeLimits") = true,
             "Check if all axis positions are reachable")
        .def("maxJointPositions", &gps::mctrl::kin::AxisConfigGroup::maxJointPositions,
             py::arg("useKinematicRangeLimits") = true,
             "Get maximum joint positions")
        .def("minJointPositions", &gps::mctrl::kin::AxisConfigGroup::minJointPositions,
             py::arg("useKinematicRangeLimits") = true,
             "Get minimum joint positions");

    // Bind ISolutionValidator (base class interface)
    py::class_<gps::kin::ISolutionValidator, std::shared_ptr<gps::kin::ISolutionValidator>>(m, "ISolutionValidator");

    // Bind GpsIKSolutionValidator
    py::class_<gps::kin::GpsIKSolutionValidator, 
               gps::kin::ISolutionValidator,
               std::shared_ptr<gps::kin::GpsIKSolutionValidator>>(m, "GpsIKSolutionValidator")
        .def(py::init<std::shared_ptr<gps::mctrl::kin::ForwardKinematicsGps>,
                      std::shared_ptr<gps::mctrl::kin::AxisConfigGroup>>(),
             py::arg("fwd_kinematics"),
             py::arg("axis_configs"),
             "Create a GPS IK solution validator")
        .def("validateSolution", &gps::kin::GpsIKSolutionValidator::validateSolution,
             py::arg("solution"),
             py::arg("robotTool_X_eeTip"),
             py::arg("target_robotBase_T_eeTip"),
             py::arg("tolerance") = 0.01)
        .def("isWithinJointLimits", &gps::kin::GpsIKSolutionValidator::isWithinJointLimits,
             py::arg("gpsJoints"),
             py::arg("useKinematicRangeLimits") = true);

    // Bind InverseKinematicsGps
    py::class_<gps::kin::InverseKinematicsGps>(m, "InverseKinematicsGps")
        .def(py::init<std::shared_ptr<gps::mctrl::kin::ForwardKinematicsGps>>(),
             py::arg("fwd_kinematics"),
             "Create an inverse kinematics solver")
        .def("findSolutionQuick", 
             [](gps::kin::InverseKinematicsGps& self,
                const gps::mctrl::geom::Ray3d& target,
                const gps::mctrl::geom::Affine3d& robotTool,
                const gps::mctrl::geom::Joints& currentJoints,
                std::shared_ptr<gps::kin::ISolutionValidator> validator,
                bool wristMode,
                bool rightHand,
                double tolerance) {
                 gps::mctrl::geom::Joints result;
                 bool success = self.findSolutionQuick(
                     target, robotTool, currentJoints, result,
                     *validator, wristMode, rightHand, tolerance);
                 return py::make_tuple(success, result);
             },
             py::arg("target"),
             py::arg("robotTool"),
             py::arg("currentJoints"),
             py::arg("validator"),
             py::arg("wristMode") = false,
             py::arg("rightHand") = true,
             py::arg("tolerance") = 0.01,
             "Find IK solution quickly.\n\n"
             "Args:\n"
             "    target: Target ray (position and direction)\n"
             "    robotTool: Robot tool transform\n"
             "    currentJoints: Current joint configuration\n"
             "    validator: Solution validator\n"
             "    wristMode: Wrist mode flag\n"
             "    rightHand: Right-hand configuration\n"
             "    tolerance: Position tolerance in mm\n\n"
             "Returns:\n"
             "    tuple: (success: bool, result: Joints)");
}