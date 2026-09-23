#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "H1_Robot.hpp"
#include "HighLevelControl.hpp"

namespace py = pybind11;

namespace {

template <typename T, std::size_t N>
std::vector<T> array_to_vector(const T (&values)[N]) {
    return std::vector<T>(values, values + N);
}

template <typename T, std::size_t N>
void vector_to_array(T (&target)[N], const std::vector<T>& values, const char* field_name) {
    if (values.size() != N) {
        throw std::invalid_argument(std::string(field_name) + " must contain exactly " + std::to_string(N) + " values");
    }
    for (std::size_t i = 0; i < N; ++i) {
        target[i] = values[i];
    }
}

template <typename Enum, typename... Args>
bool call_bool(H1Robot& robot, bool (H1Robot::*method)(Enum, const Motor_Control&), Enum id, const Motor_Control& control) {
    py::gil_scoped_release release;
    return (robot.*method)(id, control);
}

template <typename State, typename Enum>
py::tuple get_motor_like_state(H1Robot& robot, bool (H1Robot::*method)(Enum, State&), Enum id) {
    State state{};
    bool ok = false;
    {
        py::gil_scoped_release release;
        ok = (robot.*method)(id, state);
    }
    return py::make_tuple(ok, state);
}

template <typename State>
py::tuple get_state(H1Robot& robot, bool (H1Robot::*method)(State&)) {
    State state{};
    bool ok = false;
    {
        py::gil_scoped_release release;
        ok = (robot.*method)(state);
    }
    return py::make_tuple(ok, state);
}

}  // namespace

PYBIND11_MODULE(lib_h1_sdk_python, m) {
    m.doc() = "Python 3.12 bindings for Zerith H1 SDK 1.3.7 robot_SDK";

    py::enum_<InitCommand>(m, "InitCommand")
        .value("Wait_Init", Wait_Init)
        .value("Init", Init)
        .value("Deinit", Deinit)
        .value("ReInit", ReInit)
        .export_values();

    py::enum_<InitState>(m, "InitState")
        .value("Uninit", Uninit)
        .value("Initializing", Initializing)
        .value("Init_Complete", Init_Complete)
        .value("Deinitializing", Deinitializing)
        .value("Deinit_Complete", Deinit_Complete)
        .value("Error_State", Error_State)
        .export_values();

    py::enum_<ArmAction>(m, "ArmAction")
        .value("LEFT_ARM", LEFT_ARM)
        .value("RIGHT_ARM", RIGHT_ARM)
        .export_values();

    py::enum_<MotorControlMode>(m, "MotorControlMode")
        .value("UNINITIALIZED", UNINITIALIZED)
        .value("LOW_LEVEL", LOW_LEVEL)
        .value("HIGH_LEVEL", HIGH_LEVEL)
        .value("GRAVITY_COMPENSATION_LEVEL", GRAVITY_COMPENSATION_LEVEL)
        .export_values();

    py::enum_<EtherCAT_Motor_Index>(m, "EtherCAT_Motor_Index")
        .value("MOTOR_WHEEL_LEFT", MOTOR_WHEEL_LEFT)
        .value("MOTOR_WHEEL_RIGHT", MOTOR_WHEEL_RIGHT)
        .value("MOTOR_LIFT", MOTOR_LIFT)
        .value("MOTOR_WAIST_DOWN", MOTOR_WAIST_DOWN)
        .value("MOTOR_WAIST_UP", MOTOR_WAIST_UP)
        .value("MOTOR_HEAD_DOWN", MOTOR_HEAD_DOWN)
        .value("MOTOR_HEAD_UP", MOTOR_HEAD_UP)
        .value("MOTOR_LEFT_ARM_1", MOTOR_LEFT_ARM_1)
        .value("MOTOR_LEFT_ARM_2", MOTOR_LEFT_ARM_2)
        .value("MOTOR_LEFT_ARM_3", MOTOR_LEFT_ARM_3)
        .value("MOTOR_LEFT_ARM_4", MOTOR_LEFT_ARM_4)
        .value("MOTOR_LEFT_ARM_5", MOTOR_LEFT_ARM_5)
        .value("MOTOR_LEFT_ARM_6", MOTOR_LEFT_ARM_6)
        .value("MOTOR_LEFT_ARM_7", MOTOR_LEFT_ARM_7)
        .value("MOTOR_LEFT_ARM_8", MOTOR_LEFT_ARM_8)
        .value("MOTOR_RIGHT_ARM_1", MOTOR_RIGHT_ARM_1)
        .value("MOTOR_RIGHT_ARM_2", MOTOR_RIGHT_ARM_2)
        .value("MOTOR_RIGHT_ARM_3", MOTOR_RIGHT_ARM_3)
        .value("MOTOR_RIGHT_ARM_4", MOTOR_RIGHT_ARM_4)
        .value("MOTOR_RIGHT_ARM_5", MOTOR_RIGHT_ARM_5)
        .value("MOTOR_RIGHT_ARM_6", MOTOR_RIGHT_ARM_6)
        .value("MOTOR_RIGHT_ARM_7", MOTOR_RIGHT_ARM_7)
        .value("MOTOR_RIGHT_ARM_8", MOTOR_RIGHT_ARM_8)
        .value("MOTOR_COUNT", MOTOR_COUNT)
        .export_values();

    py::class_<Motor_Information>(m, "Motor_Information")
        .def(py::init([] {
            Motor_Information value{};
            return value;
        }))
        .def_property("Position_Actual", [](const Motor_Information& s) { return s.Position_Actual; }, [](Motor_Information& s, float v) { s.Position_Actual = v; })
        .def_property("Speed_Actual", [](const Motor_Information& s) { return s.Speed_Actual; }, [](Motor_Information& s, float v) { s.Speed_Actual = v; })
        .def_property("Torque_Actual", [](const Motor_Information& s) { return s.Torque_Actual; }, [](Motor_Information& s, float v) { s.Torque_Actual = v; })
        .def_property("KP_Actual", [](const Motor_Information& s) { return s.KP_Actual; }, [](Motor_Information& s, float v) { s.KP_Actual = v; })
        .def_property("KD_Actual", [](const Motor_Information& s) { return s.KD_Actual; }, [](Motor_Information& s, float v) { s.KD_Actual = v; })
        .def_property("Error_flag", [](const Motor_Information& s) { return s.Error_flag; }, [](Motor_Information& s, uint16_t v) { s.Error_flag = v; });

    py::class_<Motor_Control>(m, "Motor_Control")
        .def(py::init<>())
        .def_property("Position", [](const Motor_Control& s) { return s.Position; }, [](Motor_Control& s, float v) { s.Position = v; })
        .def_property("Speed", [](const Motor_Control& s) { return s.Speed; }, [](Motor_Control& s, float v) { s.Speed = v; })
        .def_property("Torque", [](const Motor_Control& s) { return s.Torque; }, [](Motor_Control& s, float v) { s.Torque = v; })
        .def_property("KP", [](const Motor_Control& s) { return s.KP; }, [](Motor_Control& s, float v) { s.KP = v; })
        .def_property("KD", [](const Motor_Control& s) { return s.KD; }, [](Motor_Control& s, float v) { s.KD = v; });

    py::class_<ArmPose>(m, "ArmPose")
        .def(py::init([] {
            ArmPose value{};
            return value;
        }))
        .def_readwrite("x", &ArmPose::x)
        .def_readwrite("y", &ArmPose::y)
        .def_readwrite("z", &ArmPose::z)
        .def_readwrite("roll", &ArmPose::roll)
        .def_readwrite("pitch", &ArmPose::pitch)
        .def_readwrite("yaw", &ArmPose::yaw);

    py::class_<ArmEndPose>(m, "ArmEndPose")
        .def(py::init([] {
            ArmEndPose value{};
            return value;
        }))
        .def_property("position", [](const ArmEndPose& s) { return array_to_vector(s.PositionData); }, [](ArmEndPose& s, const std::vector<float>& values) { vector_to_array(s.PositionData, values, "position"); })
        .def_property("rotation", [](const ArmEndPose& s) { return array_to_vector(s.RotationData); }, [](ArmEndPose& s, const std::vector<float>& values) { vector_to_array(s.RotationData, values, "rotation"); })
        .def_property("PositionData", [](const ArmEndPose& s) { return array_to_vector(s.PositionData); }, [](ArmEndPose& s, const std::vector<float>& values) { vector_to_array(s.PositionData, values, "PositionData"); })
        .def_property("RotationData", [](const ArmEndPose& s) { return array_to_vector(s.RotationData); }, [](ArmEndPose& s, const std::vector<float>& values) { vector_to_array(s.RotationData, values, "RotationData"); });

    py::class_<HandControl>(m, "HandControl")
        .def(py::init([] {
            HandControl value{};
            return value;
        }))
        .def_readwrite("hand_mode", &HandControl::hand_mode)
        .def_property("hand_control", [](const HandControl& s) { return array_to_vector(s.hand_control); }, [](HandControl& s, const std::vector<int32_t>& values) { vector_to_array(s.hand_control, values, "hand_control"); })
        .def_property("hand_speed", [](const HandControl& s) { return array_to_vector(s.hand_speed); }, [](HandControl& s, const std::vector<int32_t>& values) { vector_to_array(s.hand_speed, values, "hand_speed"); })
        .def_readwrite("hand_type", &HandControl::hand_type);

    py::class_<HandState>(m, "HandState")
        .def(py::init([] {
            HandState value{};
            return value;
        }))
        .def_property("hand_position", [](const HandState& s) { return array_to_vector(s.hand_position); }, [](HandState& s, const std::vector<int32_t>& values) { vector_to_array(s.hand_position, values, "hand_position"); })
        .def_property("hand_angle", [](const HandState& s) { return array_to_vector(s.hand_angle); }, [](HandState& s, const std::vector<int32_t>& values) { vector_to_array(s.hand_angle, values, "hand_angle"); })
        .def_property("hand_current", [](const HandState& s) { return array_to_vector(s.hand_current); }, [](HandState& s, const std::vector<int32_t>& values) { vector_to_array(s.hand_current, values, "hand_current"); })
        .def_property("hand_status", [](const HandState& s) { return array_to_vector(s.hand_status); }, [](HandState& s, const std::vector<int32_t>& values) { vector_to_array(s.hand_status, values, "hand_status"); })
        .def_readwrite("hand_type", &HandState::hand_type)
        .def_readwrite("error_flag", &HandState::error_flag);

    py::class_<ForceSensorState>(m, "ForceSensorState")
        .def(py::init([] {
            ForceSensorState value{};
            return value;
        }))
        .def_property("Fx", [](const ForceSensorState& s) { return array_to_vector(s.Fx); }, [](ForceSensorState& s, const std::vector<float>& values) { vector_to_array(s.Fx, values, "Fx"); })
        .def_property("Fy", [](const ForceSensorState& s) { return array_to_vector(s.Fy); }, [](ForceSensorState& s, const std::vector<float>& values) { vector_to_array(s.Fy, values, "Fy"); })
        .def_property("Fz", [](const ForceSensorState& s) { return array_to_vector(s.Fz); }, [](ForceSensorState& s, const std::vector<float>& values) { vector_to_array(s.Fz, values, "Fz"); })
        .def_property("Tx", [](const ForceSensorState& s) { return array_to_vector(s.Tx); }, [](ForceSensorState& s, const std::vector<float>& values) { vector_to_array(s.Tx, values, "Tx"); })
        .def_property("Ty", [](const ForceSensorState& s) { return array_to_vector(s.Ty); }, [](ForceSensorState& s, const std::vector<float>& values) { vector_to_array(s.Ty, values, "Ty"); })
        .def_property("Tz", [](const ForceSensorState& s) { return array_to_vector(s.Tz); }, [](ForceSensorState& s, const std::vector<float>& values) { vector_to_array(s.Tz, values, "Tz"); })
        .def_property("Error_flag", [](const ForceSensorState& s) { return array_to_vector(s.Error_flag); }, [](ForceSensorState& s, const std::vector<int32_t>& values) { vector_to_array(s.Error_flag, values, "Error_flag"); });

    py::class_<RobotInfo>(m, "RobotInfo")
        .def(py::init<>())
        .def_readwrite("robot_name", &RobotInfo::robot_name)
        .def_readwrite("robot_type", &RobotInfo::robot_type)
        .def_readwrite("firmware_version", &RobotInfo::firmware_version)
        .def_readwrite("hardware_version", &RobotInfo::hardware_version)
        .def_readwrite("software_version", &RobotInfo::software_version)
        .def_readwrite("manufacturer", &RobotInfo::manufacturer);

    py::class_<power_charge_state> power_charge(m, "power_charge_state");
    py::enum_<power_charge_state::ChargeStatus>(power_charge, "ChargeStatus")
        .value("UNKNOWN", power_charge_state::UNKNOWN)
        .value("CHARGING", power_charge_state::CHARGING)
        .value("DISCHARGING", power_charge_state::DISCHARGING)
        .value("FULL", power_charge_state::FULL)
        .export_values();
    power_charge
        .def(py::init<>())
        .def_readwrite("status", &power_charge_state::status)
        .def_readwrite("temperature", &power_charge_state::temperature)
        .def_readwrite("soc", &power_charge_state::soc);

    py::class_<ImuData>(m, "ImuData")
        .def(py::init([] {
            ImuData value{};
            return value;
        }))
        .def_property("rpy", [](const ImuData& s) { return array_to_vector(s.rpy); }, [](ImuData& s, const std::vector<float>& values) { vector_to_array(s.rpy, values, "rpy"); })
        .def_property("omega", [](const ImuData& s) { return array_to_vector(s.omega); }, [](ImuData& s, const std::vector<float>& values) { vector_to_array(s.omega, values, "omega"); })
        .def_readwrite("error_flag", &ImuData::error_flag);

    py::class_<XboxMap>(m, "XboxMap")
        .def(py::init([] {
            XboxMap value{};
            return value;
        }))
        .def_property("a", [](const XboxMap& s) { return static_cast<int>(s.a); }, [](XboxMap& s, int v) { s.a = static_cast<int8_t>(v); })
        .def_property("b", [](const XboxMap& s) { return static_cast<int>(s.b); }, [](XboxMap& s, int v) { s.b = static_cast<int8_t>(v); })
        .def_property("x", [](const XboxMap& s) { return static_cast<int>(s.x); }, [](XboxMap& s, int v) { s.x = static_cast<int8_t>(v); })
        .def_property("y", [](const XboxMap& s) { return static_cast<int>(s.y); }, [](XboxMap& s, int v) { s.y = static_cast<int8_t>(v); })
        .def_property("leftShoulder", [](const XboxMap& s) { return static_cast<int>(s.leftShoulder); }, [](XboxMap& s, int v) { s.leftShoulder = static_cast<int8_t>(v); })
        .def_property("rightShoulder", [](const XboxMap& s) { return static_cast<int>(s.rightShoulder); }, [](XboxMap& s, int v) { s.rightShoulder = static_cast<int8_t>(v); })
        .def_property("leftStick", [](const XboxMap& s) { return static_cast<int>(s.leftStick); }, [](XboxMap& s, int v) { s.leftStick = static_cast<int8_t>(v); })
        .def_property("rightStick", [](const XboxMap& s) { return static_cast<int>(s.rightStick); }, [](XboxMap& s, int v) { s.rightStick = static_cast<int8_t>(v); })
        .def_property("start", [](const XboxMap& s) { return static_cast<int>(s.start); }, [](XboxMap& s, int v) { s.start = static_cast<int8_t>(v); })
        .def_property("back", [](const XboxMap& s) { return static_cast<int>(s.back); }, [](XboxMap& s, int v) { s.back = static_cast<int8_t>(v); })
        .def_property("dpadUp", [](const XboxMap& s) { return static_cast<int>(s.dpadUp); }, [](XboxMap& s, int v) { s.dpadUp = static_cast<int8_t>(v); })
        .def_property("dpadDown", [](const XboxMap& s) { return static_cast<int>(s.dpadDown); }, [](XboxMap& s, int v) { s.dpadDown = static_cast<int8_t>(v); })
        .def_property("dpadLeft", [](const XboxMap& s) { return static_cast<int>(s.dpadLeft); }, [](XboxMap& s, int v) { s.dpadLeft = static_cast<int8_t>(v); })
        .def_property("dpadRight", [](const XboxMap& s) { return static_cast<int>(s.dpadRight); }, [](XboxMap& s, int v) { s.dpadRight = static_cast<int8_t>(v); })
        .def_readwrite("leftX", &XboxMap::leftX)
        .def_readwrite("leftY", &XboxMap::leftY)
        .def_readwrite("rightX", &XboxMap::rightX)
        .def_readwrite("rightY", &XboxMap::rightY)
        .def_readwrite("leftTrigger", &XboxMap::leftTrigger)
        .def_readwrite("rightTrigger", &XboxMap::rightTrigger)
        .def_readwrite("error_flag", &XboxMap::error_flag);

    py::class_<HighLevelControl>(m, "HighLevelControl")
        .def(py::init([] {
            HighLevelControl value{};
            return value;
        }))
        .def_readwrite("control_type", &HighLevelControl::control_type)
        .def_readwrite("control_part", &HighLevelControl::control_part)
        .def_property("waist_position", [](const HighLevelControl& s) { return array_to_vector(s.waist_position); }, [](HighLevelControl& s, const std::vector<float>& values) { vector_to_array(s.waist_position, values, "waist_position"); })
        .def_property("hand_pose", [](const HighLevelControl& s) { return array_to_vector(s.hand_pose); }, [](HighLevelControl& s, const std::vector<float>& values) { vector_to_array(s.hand_pose, values, "hand_pose"); })
        .def_readwrite("eef_velocity", &HighLevelControl::eef_velocity)
        .def_readwrite("eef_acceleration", &HighLevelControl::eef_acceleration)
        .def_property("block", [](const HighLevelControl& s) { return s.block != 0; }, [](HighLevelControl& s, bool v) { s.block = static_cast<int8_t>(v); })
        .def_readwrite("duration", &HighLevelControl::duration);

    py::class_<HighLevelState>(m, "HighLevelState")
        .def(py::init([] {
            HighLevelState value{};
            return value;
        }))
        .def_readwrite("state", &HighLevelState::state)
        .def_readwrite("progress", &HighLevelState::progress);

    py::class_<H1Robot>(m, "H1Robot")
        .def(py::init<>())
        .def(py::init<const std::string&>(), py::arg("udp_url"))
        .def("robot_connect", [](H1Robot& self) {
            py::gil_scoped_release release;
            return self.robot_connect();
        })
        .def("switchControlMode", [](H1Robot& self, MotorControlMode mode) {
            py::gil_scoped_release release;
            return self.switchControlMode(mode);
        }, py::arg("new_mode"))
        .def("robot_init", [](H1Robot& self) {
            py::gil_scoped_release release;
            return self.robot_init();
        })
        .def("robot_deinit", [](H1Robot& self) {
            py::gil_scoped_release release;
            return self.robot_deinit();
        })
        .def("setMotorControl_low", [](H1Robot& self, EtherCAT_Motor_Index motor_id, const Motor_Control& control) {
            return call_bool(self, &H1Robot::setMotorControl_low, motor_id, control);
        }, py::arg("motor_id"), py::arg("control"))
        .def("setChassis_low", [](H1Robot& self, EtherCAT_Motor_Index chassis_id, const Motor_Control& control) {
            return call_bool(self, &H1Robot::setChassis_low, chassis_id, control);
        }, py::arg("chassis_id"), py::arg("control"))
        .def("setWaist_low", [](H1Robot& self, EtherCAT_Motor_Index waist_id, const Motor_Control& control) {
            return call_bool(self, &H1Robot::setWaist_low, waist_id, control);
        }, py::arg("waist_id"), py::arg("control"))
        .def("setHead_low", [](H1Robot& self, EtherCAT_Motor_Index head_id, const Motor_Control& control) {
            return call_bool(self, &H1Robot::setHead_low, head_id, control);
        }, py::arg("head_id"), py::arg("control"))
        .def("setArm_low", [](H1Robot& self, EtherCAT_Motor_Index arm_id, const Motor_Control& control) {
            return call_bool(self, &H1Robot::setArm_low, arm_id, control);
        }, py::arg("arm_id"), py::arg("control"))
        .def("setGripper_low", [](H1Robot& self, EtherCAT_Motor_Index gripper_id, const Motor_Control& control, bool is_hold_torque) {
            py::gil_scoped_release release;
            return self.setGripper_low(gripper_id, control, is_hold_torque);
        }, py::arg("gripper_id"), py::arg("control"), py::arg("is_hold_torque") = true)
        .def("setHand_low", [](H1Robot& self, ArmAction arm, const HandControl& control) {
            py::gil_scoped_release release;
            return self.setHand_low(arm, control);
        }, py::arg("arm"), py::arg("control"))
        .def("setFixedRod_low", [](H1Robot& self, int is_open) {
            py::gil_scoped_release release;
            return self.setFixedRod_low(is_open);
        }, py::arg("is_open"))
        .def("resetForceSensorData", [](H1Robot& self, int sensor_id) {
            py::gil_scoped_release release;
            return self.resetForceSensorData(sensor_id);
        }, py::arg("sensor_id"))
        .def("setChassis_high", [](H1Robot& self, float speed_x, float speed_y) {
            py::gil_scoped_release release;
            return self.setChassis_high(speed_x, speed_y);
        }, py::arg("speed_x"), py::arg("speed_y"))
        .def("setWaist_high", [](H1Robot& self, const ArmEndPose& action) {
            py::gil_scoped_release release;
            return self.setWaist_high(action);
        }, py::arg("action"))
        .def("setHead_high", [](H1Robot& self, EtherCAT_Motor_Index head_id, const Motor_Control& control) {
            py::gil_scoped_release release;
            return self.setHead_high(head_id, control);
        }, py::arg("head_id"), py::arg("control"))
        .def("setArm_high", [](H1Robot& self, ArmAction arm, const ArmEndPose& action) {
            py::gil_scoped_release release;
            return self.setArm_high(arm, action);
        }, py::arg("arm"), py::arg("action"))
        .def("setArmMove_high", [](H1Robot& self, ArmAction arm, const ArmEndPose& action, float eef_velocity, float eef_acceleration, float duration, bool block) {
            py::gil_scoped_release release;
            return self.setArmMove_high(arm, action, eef_velocity, eef_acceleration, duration, block);
        }, py::arg("arm"), py::arg("action"), py::arg("eef_velocity") = 0.0f, py::arg("eef_acceleration") = 0.0f, py::arg("duration") = 0.0f, py::arg("block") = false)
        .def("setGripper_high", [](H1Robot& self, EtherCAT_Motor_Index gripper_id, const Motor_Control& control, bool is_hold_torque) {
            py::gil_scoped_release release;
            return self.setGripper_high(gripper_id, control, is_hold_torque);
        }, py::arg("gripper_id"), py::arg("control"), py::arg("is_hold_torque") = true)
        .def("setHand_high", [](H1Robot& self, ArmAction arm, const HandControl& control) {
            py::gil_scoped_release release;
            return self.setHand_high(arm, control);
        }, py::arg("arm"), py::arg("control"))
        .def("setFixedRod_high", [](H1Robot& self, int is_open) {
            py::gil_scoped_release release;
            return self.setFixedRod_high(is_open);
        }, py::arg("is_open"))
        .def("armPoseToArmEndPose", [](H1Robot& self, const ArmPose& pose) {
            py::gil_scoped_release release;
            return self.armPoseToArmEndPose(pose);
        }, py::arg("pose"))
        .def("getRobotInfo", [](H1Robot& self) { return get_state(self, &H1Robot::getRobotInfo); })
        .def("getMotorState", [](H1Robot& self, EtherCAT_Motor_Index motor_id) { return get_motor_like_state(self, &H1Robot::getMotorState, motor_id); }, py::arg("motor_id"))
        .def("getChassisState", [](H1Robot& self, EtherCAT_Motor_Index chassis_id) { return get_motor_like_state(self, &H1Robot::getChassisState, chassis_id); }, py::arg("chassis_id"))
        .def("getChassisSpeedState", [](H1Robot& self) {
            std::array<float, 2> speed_actual{};
            std::array<float, 2> speed_algo{};
            bool ok = false;
            {
                py::gil_scoped_release release;
                ok = self.getChassisSpeedState(speed_actual, speed_algo);
            }
            return py::make_tuple(ok, std::vector<float>(speed_actual.begin(), speed_actual.end()), std::vector<float>(speed_algo.begin(), speed_algo.end()));
        })
        .def("getWaistState", [](H1Robot& self, EtherCAT_Motor_Index waist_id) { return get_motor_like_state(self, &H1Robot::getWaistState, waist_id); }, py::arg("waist_id"))
        .def("getHeadState", [](H1Robot& self, EtherCAT_Motor_Index head_id) { return get_motor_like_state(self, &H1Robot::getHeadState, head_id); }, py::arg("head_id"))
        .def("getArmState", [](H1Robot& self, EtherCAT_Motor_Index joint_id) { return get_motor_like_state(self, &H1Robot::getArmState, joint_id); }, py::arg("joint_id"))
        .def("getGripperState", [](H1Robot& self, EtherCAT_Motor_Index gripper_id) { return get_motor_like_state(self, &H1Robot::getGripperState, gripper_id); }, py::arg("gripper_id"))
        .def("getGripperControlMode", [](H1Robot& self) {
            int control_mode = 0;
            bool ok = false;
            {
                py::gil_scoped_release release;
                ok = self.getGripperControlMode(control_mode);
            }
            return py::make_tuple(ok, control_mode);
        })
        .def("getHandRelative", [](H1Robot& self, ArmAction arm) {
            ArmEndPose state{};
            bool ok = false;
            {
                py::gil_scoped_release release;
                ok = self.getHandRelative(arm, state);
            }
            return py::make_tuple(ok, state);
        }, py::arg("arm"))
        .def("getHeadRelative", [](H1Robot& self) { return get_state(self, &H1Robot::getHeadRelative); })
        .def("getHandCameraRelative", [](H1Robot& self, ArmAction arm) {
            ArmEndPose state{};
            bool ok = false;
            {
                py::gil_scoped_release release;
                ok = self.getHandCameraRelative(arm, state);
            }
            return py::make_tuple(ok, state);
        }, py::arg("arm"))
        .def("getHeadCameraRelative", [](H1Robot& self) { return get_state(self, &H1Robot::getHeadCameraRelative); })
        .def("getHandState", [](H1Robot& self, ArmAction arm) {
            HandState state{};
            bool ok = false;
            {
                py::gil_scoped_release release;
                ok = self.getHandState(arm, state);
            }
            return py::make_tuple(ok, state);
        }, py::arg("arm"))
        .def("getFixedRodState", [](H1Robot& self) {
            int is_open = 0;
            bool ok = false;
            {
                py::gil_scoped_release release;
                ok = self.getFixedRodState(is_open);
            }
            return py::make_tuple(ok, is_open);
        })
        .def("getHighLevelState", [](H1Robot& self) { return get_state(self, &H1Robot::getHighLevelState); })
        .def("getJoystickState", [](H1Robot& self) { return get_state(self, &H1Robot::getJoystickState); })
        .def("getIMU_State", [](H1Robot& self) { return get_state(self, &H1Robot::getIMU_State); })
        .def("getForceSensorState", [](H1Robot& self) { return get_state(self, &H1Robot::getForceSensorState); })
        .def("getPowerChargeState", [](H1Robot& self) { return get_state(self, &H1Robot::getPowerChargeState); })
        .def("getCurrentMode", [](H1Robot& self) {
            py::gil_scoped_release release;
            return self.getCurrentMode();
        })
        .def("getInitState", [](H1Robot& self) {
            py::gil_scoped_release release;
            return self.getInitState();
        })
        .def("isRobotConnected", [](H1Robot& self) {
            py::gil_scoped_release release;
            return self.isRobotConnected();
        });
}
