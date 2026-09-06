#pragma once

// Имена топиков внутренней шины — единственное место, где они объявлены
// . В сервисах — только через topics::k*.
namespace project
{
namespace topics
{

// Platform -> шина: сырые сенсоры
inline constexpr const char* kSensorsLaser = "sensors/laser";        // types::LaserScan
inline constexpr const char* kSensorsOdometry = "sensors/odometry";  // types::Odometry
inline constexpr const char* kSensorsImu = "sensors/imu";            // types::Imu
inline constexpr const char* kSensorsBumper = "sensors/bumper";      // types::Bumper
inline constexpr const char* kSensorsIr = "sensors/ir";              // types::IrData
inline constexpr const char* kSensorsSonar = "sensors/sonar";        // types::SonarData
inline constexpr const char* kRobotState = "robot/state";            // types::RobotState

// Slam
inline constexpr const char* kSlamPose = "slam/pose";  // types::Pose2D
inline constexpr const char* kSlamMap = "slam/map";    // types::OccupancyMap

// Mission (цикл уборки)
inline constexpr const char* kMissionCommand = "mission/command";  // types::MissionCommand
inline constexpr const char* kMissionState = "mission/state";      // types::MissionState

// Planner
inline constexpr const char* kPlannerGoal = "planner/goal";              // types::Goal (Mission и внешняя цель)
inline constexpr const char* kPlannerTrajectory = "planner/trajectory";  // types::Trajectory
inline constexpr const char* kPlannerState = "planner/state";            // types::PlannerState

// Control -> Platform (и телеоперация из ZmqBridge в обход планировщика)
inline constexpr const char* kCmdVel = "controls/cmd_vel";        // types::CmdVel
inline constexpr const char* kActuators = "controls/actuators";  // types::ActuatorCommand

// Safety -> Control
inline constexpr const char* kSafetyStop = "safety/stop";  // types::SafetyStop

}  // namespace topics
}  // namespace project
