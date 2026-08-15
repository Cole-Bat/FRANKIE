// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/angular_velocity.h>
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

// CONTROLLER CONSTANTS
inline constexpr int kDriverControllerPort = 0;

// JOYSTICK SIGNAL PROCESSING
inline constexpr double DeadbandValue = 0.04;
inline constexpr double DriveCurve = 2.0;
inline constexpr double RotCurve = 3.0;
inline constexpr double velScale = 0.8;
inline constexpr double omegaScale = 0.5; // must be between 0.0 and 1.0

// CAN ID CONSTANTS
inline constexpr int MotorALeadID = 15;
inline constexpr int MotorAFollowID = 12;
inline constexpr int MotorBLeadID = 11;
inline constexpr int MotorBFollowID = 10;
inline constexpr int MotorCLeadID = 13;
inline constexpr int MotorCFollowID = 14;

// PLANT INVERSION CONSTANTS
inline constexpr double FFA_Ks = 0.40068; 
inline constexpr double FFA_Kv = 1.9736;
inline constexpr double FFA_Ka = 0.1967;

inline constexpr double FFB_Ks = 0.26755;
inline constexpr double FFB_Kv = 1.5894;
inline constexpr double FFB_Ka = 0.35959;

inline constexpr double FFC_Ks = 0.22346;
inline constexpr double FFC_Kv = 1.814;
inline constexpr double FFC_Ka = 0.354;

// PID CONSTANTS
inline constexpr double FBA_P = 0.51; //0.51
inline constexpr double FBA_I = 0.0;
inline constexpr double FBA_D = 0.0;

inline constexpr double FBB_P = 0.89898; // 0.89898
inline constexpr double FBB_I = 0.0;
inline constexpr double FBB_D = 0.0;

inline constexpr double FBC_P = 0.98847; //0.98847
inline constexpr double FBC_I = 0.0;
inline constexpr double FBC_D = 0.0;

// MECHANICAL CONSTANTS
inline constexpr double WheelATheta = 90.0;
inline constexpr double WheelBTheta = 210.0;
inline constexpr double WheelCTheta = 330.0;
inline constexpr double WheelDiaMeter = 0.2032; // 8inch wheel in meters
inline constexpr double DriveGearRatio = 10.71; // 10.71:1 toughbox mini
inline constexpr double WheelPosRadius = 0.25; // in meters to the center also equal for all 3 wheels
inline constexpr units::velocity::meters_per_second_t maxRobotVelocity{1.6154}; // (m/s) 
inline constexpr units::velocity::meters_per_second_t maxRobotVelocityX{1.3989}; // (m/s)
inline constexpr units::velocity::meters_per_second_t maxRobotVelocityY{1.6154}; // (m/s)
inline constexpr units::angular_velocity::radians_per_second_t maxRobotVelocityOmega{6.4614}; // (rad/s)


// ELECTRICAL CONSTANTS
inline constexpr double nominalVoltage = 12.0;
inline constexpr double dtLoop = 0.020; //seconds that the periodic function is called

}  // namespace OperatorConstants
