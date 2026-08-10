// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TeleopDriveCommand.h"

#include "cmath"
#include "units/math.h"


TeleopDriveCommand::TeleopDriveCommand(DriveSubsystem* drive, frc2::CommandXboxController* controller)
    : m_drive{drive}, m_controller{controller} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drive);
}

void TeleopDriveCommand::Execute(){
  
  Frame raw_xyz { m_controller->GetLeftX(), m_controller->GetLeftY(), m_controller-> GetRightX()};
  Frame robot_xyz { -raw_xyz.y, -raw_xyz.x, -raw_xyz.z};
  
  Polar pol_xyz = PolarOut(robot_xyz);
  pol_xyz.magnitude = ApplyDeadband(pol_xyz.magnitude, OperatorConstants::DeadbandValue);
  pol_xyz.magnitude = ApplyDeadband(pol_xyz.z, OperatorConstants::DeadbandValue);
  pol_xyz.magnitude = ApplyCurve(pol_xyz.magnitude, OperatorConstants::DriveCurve);
  pol_xyz.magnitude = ApplyCurve(pol_xyz.z, OperatorConstants::DriveCurve);
  Frame final_xyz = CartOut(pol_xyz);

  m_drive->Drive(final_xyz.x, final_xyz.y, final_xyz.z);

  //m_drive->Drive(robot_xyz.x, robot_xyz.y, robot_xyz.z);
  printf("%f \n %f \n %f \n", raw_xyz.x, raw_xyz.y, raw_xyz.z);
  printf("%f \n %f \n %f \n", robot_xyz.x, robot_xyz.y, robot_xyz.z);

}

TeleopDriveCommand::Polar TeleopDriveCommand::PolarOut(const Frame& frame) {
    return {std::hypot(frame.x, frame.y), 
      units::radian_t{std::atan2(frame.y, frame.x)}, frame.z};
}

TeleopDriveCommand::Frame TeleopDriveCommand::CartOut(const Polar& polar) {
    return {units::math::cos(polar.angle) * polar.magnitude,
      units::math::sin(polar.angle) * polar.magnitude, polar.z};
}

double TeleopDriveCommand::ApplyDeadband(double mag, double deadband) {
    if (std::abs(mag) < deadband) return 0.0;
    return std::copysign((std::abs(mag) - deadband) / (1.0 - deadband), mag);
}

double TeleopDriveCommand::ApplyCurve(double mag, double curve) {
    return std::copysign(std::pow(std::abs(mag), curve), mag);
}

bool TeleopDriveCommand::IsFinished() {
  return false;
}