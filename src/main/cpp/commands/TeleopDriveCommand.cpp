// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/kinematics/ChassisSpeeds.h>
#include "commands/TeleopDriveCommand.h"
#include <units/velocity.h>

#include "cmath"
#include "units/math.h"


TeleopDriveCommand::TeleopDriveCommand(DriveSubsystem* drive, frc2::CommandXboxController* controller)
    : m_drive{drive}, m_controller{controller} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drive);
}

void TeleopDriveCommand::Execute(){
  
  Frame raw_xyrot { m_controller->GetLeftX(), m_controller->GetLeftY(), m_controller-> GetRightX()};
  // x and y are swapped with negatives to match the field coord system
  Frame robot_xyrot { -raw_xyrot.y, -raw_xyrot.x, raw_xyrot.rot};
  
  Polar pol_xyrot = PolarOut(robot_xyrot);
  pol_xyrot.magnitude = ApplyDeadband(pol_xyrot.magnitude, OperatorConstants::DeadbandValue);
  pol_xyrot.rot = ApplyDeadband(pol_xyrot.rot, OperatorConstants::DeadbandValue);
  pol_xyrot.magnitude = ApplyCurve(pol_xyrot.magnitude, OperatorConstants::DriveCurve);
  pol_xyrot.rot = ApplyCurve(pol_xyrot.rot, OperatorConstants::DriveCurve);
  frc::ChassisSpeeds m_commandSpeeds = CartOut(pol_xyrot);

  m_drive->Drive(m_commandSpeeds);

}

TeleopDriveCommand::Polar TeleopDriveCommand::PolarOut(const Frame& frame) {
    return {std::hypot(frame.x, frame.y), 
      units::radian_t{std::atan2(frame.y, frame.x)}, frame.rot};
}

frc::ChassisSpeeds TeleopDriveCommand::CartOut(const Polar& polar) {
    
  units::velocity::meters_per_second_t vx{units::math::cos(polar.angle) * polar.magnitude};
  units::velocity::meters_per_second_t vy{units::math::sin(polar.angle) * polar.magnitude};
  units::angular_velocity::radians_per_second_t omega{polar.rot};

  return { vx, vy, omega};

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