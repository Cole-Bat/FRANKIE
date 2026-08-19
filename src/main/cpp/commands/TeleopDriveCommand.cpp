// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/kinematics/ChassisSpeeds.h>
#include "commands/TeleopDriveCommand.h"
#include <util/Kinematics.h>
#include <units/velocity.h>

#include "cmath"
#include "units/math.h"


TeleopDriveCommand::TeleopDriveCommand(DriveSubsystem* drive, frc2::CommandXboxController* controller)
    : m_drive{drive}, m_controller{controller} {
  // Register that this command requires the subsystem.
  AddRequirements(m_drive);
}

void TeleopDriveCommand::Execute(){
  
  kn::KiwiKinematics::Frame raw_xyrot { m_controller->GetLeftX(), m_controller->GetLeftY(), m_controller-> GetRightX()};
  // x and y are swapped with negatives to match the field coord system
  kn::KiwiKinematics::Frame robot_xyrot { -raw_xyrot.y, -raw_xyrot.x, raw_xyrot.rot};
  
  kn::KiwiKinematics::Polar pol_xyrot = m_kinematics.PolarOut(robot_xyrot);
  pol_xyrot.magnitude = ApplyDeadband(pol_xyrot.magnitude, Constants::DeadbandValue);
  pol_xyrot.rot = ApplyDeadband(pol_xyrot.rot, Constants::DeadbandValue);
  pol_xyrot.magnitude = ApplyCurve(pol_xyrot.magnitude, Constants::DriveCurve);
  pol_xyrot.rot = ApplyCurve(pol_xyrot.rot, Constants::DriveCurve);
  kn::KiwiKinematics::Frame final_xyrot = m_kinematics.CartOut(pol_xyrot);

  m_drive->Drive(final_xyrot.x, final_xyrot.y, final_xyrot.rot);

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