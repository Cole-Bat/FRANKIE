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
  
  Cart raw_xy { m_controller->GetLeftX(), m_controller->GetLeftY()};

  Polar pol_xy = PolarOut(raw_xy);
  pol_xy.magnitude = ApplyDeadband(pol_xy.magnitude, OperatorConstants::DeadbandValue);
  pol_xy.magnitude = ApplyCurve(pol_xy.magnitude, OperatorConstants::DriveCurve);
  Cart final_xy = CartOut(pol_xy);
  double final_z = ApplyCurve(m_controller-> GetRightX(), OperatorConstants::RotCurve);

  m_drive->Drive(final_xy.x, final_xy.y, final_z);

}

TeleopDriveCommand::Polar TeleopDriveCommand::PolarOut(const Cart& cart) {
    return {std::hypot(cart.x, cart.y), 
      units::radian_t{std::atan2(cart.y, cart.x)}};
}

TeleopDriveCommand::Cart TeleopDriveCommand::CartOut(const Polar& polar) {
    return {units::math::cos(polar.angle) * polar.magnitude,
      units::math::sin(polar.angle) * polar.magnitude};
}

double TeleopDriveCommand::ApplyDeadband(double mag, double deadband) {
    if (mag < deadband) return 0.0;
    return (mag - deadband) / (1.0 - deadband);
}

double TeleopDriveCommand::ApplyCurve(double mag, double curve) {
    return std::pow(mag, curve);
}

bool TeleopDriveCommand::IsFinished() {
  return false;
}