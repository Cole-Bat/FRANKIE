// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandXboxController.h>

#include "subsystems/DriveSubsystem.h"
#include "Constants.h"
#include "units/angle.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TeleopDriveCommand
    : public frc2::CommandHelper<frc2::Command, TeleopDriveCommand> {
 public:
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  explicit TeleopDriveCommand(DriveSubsystem* drive, frc2::CommandXboxController* controller);
  
  void Execute() override;
  bool IsFinished() override;

 private:
  DriveSubsystem* m_drive;
  frc2::CommandXboxController* m_controller;
  struct Polar {double magnitude; units::radian_t angle; };
  struct Cart {double x; double y; };

  double ApplyDeadband(double mag, double deadband);
  double ApplyCurve(double mag, double curve);
  Polar PolarOut(const Cart& cart);
  Cart CartOut(const Polar& polar);

};
