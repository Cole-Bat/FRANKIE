// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/TeleopDriveCommand.h"

RobotContainer::RobotContainer() {
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // might not need a toptr in default command
  
  m_drive.SetDefaultCommand(TeleopDriveCommand(&m_drive, &m_driverController));
  // Need to configure the pose reset button
  
  // Configure your trigger bindings here

  (m_driverController.LeftBumper() && m_driverController.Y())
      .WhileTrue(m_drive.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  
  (m_driverController.LeftBumper() && (m_driverController.B()))
      .WhileTrue(m_drive.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  
  (m_driverController.LeftBumper() && m_driverController.A())
      .WhileTrue(m_drive.SysIdDynamic(frc2::sysid::Direction::kForward));
  
  (m_driverController.LeftBumper() && m_driverController.X())
      .WhileTrue(m_drive.SysIdDynamic(frc2::sysid::Direction::kReverse));            

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return m_drive.Run([] {});
}
