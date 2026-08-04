// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  void Drive(double x, double y, double z);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  
  // requires the declarations of the various drivetrain algorithims
  
   void Periodic() override;
  // put something about the pose here
  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:
  
 
 // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
