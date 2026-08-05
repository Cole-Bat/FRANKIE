// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include <rev/SparkMax.h>

#include "Constants.h"
#include <array>


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();
  
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  void ConfigureControllers();
  
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
  
  frc2::sysid::SysIdRoutine m_sysIdRoutine;
  
  std::array<double, 3> m_wheelSpeedVector; 
  std::array<double, 3> InverseKinematics(double x, double y, double z);
  std::array<double, 3> NormalizedKinematics(const std::array<double, 3>& vector);

  // Lead Motor Objects
  rev::spark::SparkMax m_motorALead;
  rev::spark::SparkMax m_motorBLead;
  rev::spark::SparkMax m_motorCLead;

  // Follow Motor Objects
  rev::spark::SparkMax m_motorAFollow;
  rev::spark::SparkMax m_motorBFollow;
  rev::spark::SparkMax m_motorCFollow;

  // Encoder Objects (must be placed below Motor since they are dependant)
  rev::spark::SparkRelativeEncoder m_wheelAEncoder;
  rev::spark::SparkRelativeEncoder m_wheelBEncoder;
  rev::spark::SparkRelativeEncoder m_wheelCEncoder;


  // also need an IMU sensor

  // potential limelight pose estimation

  // can also add the follower encoders for diagnosis purposes

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
