// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include "Constants.h"
#include <array>


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();
  
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
  
  std::array<double, 3> m_wheelSpeedVector; 
  std::array<double, 3> InverseKinematics(double x, double y, double z);
  std::array<double, 3> NormalizedKinematics(const std::array<double, 3>& vector);

  // Lead Motor Objects
  rev::spark::SparkMax m_motorALead{ OperatorConstants::MotorALeadID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::FeedForwardConfig m_FFmotorAleadcfg;
  rev::spark::ClosedLoopConfig m_FBmotorAleadcfg;
  rev::spark::SparkBaseConfig m_motorALeadcfg;

  rev::spark::SparkMax m_motorBLead{ OperatorConstants::MotorBLeadID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::FeedForwardConfig m_FFmotorBleadcfg;
  rev::spark::ClosedLoopConfig m_FBmotorBleadcfg;
  rev::spark::SparkBaseConfig m_motorBLeadcfg;
  
  rev::spark::SparkMax m_motorCLead{ OperatorConstants::MotorCLeadID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::FeedForwardConfig m_FFmotorCleadcfg;
  rev::spark::ClosedLoopConfig m_FBmotorCleadcfg;
  rev::spark::SparkBaseConfig m_motorCLeadcfg;
  
  // Follow Motor Objects
  rev::spark::SparkMax m_motorAFollow{ OperatorConstants::MotorAFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_motorAFollowcfg;
  
  rev::spark::SparkMax m_motorBFollow{ OperatorConstants::MotorBFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_motorBFollowcfg;

  rev::spark::SparkMax m_motorCFollow{ OperatorConstants::MotorCFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkBaseConfig m_motorCFollowcfg;

  // Encoder Objects
  rev::spark::SparkRelativeEncoder wheelAEncoder{ m_motorALead.GetEncoder()};
  rev::spark::SparkRelativeEncoder wheelBEncoder{ m_motorBLead.GetEncoder()};
  rev::spark::SparkRelativeEncoder wheelCEncoder{ m_motorCLead.GetEncoder()};

  // also need an IMU sensor

  // potential limelight pose estimation

  // can also add the follower encoders for diagnosis purposes

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
