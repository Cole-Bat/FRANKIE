// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>

#include <rev/SparkMax.h>
#include <rev/sim/SparkMaxSim.h>

#include "units/time.h"

#include "Constants.h"
#include <array>


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();
  
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

  void ConfigureControllers();
  
  void Drive(const frc::ChassisSpeeds& speeds);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  
  void Periodic() override;
  // put something about the pose here
  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  // Pose for the SIM
  frc::Pose2d m_Pose2dSim;
  frc::Field2d m_field;

  nt::DoublePublisher m_aPubEncoder;
  nt::DoublePublisher m_bPubEncoder;
  nt::DoublePublisher m_cPubEncoder;
  nt::DoublePublisher m_aPubSim;
  nt::DoublePublisher m_bPubSim;
  nt::DoublePublisher m_cPubSim;

 private:
  
  frc::ChassisSpeeds m_driveSpeeds; 
  std::array<double, 3> m_wheelSpeedVector{}; 
  std::array<double, 3> InverseKinematics(const frc::ChassisSpeeds& driveSpeeds);
  std::array<double, 3> NormalizedKinematics(const std::array<double, 3>& vector);

  void KiwiPoseEstimator(const double va, const double vb, const double vc, const double radius);

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

  // Motor Physics Model
  frc::DCMotor m_neoMotors;

  // Motor Objects for Simulation
  rev::spark::SparkMaxSim m_motorASim;
  rev::spark::SparkMaxSim m_motorBSim;
  rev::spark::SparkMaxSim m_motorCSim;

  // System ID object
  frc2::sysid::SysIdRoutine m_sysIdRoutine;

  // also need an IMU sensor

  // potential limelight pose estimation

  // can also add the follower encoders for diagnosis purposes

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
