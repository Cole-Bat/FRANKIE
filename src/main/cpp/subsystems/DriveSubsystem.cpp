// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <rev/config/SparkMaxConfig.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/DoubleTopic.h>

#include "cmath"
#include "units/angle.h"
#include "units/time.h"
#include <array>

DriveSubsystem::DriveSubsystem()  // Initialization area for private member variables
  : m_motorALead{ OperatorConstants::MotorALeadID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorBLead{ OperatorConstants::MotorBLeadID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorCLead{ OperatorConstants::MotorCLeadID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorAFollow{ OperatorConstants::MotorAFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorBFollow{ OperatorConstants::MotorBFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorCFollow{ OperatorConstants::MotorCFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_wheelAEncoder{ m_motorALead.GetEncoder()},
    m_wheelBEncoder{ m_motorBLead.GetEncoder()},
    m_wheelCEncoder{ m_motorCLead.GetEncoder()},
    m_neoMotors{frc::DCMotor::NEO(2)},
    m_motorASim{ &m_motorALead, &m_neoMotors},
    m_motorBSim{ &m_motorBLead, &m_neoMotors},
    m_motorCSim{ &m_motorCLead, &m_neoMotors},
    m_sysIdRoutine(
      frc2::sysid::Config {1_V / 1_s, 7_V, 10_s, nullptr},
      frc2::sysid::Mechanism {
        [this] (units::volt_t voltage) {
          m_motorALead.SetVoltage(voltage);
          m_motorBLead.SetVoltage(voltage * -0.5);
          m_motorCLead.SetVoltage(voltage * -0.5);
        }, 
        [this](frc::sysid::SysIdRoutineLog* log) {
          // bus voltage * applied output to get the voltage at the motor
          log->Motor("wheel-A") 
            .voltage(m_motorALead.GetBusVoltage() * units::volt_t{m_motorALead.GetAppliedOutput()}) 
            .position(units::meter_t{m_wheelAEncoder.GetPosition()})
            .velocity(units::meters_per_second_t{m_wheelAEncoder.GetVelocity()});
          log->Motor("wheel-B") 
            .voltage(m_motorBLead.GetBusVoltage() * units::volt_t{m_motorBLead.GetAppliedOutput()}) 
            .position(units::meter_t{m_wheelBEncoder.GetPosition()})
            .velocity(units::meters_per_second_t{m_wheelBEncoder.GetVelocity()});
          log->Motor("wheel-C") 
            .voltage(m_motorCLead.GetBusVoltage() * units::volt_t{m_motorCLead.GetAppliedOutput()}) 
            .position(units::meter_t{m_wheelCEncoder.GetPosition()})
            .velocity(units::meters_per_second_t{m_wheelCEncoder.GetVelocity()});  
        },
        this
      }
    )

{ //Constructor for Drivesystem public
  ConfigureControllers();
  frc::SmartDashboard::PutData("Field", &m_field);
  auto inst = nt::NetworkTableInstance::GetDefault();
  auto table = inst.GetTable("datatable");
  m_aPubEncoder = table->GetDoubleTopic("Encoder A").Publish();
  m_bPubEncoder = table->GetDoubleTopic("Encoder B").Publish();
  m_cPubEncoder = table->GetDoubleTopic("Encoder C").Publish();
  m_aPubSim = table->GetDoubleTopic("Encoder A Sim").Publish();
  m_bPubSim = table->GetDoubleTopic("Encoder B Sim").Publish();
  m_cPubSim = table->GetDoubleTopic("Encoder C Sim").Publish();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_aPubEncoder.Set(m_wheelAEncoder.GetVelocity());
  m_bPubEncoder.Set(m_wheelBEncoder.GetVelocity());
  m_cPubEncoder.Set(m_wheelCEncoder.GetVelocity());
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
  m_motorASim.SetVelocity(m_wheelSpeedVector[0] * 2500);
  m_motorBSim.SetVelocity(m_wheelSpeedVector[1] * 2500);
  m_motorCSim.SetVelocity(m_wheelSpeedVector[2] * 2500);
  
  m_aPubSim.Set(m_motorASim.GetVelocity());
  m_bPubSim.Set(m_motorBSim.GetVelocity());
  m_cPubSim.Set(m_motorCSim.GetVelocity());
  
  KiwiPoseEstimator(m_motorASim.GetVelocity(), m_motorBSim.GetVelocity(), m_motorCSim.GetVelocity(), OperatorConstants::WheelPosRadius);
  m_field.SetRobotPose(m_Pose2dSim);
}

void DriveSubsystem::Drive(double target_x, double target_y, double target_rot) {
  
  m_wheelSpeedVector = InverseKinematics(target_x, target_y, target_rot);
  m_wheelSpeedVector = NormalizedKinematics(m_wheelSpeedVector);
  
  m_motorALead.Set(m_wheelSpeedVector[0]);
  m_motorBLead.Set(m_wheelSpeedVector[1]);
  m_motorCLead.Set(m_wheelSpeedVector[2]);
}

std::array<double, 3> DriveSubsystem::InverseKinematics(double x, double y, double rot) {
      
  return {x * std::cos(std::numbers::pi * OperatorConstants::WheelATheta / 180) + 
          y * std::sin(std::numbers::pi * OperatorConstants::WheelATheta / 180) + rot,

          x * std::cos(std::numbers::pi * OperatorConstants::WheelBTheta / 180) + 
          y * std::sin(std::numbers::pi * OperatorConstants::WheelBTheta / 180) + rot,

          x * std::cos(std::numbers::pi * OperatorConstants::WheelCTheta / 180) + 
          y * std::sin(std::numbers::pi * OperatorConstants::WheelCTheta / 180) + rot };

}

std::array<double, 3>DriveSubsystem::NormalizedKinematics(const std::array<double, 3>& vector) {
  
  auto maxIterator = std::max_element(vector.begin(), vector.end(), [] (double a, double b) {
    return std::abs(a) < std:: abs(b);
  });
  double maxSpeed = std::abs(*maxIterator);
  
  if (maxSpeed > OperatorConstants::MaxValue) 
  return {vector[0] * ( OperatorConstants::MaxValue / maxSpeed ),
          vector[1] * ( OperatorConstants::MaxValue / maxSpeed ),
          vector[2] * ( OperatorConstants::MaxValue / maxSpeed )};

  return vector;

}

frc2::CommandPtr DriveSubsystem::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}
  
frc2::CommandPtr DriveSubsystem::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}

void DriveSubsystem::KiwiPoseEstimator(const double va, const double vb, const double vc, const double radius) {

  double pCF = (OperatorConstants::WheelDiaMeter * std::numbers::pi) / OperatorConstants::DriveGearRatio;
  double vCF = pCF / 60.0; //number of seconds in a minute for m/s
  
  double velocityX =  va * vCF * cos((std::numbers::pi * OperatorConstants::WheelATheta / 180) + m_Pose2dSim.Rotation().Radians().value()) +
                      vb * vCF * cos((std::numbers::pi * OperatorConstants::WheelBTheta / 180) + m_Pose2dSim.Rotation().Radians().value()) +
                      vc * vCF * cos((std::numbers::pi * OperatorConstants::WheelCTheta / 180) + m_Pose2dSim.Rotation().Radians().value());

  double velocityY =  va * vCF * sin((std::numbers::pi * OperatorConstants::WheelATheta / 180) + m_Pose2dSim.Rotation().Radians().value()) +
                      vb * vCF * sin((std::numbers::pi * OperatorConstants::WheelBTheta / 180) + m_Pose2dSim.Rotation().Radians().value()) +
                      vc * vCF * sin((std::numbers::pi * OperatorConstants::WheelCTheta / 180) + m_Pose2dSim.Rotation().Radians().value());  
 
  double velocityW = (va + vb + vc) * vCF / (3 * radius);

  units::length::meter_t dX = units::length::meter_t(velocityX * OperatorConstants::dtLoop);
  units::length::meter_t dY = units::length::meter_t(velocityY * OperatorConstants::dtLoop);
  frc::Rotation2d theta{units::angle::radian_t(velocityW * OperatorConstants::dtLoop)};

  m_Pose2dSim = m_Pose2dSim + frc::Transform2d{dX, dY, theta};

}

void DriveSubsystem::ConfigureControllers() {
  
  //Conversion Factors based on 8 inch wheels with 10.71:1 Gear Ratio
  double positionConversionFactor = (OperatorConstants::WheelDiaMeter * std::numbers::pi) / OperatorConstants::DriveGearRatio;
  double velocityConversionFactor = positionConversionFactor / 60.0; //number of seconds in a minute for m/s
  
  //everything configured in one object for wheel A
  rev::spark::SparkMaxConfig ALConfig;
    //Voltage compensation for when voltage dips
    ALConfig.VoltageCompensation(OperatorConstants::nominalVoltage);
    
    //FeedForward Controller Constants
    ALConfig.closedLoop.feedForward
        .kS(OperatorConstants::FFA_Ks)
        .kV(OperatorConstants::FFA_Kv)
        .kA(OperatorConstants::FFA_Ka);
    
    //Feedback Controller Constants    
    ALConfig.closedLoop.Pid(OperatorConstants::FBA_P, OperatorConstants::FBA_I, OperatorConstants::FBA_D);
    
    //Encoder Conversion Factors
    ALConfig.encoder
        .PositionConversionFactor(positionConversionFactor)
        .VelocityConversionFactor(velocityConversionFactor);
  
  rev::spark::SparkMaxConfig AFConfig;
    AFConfig.Follow(m_motorALead,false);

  m_motorALead.Configure(ALConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
  m_motorAFollow.Configure(AFConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);  

  //everything configured in one object for wheel B
  rev::spark::SparkMaxConfig BLConfig;
    //Voltage compensation for when voltage dips
    BLConfig.VoltageCompensation(OperatorConstants::nominalVoltage);
    
    //FeedForward Controller Constants
    BLConfig.closedLoop.feedForward
        .kS(OperatorConstants::FFB_Ks)
        .kV(OperatorConstants::FFB_Kv)
        .kA(OperatorConstants::FFB_Ka);
    
    //Feedback Controller Constants
    BLConfig.closedLoop.Pid(OperatorConstants::FBB_P, OperatorConstants::FBB_I, OperatorConstants::FBB_D);
    
    //Encoder Conversion Factors
    BLConfig.encoder
        .PositionConversionFactor(positionConversionFactor)
        .VelocityConversionFactor(velocityConversionFactor);
  
  rev::spark::SparkMaxConfig BFConfig;
    BFConfig.Follow(m_motorBLead,false);

  m_motorBLead.Configure(BLConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
  m_motorBFollow.Configure(BFConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);  

  //everything configured in one object for wheel C
  rev::spark::SparkMaxConfig CLConfig;
    //Voltage compensation for when voltage dips
    CLConfig.VoltageCompensation(OperatorConstants::nominalVoltage);
    
    //FeedForward Controller Constants
    CLConfig.closedLoop.feedForward
        .kS(OperatorConstants::FFC_Ks)
        .kV(OperatorConstants::FFC_Kv)
        .kA(OperatorConstants::FFC_Ka);
    
    //Feedback Controller Constants
    CLConfig.closedLoop.Pid(OperatorConstants::FBC_P, OperatorConstants::FBC_I, OperatorConstants::FBC_D);
    
    //Encoder Conversion Factors
    CLConfig.encoder
        .PositionConversionFactor(positionConversionFactor)
        .VelocityConversionFactor(velocityConversionFactor);
  
  rev::spark::SparkMaxConfig CFConfig;
    CFConfig.Follow(m_motorCLead,false);
  
  m_motorCLead.Configure(CLConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
  m_motorCFollow.Configure(CFConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);  

}
