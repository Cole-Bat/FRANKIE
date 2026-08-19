// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
#include <util/Kinematics.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Twist2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
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
  : m_motorALead{ Constants::MotorALeadID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorBLead{ Constants::MotorBLeadID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorCLead{ Constants::MotorCLeadID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorAFollow{ Constants::MotorAFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorBFollow{ Constants::MotorBFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless},
    m_motorCFollow{ Constants::MotorCFollowID, rev::spark::SparkLowLevel::MotorType::kBrushless},
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
  ConfigureParameters();
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
  m_motorASim.SetVelocity(m_wheelSpeeds.a);
  m_motorBSim.SetVelocity(m_wheelSpeeds.b);
  m_motorCSim.SetVelocity(m_wheelSpeeds.c);
  
  m_aPubSim.Set(m_motorASim.GetVelocity());
  m_bPubSim.Set(m_motorBSim.GetVelocity());
  m_cPubSim.Set(m_motorCSim.GetVelocity());
  
  //KiwiPoseEstimator(m_motorASim.GetVelocity(), m_motorBSim.GetVelocity(), m_motorCSim.GetVelocity(), double(Constants::driveWheelPosition));
  
  units::length::meter_t dX = m_driveSpeeds.vx * Constants::dtLoop;
  units::length::meter_t dY = m_driveSpeeds.vy * Constants::dtLoop;
  units::angle::radian_t dtheta = -m_driveSpeeds.omega * Constants::dtLoop;

  m_Twist2dsim = {dX, dY, dtheta};
  m_Pose2dSim = m_Pose2dSim.Exp(m_Twist2dsim);

  m_field.SetRobotPose(m_Pose2dSim);
}

void DriveSubsystem::Drive(const double vx, const double vy, const double rot) {

  // might want a teleop drive and an auto drive
  // need controller objects and set reference for the auto drive
  //  inital runtime characteristic variables set

  m_driveSpeeds = m_kinematics.convertPercentToSpeeds_kiwi(vx, vy, rot, m_maxRobotVelocityX, m_maxRobotVelocityY, m_maxRobotVelocityOmega);
  
  m_wheelSpeedArray = m_kinematics.inverseKinematics(m_driveSpeeds);
  m_wheelSpeeds = m_kinematics.NormalizedKinematics(m_wheelSpeedArray, m_maxRobotVelocity);
  
  //Set requires Duty Cycle values
  m_motorALead.Set(m_wheelSpeeds.a / m_maxRobotVelocity);
  m_motorBLead.Set(m_wheelSpeeds.b / m_maxRobotVelocity);
  m_motorCLead.Set(m_wheelSpeeds.c / m_maxRobotVelocity);
}

void DriveSubsystem::autoDrive(const frc::ChassisSpeeds& autoDriveSpeeds) {

  
  m_wheelSpeedArray = m_kinematics.inverseKinematics(autoDriveSpeeds);
  m_wheelSpeeds = m_kinematics.NormalizedKinematics(m_wheelSpeedArray, m_maxRobotVelocity);
  
  //Set requires Duty Cycle values
  m_motorALead.Set(m_wheelSpeeds.a / m_maxRobotVelocity);
  m_motorBLead.Set(m_wheelSpeeds.b / m_maxRobotVelocity);
  m_motorCLead.Set(m_wheelSpeeds.c / m_maxRobotVelocity);

}

frc2::CommandPtr DriveSubsystem::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}
  
frc2::CommandPtr DriveSubsystem::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}

void DriveSubsystem::KiwiPoseEstimator(const double va, const double vb, const double vc, const double radius) {


  double degToRad = std::numbers::pi / 180;
  units::length::meter_t radius_m{radius};
  units::time::second_t dt{Constants::dtLoop};

  units::velocity::meters_per_second_t velocityA{va};
  units::velocity::meters_per_second_t velocityB{vb};
  units::velocity::meters_per_second_t velocityC{vc};
  units::velocity::meters_per_second_t velocityRot = (velocityA + velocityB + velocityC) / 3.0;

  units::velocity::meters_per_second_t velocityVectorX = 
        (velocityA - velocityRot) * cos(Constants::WheelATheta * degToRad) +
        (velocityB - velocityRot) * cos(Constants::WheelBTheta * degToRad) +
        (velocityC - velocityRot) * cos(Constants::WheelCTheta * degToRad);
  
  units::velocity::meters_per_second_t velocityVectorY = 
        (velocityA - velocityRot) * sin(Constants::WheelATheta * degToRad) +
        (velocityB - velocityRot) * sin(Constants::WheelBTheta * degToRad) +
        (velocityC - velocityRot) * sin(Constants::WheelCTheta * degToRad);
 
  units::angular_velocity::radians_per_second_t velocityW{-(velocityRot / radius_m).value()};



}

void DriveSubsystem::ConfigureParameters(){

//  need to get max rpm under load

  m_positionConversionFactor = m_kinematics.getCF(Constants::driveWheelDiameter, Constants::DriveGearRatio, kn::KiwiKinematics::position);

  m_velocityConversionFactor = m_kinematics.getCF(Constants::driveWheelDiameter, Constants::DriveGearRatio, kn::KiwiKinematics::velocity);

  m_maxRobotVelocityX = m_kinematics.maxRobotVelocityX_kiwi( m_velocityConversionFactor, Constants::max_rpm);
  
  m_maxRobotVelocityY = m_kinematics.maxRobotVelocityY_kiwi( m_velocityConversionFactor, Constants::max_rpm);         

  m_maxRobotVelocityOmega = m_kinematics.maxRobotVelocityOmega_kiwi( m_velocityConversionFactor, Constants::max_rpm, Constants::driveWheelPosition);
  
  m_maxRobotVelocity = m_maxRobotVelocityY;

}

void DriveSubsystem::ConfigureControllers() {
  
  //everything configured in one object for wheel A
  rev::spark::SparkMaxConfig ALConfig;
    //Voltage compensation for when voltage dips
    ALConfig.VoltageCompensation(Constants::nominalVoltage);
    
    //FeedForward Controller Constants
    ALConfig.closedLoop.feedForward
        .kS(Constants::FFA_Ks)
        .kV(Constants::FFA_Kv)
        .kA(Constants::FFA_Ka);
    
    //Feedback Controller Constants    
    ALConfig.closedLoop.Pid(Constants::FBA_P, Constants::FBA_I, Constants::FBA_D);
    
    //Encoder Conversion Factors
    ALConfig.encoder
        .PositionConversionFactor(m_positionConversionFactor)
        .VelocityConversionFactor(m_velocityConversionFactor);
  
  rev::spark::SparkMaxConfig AFConfig;
    AFConfig.Follow(m_motorALead,false);

  m_motorALead.Configure(ALConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
  m_motorAFollow.Configure(AFConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);  

  //everything configured in one object for wheel B
  rev::spark::SparkMaxConfig BLConfig;
    //Voltage compensation for when voltage dips
    BLConfig.VoltageCompensation(Constants::nominalVoltage);
    
    //FeedForward Controller Constants
    BLConfig.closedLoop.feedForward
        .kS(Constants::FFB_Ks)
        .kV(Constants::FFB_Kv)
        .kA(Constants::FFB_Ka);
    
    //Feedback Controller Constants
    BLConfig.closedLoop.Pid(Constants::FBB_P, Constants::FBB_I, Constants::FBB_D);
    
    //Encoder Conversion Factors
    BLConfig.encoder
        .PositionConversionFactor(m_positionConversionFactor)
        .VelocityConversionFactor(m_velocityConversionFactor);
  
  rev::spark::SparkMaxConfig BFConfig;
    BFConfig.Follow(m_motorBLead,false);

  m_motorBLead.Configure(BLConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);
  m_motorBFollow.Configure(BFConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kPersistParameters);  

  //everything configured in one object for wheel C
  rev::spark::SparkMaxConfig CLConfig;
    //Voltage compensation for when voltage dips
    CLConfig.VoltageCompensation(Constants::nominalVoltage);
    
    //FeedForward Controller Constants
    CLConfig.closedLoop.feedForward
        .kS(Constants::FFC_Ks)
        .kV(Constants::FFC_Kv)
        .kA(Constants::FFC_Ka);
    
    //Feedback Controller Constants
    CLConfig.closedLoop.Pid(Constants::FBC_P, Constants::FBC_I, Constants::FBC_D);
    
    //Encoder Conversion Factors
    CLConfig.encoder
        .PositionConversionFactor(m_positionConversionFactor)
        .VelocityConversionFactor(m_velocityConversionFactor);
  
  rev::spark::SparkMaxConfig CFConfig;
    CFConfig.Follow(m_motorCLead,false);
  
  m_motorCLead.Configure(CLConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
  m_motorCFollow.Configure(CFConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);  

}
