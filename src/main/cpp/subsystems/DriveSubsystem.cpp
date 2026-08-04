// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include "cmath"
#include "units/angle.h"
#include <array>

DriveSubsystem::DriveSubsystem() {
  // Lead Motor configs comment these out for initial testing all are at slot 0
  m_FFmotorAleadcfg.kS(OperatorConstants::FFA_Ks);
  m_FFmotorAleadcfg.kV(OperatorConstants::FFA_Kv);
  m_FFmotorAleadcfg.kA(OperatorConstants::FFA_Ka);

  m_FFmotorBleadcfg.kS(OperatorConstants::FFB_Ks);
  m_FFmotorBleadcfg.kV(OperatorConstants::FFB_Kv);
  m_FFmotorBleadcfg.kA(OperatorConstants::FFB_Ka);

  m_FFmotorCleadcfg.kS(OperatorConstants::FFC_Ks);
  m_FFmotorCleadcfg.kV(OperatorConstants::FFC_Kv);
  m_FFmotorCleadcfg.kA(OperatorConstants::FFC_Ka);

  m_FBmotorAleadcfg.Pid(OperatorConstants::FBA_P,
                        OperatorConstants::FBA_I,
                        OperatorConstants::FBA_D);

  m_FBmotorBleadcfg.Pid(OperatorConstants::FBB_P,
                        OperatorConstants::FBB_I,
                        OperatorConstants::FBB_D);
  
  m_FBmotorCleadcfg.Pid(OperatorConstants::FBC_P,
                        OperatorConstants::FBC_I,
                        OperatorConstants::FBC_D);


  // Follow Motor configs
  m_motorAFollowcfg.Follow(m_motorALead, false);
  m_motorBFollowcfg.Follow(m_motorBLead, false);
  m_motorCFollowcfg.Follow(m_motorCLead, false);

  // Voltage Compensation mode
  m_motorALeadcfg.VoltageCompensation(OperatorConstants::nominalVoltage);
  m_motorBLeadcfg.VoltageCompensation(OperatorConstants::nominalVoltage);
  m_motorCLeadcfg.VoltageCompensation(OperatorConstants::nominalVoltage);
}


void DriveSubsystem::Drive(double target_x, double target_y, double target_z) {
  
  m_wheelSpeedVector = DriveSubsystem::InverseKinematics(target_x, target_y, target_z);
  m_wheelSpeedVector = NormalizedKinematics(m_wheelSpeedVector);
  
  m_motorALead.Set(m_wheelSpeedVector[0]);
  m_motorBLead.Set(m_wheelSpeedVector[1]);
  m_motorCLead.Set(m_wheelSpeedVector[2]);
}

std::array<double, 3> DriveSubsystem::InverseKinematics(double x, double y, double z) {
      
  return {x * std::cos(OperatorConstants::WheelATheta) + y * std::sin(OperatorConstants::WheelATheta) - z,
          x * std::cos(OperatorConstants::WheelBTheta) + y * std::sin(OperatorConstants::WheelBTheta) - z,
          x * std::cos(OperatorConstants::WheelCTheta) + y * std::sin(OperatorConstants::WheelCTheta) - z };

}

std::array<double, 3>DriveSubsystem::NormalizedKinematics(const std::array<double, 3>& vector) {
  
  auto maxIterator = std::max_element(vector.begin(), vector.end(), [](double a, double b) {
    return std::abs(a) < std:: abs(b);
  });
  double maxSpeed = std::abs(*maxIterator);
  
  if (maxSpeed > OperatorConstants::MaxValue) 
  return {vector[0] * ( OperatorConstants::MaxValue / maxSpeed ),
          vector[1] * ( OperatorConstants::MaxValue / maxSpeed ),
          vector[2] * ( OperatorConstants::MaxValue / maxSpeed )};

  return vector;

}


void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void DriveSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
