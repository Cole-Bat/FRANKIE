#pragma once

#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/angle.h>
#include <array>

namespace kn {

class KiwiKinematics {

public:

struct WheelDouble {double a; double b; double c; };
struct Polar {double magnitude; units::radian_t angle; double rot; };
struct Frame {double x; double y; double rot; };

enum Conversion {
    velocity = 0,
    position = 1
};

Polar PolarOut(const Frame& frame);

Frame CartOut(const Polar& polar);

double getCF(const units::length::meter_t wheel_diameter, const double gear_ratio, const Conversion cf_type);

frc::ChassisSpeeds convertPercentToSpeeds_kiwi(double joyX, double joyY, double joyOmega, double maxX, double maxY, double maxOmega);
// joy stick inputs converted as a percentage of maximum values

double maxRobotVelocityX_kiwi(double vCF, double max_motor_rpm);

double maxRobotVelocityY_kiwi(double vCF, double max_motor_rpm);

double maxRobotVelocityOmega_kiwi(double vCF, double max_motor_rpm, units::length::meter_t wheelPosition);

std::array<double, 3> inverseKinematics(const frc::ChassisSpeeds& driveSpeeds);

WheelDouble NormalizedKinematics(const std::array<double, 3>& vector, const double& max_velocity);

// Inverse kinematics function
// Normalized Kinematics function
// Forward Kinematics funciton
// Wheel speed kinematics - single, max x, max y, max rotation

//  Auto input takes position data and outputs velocities based off of pathplanning and motion profile
//  Inputs: Auto x y velocities and angular velocity (m/s, m/s, rads/s) , Teleop percent x y rot (%, %, %) - > (m/s, m/s, rads/s)
//  Drive:  Auto Drive (frc Chassis speed), Teleop Drive (frc Chassis Speed) 
//  kine:   Auto kiwi Kinematics ( frc Chassis Speed), Teleop Kiwi kinematics ( frc Chassis Speed)
//  kine 2: Auto Normalize kinematics values (array of doubles), Teleop Normalize Kinematic values (array of doubles)
//  Drive:  Auto Drive with set reference (Velocity), Teleop drive with set (m/s, m/s, m/s) - > (% Duty Cycle)

//  Should I normalize the speeds that can be requested to fit in field centric driving
//  should I make sure the the same kinematics and normalization is used for auto and teleop
//      requires conversions into correct units into the kinematics pipeline
//      requires conversions into the correct units when inputing to the motors
//      the chassisspeeds type is used as an output speed for motion profiling

//  should have a test routine that gets the lowest m/s of the wheels for driving maximum speeds

private:

frc::ChassisSpeeds forwardKinematics(std::array<double, 3>);

units::meters_per_second_t singleWheelKinematics(double radius, double gear_ratio, double max_rpm);

};

}