#include <util/Kinematics.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/length.h>

#include <Constants.h>
#include <math.h>

kn::KiwiKinematics::Polar kn::KiwiKinematics::PolarOut(const Frame& frame) {
  return {std::hypot(frame.x, frame.y), 
          units::radian_t{std::atan2(frame.y, frame.x)}, 
          frame.rot};
}

kn::KiwiKinematics::Frame kn::KiwiKinematics::CartOut(const Polar& polar) {
  return  {units::math::cos(polar.angle) * polar.magnitude,
          units::math::sin(polar.angle) * polar.magnitude,
          polar.rot}; 

}

double kn::KiwiKinematics::getCF(const units::length::meter_t wheel_diameter, const double gear_ratio, const kn::KiwiKinematics::Conversion cf){
    if (cf == 0)
        return (double(wheel_diameter * std::numbers::pi) / (gear_ratio * 60));
    else
        return (double(wheel_diameter * std::numbers::pi) / gear_ratio);
}

frc::ChassisSpeeds kn::KiwiKinematics::convertPercentToSpeeds_kiwi(double joyX, double joyY, double joyOmega, double maxX, double maxY, double maxOmega){
    //joy stick inputs converted as a percentage of maximum values

    return {units::velocity::meters_per_second_t{joyX * maxX}, 
            units::velocity::meters_per_second_t{joyY * maxY}, 
            units::angular_velocity::radians_per_second_t{ joyOmega * maxOmega}};
}

std::array<double, 3> kn::KiwiKinematics::inverseKinematics(const frc::ChassisSpeeds& driveSpeeds){

    constexpr double degToRad = std::numbers::pi / 180;

    return {double(driveSpeeds.vx * std::cos(Constants::WheelATheta * degToRad) + 
                 driveSpeeds.vy * std::sin(Constants::WheelATheta * degToRad) + 
                units::velocity::meters_per_second_t{(driveSpeeds.omega.value() * Constants::driveWheelPosition.value())}),

          double(driveSpeeds.vx * std::cos(Constants::WheelBTheta * degToRad) + 
                 driveSpeeds.vy * std::sin(Constants::WheelBTheta * degToRad) + 
                units::velocity::meters_per_second_t{(driveSpeeds.omega.value() * Constants::driveWheelPosition.value())}),
          
          double(driveSpeeds.vx * std::cos(Constants::WheelCTheta * degToRad) + 
                 driveSpeeds.vy * std::sin(Constants::WheelCTheta * degToRad) + 
                units::velocity::meters_per_second_t{(driveSpeeds.omega.value() * Constants::driveWheelPosition.value())})
  };

}

kn::KiwiKinematics::WheelDouble kn::KiwiKinematics::NormalizedKinematics(const std::array<double, 3>& vector, const double& max_velocity){

     auto maxIterator = std::max_element(vector.begin(), vector.end(), [] (double a, double b) {
    return std::abs(a) < std:: abs(b);
    });
    double maxSpeed = std::abs(*maxIterator);
    
    if (maxSpeed > max_velocity) 
    return {vector[0] * ( max_velocity / maxSpeed ),
            vector[1] * ( max_velocity / maxSpeed ),
            vector[2] * ( max_velocity / maxSpeed )};

    return {vector[0], vector[1], vector[2]};

}

frc::ChassisSpeeds forwardKinematics(std::array<double, 3>){
// need to create, for sims
}

double kn::KiwiKinematics::maxRobotVelocityX_kiwi(double vCF, double max_motor_rpm){
    return (vCF * max_motor_rpm * cos(std::numbers::pi / 6));
}

double kn::KiwiKinematics::maxRobotVelocityY_kiwi(double vCF, double max_motor_rpm){
    return (vCF * max_motor_rpm);
}

double kn::KiwiKinematics::maxRobotVelocityOmega_kiwi(double vCF, double max_motor_rpm, units::length::meter_t wheelPosition){
    return (vCF * max_motor_rpm / double(wheelPosition)); 
}