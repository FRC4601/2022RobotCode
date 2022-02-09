// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//  Goals:








#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>

// CTRE Docs - https://docs.ctre-phoenix.com/en/stable/ch05a_CppJava.html
#include <ctre/Phoenix.h>

#include <iostream>
#include <memory>

// Limelight directives
// API - https://docs.limelightvision.io/en/latest/getting_started.html
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"


// Declare variables
TalonSRX shooter1 = {8}; // number refers to device id. Can be found in Tuner
TalonSRX shooter2 = {9};

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {

  // Motor controllers
  frc::PWMSparkMax m_leftMotor{0};
  frc::PWMSparkMax m_rightMotor{1};

  // Robot Drive
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

  // Joysticks
  frc::Joystick m_leftStick{0};
  frc::Joystick m_rightStick{1};

  // Limelight
  // std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight"); // why is NetworkTable undefined?
  // double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
  // double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
  // double targetArea = table->GetNumber("ta",0.0);
  // double targetSkew = table->GetNumber("ts",0.0);

 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.SetInverted(true);

    shooter1.Set(ControlMode::PercentOutput, 0);
  };

  void TeleopPeriodic() override {
    // Drive with tank style
    //m_robotDrive.TankDrive(m_leftStick.GetY(), m_rightStick.GetY());

    shooter1.Set(ControlMode::PercentOutput, 0.5);
    shooter2.Set(ControlMode::Follower, 5); // I'm unsure exactly how the 'Follower' control mode works. Needs testing
  }

  /*double EstimateDistance() {
    std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double ty = table->GetNumber("ty", 0.0);
    double h2 = 20; // Height of target = 98.25
    double h1 = 26.13; // Height of camera from floor = 26.13
    double a1 = 3;  // Yaw of camera = 0
    double d = (h2 - h1) / tan(a1 + ty);
    return d;
    //D Returns in inches
    //TODO: return in metric
  }

  double EstimateSpeed(double d) {
    double h = 1.8;
    double theta, v;
    theta = atan(2 * h / d);
    v = pow(2.0 * g * d / sin(2.0 * theta), 0.5);
    return v;
  }

  double EstimateAngle(double d) {
    double h = 1.8;
    double theta;
    theta = atan(2 * h/d);
    theta = 180 * theta / pi;
    return theta;
  }*/
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
