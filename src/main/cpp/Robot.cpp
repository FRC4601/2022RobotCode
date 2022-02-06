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
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/span.h>

// Declare variables
TalonSRX shooter1 = {0}; // number refers to device id. Can be found in Tuner
TalonSRX shooter2 = {1};

/**
 * This is a demo program showing the use of the DifferentialDrive class.
 * Runs the motors with tank steering.
 */
class Robot : public frc::TimedRobot {
  frc::PWMSparkMax m_leftMotor{0};
  frc::PWMSparkMax m_rightMotor{1};
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};
  frc::Joystick m_leftStick{0};
  frc::Joystick m_rightStick{1};

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

    shooter1.Set(ControlMode::PercentOutput, m_leftStick.GetY());
    shooter2.Set(ControlMode::Follower, 5); // I'm unsure exactly how the 'Follower' control mode works. Needs testing
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
