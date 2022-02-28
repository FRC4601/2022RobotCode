// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//  Goals:

/*Changelog

*/





#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/PWMVictorSPX.h>

// CTRE Docs - https://docs.ctre-phoenix.com/en/stable/ch05a_CppJava.html
#include <ctre/Phoenix.h>

#include <iostream>
#include <memory>
#include <string.h>

// Limelight directives
// API - https://docs.limelightvision.io/en/latest/getting_started.html
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"

// Rev directives
// API - https://codedocs.revrobotics.com/cpp/namespacerev.html -/- Relative Encoder Class https://codedocs.revrobotics.com/cpp/classrev_1_1_relative_encoder.html
#include <rev/CANSparkMax.h>
#include <rev/RelativeEncoder.h>  // useful doc ~ https://github.com/REVrobotics/SPARK-MAX-Examples/issues/15

//FRC Pathplanner

//Path



class Robot : public frc::TimedRobot {

  // Initialization
  bool shooterArmPosition = false;  // false - up ~ true - down


  // Talon
  TalonFX shooter1 = {1}; // number refers to device id. Can be found in Tuner
  TalonFX shooter2 = {2};

  // Motor controllers
  frc::PWMVictorSPX m_leftMotor{1};
  frc::PWMVictorSPX m_rightMotor{2};
  frc::PWMVictorSPX armMotor{3};
  frc::PWMVictorSPX ballsuckingMotor{4};

  // Encoders
  static constexpr int kCanID = 1;
  static constexpr auto kMotorType = rev::CANSparkMax::MotorType::kBrushless;
  static constexpr auto kAltEncType = rev::CANEncoder::AlternateEncoderType::kQuadrature;
  static constexpr int kCPR = 8192;

  rev::CANSparkMax m_motor{kCanID, kMotorType};
  rev::SparkMaxAlternateEncoder m_alternateEncoder = m_motor.GetAlternateEncoder(kAltEncType, kCPR);
  rev::SparkMaxPIDController m_pidController = m_motor.GetPIDController();

  // PID coefficients
  double kP = 0.1, kI = 1e-4, kD = 1, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
  
  // Robot Drive
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

  // Joysticks
  frc::Joystick m_leftStick{0};
  frc::Joystick m_rightStick{1};


 public:
  void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotor.SetInverted(true);
    shooter2.SetInverted(true);
    
  };

  void TeleopPeriodic() override {
    // Drive with tank style
    m_robotDrive.TankDrive(m_leftStick.GetY(), m_rightStick.GetY());


    // Shooter encoder rotation control
    m_motor.RestoreFactoryDefaults();
    m_pidController.SetFeedbackDevice(m_alternateEncoder);

    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // use SetPosition to make shooter up/down positioning a toggle
    if (m_leftStick.GetRawButton(2)) {
      // if arm is down move up
      if(shooterArmPosition){
        m_alternateEncoder.SetPosition(-1); // These values need testing
      }
      else {
        m_alternateEncoder.SetPosition(1);
      }
      shooterArmPosition = !shooterArmPosition;
    }


      
    // Shooter control
    if (m_leftStick.GetRawButton(1)){
      shooter1.Set(ControlMode::PercentOutput, 0.5);
      //shooter2.Set(ControlMode::Follower, 5); // I'm unsure exactly how the 'Follower' control mode works. Needs testing
      shooter2.Set(ControlMode::PercentOutput, 0.5);
    }
    else if (m_rightStick.GetRawButton(1)){
      shooter1.Set(ControlMode::PercentOutput, 0.25);
      //shooter2.Set(ControlMode::Follower, 5); // I'm unsure exactly how the 'Follower' control mode works. Needs testing
      shooter2.Set(ControlMode::PercentOutput, 0.25);
    }
    else{
      shooter1.Set(ControlMode::PercentOutput, 0.0);
      shooter2.Set(ControlMode::PercentOutput, 0.0);
    }

    // TESTING LIMELIGHT ALIGN CODE
    bool hasAligned = true;
    if (m_rightStick.GetRawButton(8)){
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",0);
      double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      double ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      double tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);

      if (!hasAligned) {

        // Rotational Tracking
        if (!(tx < 6 && tx > -6)){
          tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
          if (tx > 6){
            tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
            m_robotDrive.TankDrive(-0.75,0.75);
          }
          else if (tx < -6){
            tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
            m_robotDrive.TankDrive(0.75,-0.75);
          }
        }
        else if (!(tx < 3 && tx > -3)){
          tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
          if (tx > 3){
            tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
            m_robotDrive.TankDrive(-0.55,0.55);
          }
          else if (tx < -3){
            tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
            m_robotDrive.TankDrive(0.5,-0.55);
          }
        }
        else {
          tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
          m_robotDrive.TankDrive(0,0);
          hasAligned = true;
        }
      }

      // Distance Tracking
      else {
        
        double desiredDistance = 15; // Most likely in feet? needs testings
        double currentDistance = EstimateDistance();

        if (desiredDistance > currentDistance){
          m_robotDrive.TankDrive(0.55,0.55);  // Backup
        }
        else if (desiredDistance < currentDistance){
          m_robotDrive.TankDrive(-0.55,-0.55);  // Move Forward
        }
        else {
          m_robotDrive.TankDrive(1,1);
        }

        std::string s = std::to_string(currentDistance);
        frc::SmartDashboard::PutString("DB/String 0", s);
        

      }


    }
    else {
      // Force limelight LED off
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1);
    }



        
      
  }

  double EstimateDistance() {
    double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
    double h2 = 55; // Height of target = 98.25
    double h1 = 9; // Height of camera from floor = 26.13
    double a1 = 10;  // Yaw of camera = 0

    // Based off limelight docs
    double angleToGoalDegrees = a1 + ty;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double d = (h2 - h1)/ tan(angleToGoalDegrees);

    //double d = (h2 - h1) / tan(a1 + ty);
    return d;
    //D Returns in inches
    //TODO: return in metric (is it really necessary to?)
  }

  /*
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
