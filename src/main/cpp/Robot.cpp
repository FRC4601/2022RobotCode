// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/PWMVictorSPX.h>

// CTRE Docs - https://docs.ctre-phoenix.com/en/stable/ch05a_CppJava.html
#include <ctre/Phoenix.h>

#include <iostream>
#include <memory>
#include <string.h>
#include <cstdlib>

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

#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>

//FRC Pathplanner
#include <pathplanner/lib/PathPlanner.h>


class Robot : public frc::TimedRobot {

  #pragma region // Initialization

  // Initialization
  bool shooterArmPosition = false;  // false - up ~ true - down
  bool driveCodeToggle = true;  // true - tank // false - arcade // (currently true - arcade forward //false - arcade backwards)

  double arcadeY, arcadeX;

  // Color Sensor
  static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
  rev::ColorSensorV3 m_colorSensor{i2cPort};
  rev::ColorMatch m_colorMatcher;

  // Color Targets (values need calibrated)
  static constexpr frc::Color kBlueTarget = frc::Color(0.146, 0.375, 0.478);
  static constexpr frc::Color kRedTarget = frc::Color(0.579, 0.315, 0.107);  

  // Talon
  TalonFX shooter1 = {1}; // number refers to device id. Can be found in Tuner
  TalonFX shooter2 = {2};

  // Motor controllers
  frc::PWMVictorSPX m_leftMotor{1};
  frc::PWMVictorSPX m_rightMotor{2};
  frc::PWMVictorSPX armMotor{3};
  frc::PWMVictorSPX intakeMotor{4};
  frc::PWMVictorSPX stagingMotor{5};

  // Robot Drive
  frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

  // Joysticks
  frc::Joystick m_leftStick{0};
  frc::Joystick m_rightStick{1};
  frc::XboxController xboxController{2};

  int maxSpeed = 0;

  //Pathplanner

  //timer
  frc::Timer m_timer; 
  
  #pragma endregion


 public:
  void RobotInit() override {
    // Motor inverts
    m_leftMotor.SetInverted(true);
   shooter2.SetInverted(true);

    // Color Match Targets
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);

    //shooter PID control
    /*
    shooter1.ConfigFactoryDefault();
   shooter2.ConfigFactoryDefault();
    shooter2.Follow(shooter1);
    shooter1.SetInverted(TalonFXInvertType::Clockwise);
    shooter2.SetInverted(TalonFXInvertType::OpposeMaster);
    shooter1.ConfigNominalOutputForward(0);
    shooter1.ConfigNominalOutputReverse(0);
    shooter1.ConfigPeakOutputForward(1);
    shooter1.ConfigPeakOutputReverse(-1);
    shooter1.Config_kF(0,0.0801); //(values need changed for kf, kp, etc.)
    shooter1.Config_kP(0, 0.0);
    shooter1.Config_kI(0,0.0);
    shooter1.Config_kD(0,0.0);
    */
  };

  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
  };
  
  void AutonomousPeriodic() override {
      /*
    if (m_timer.Get() < 2_s)
    {
      armMotor.Set(0.25);
    }
    else if (m_timer.Get() < 4_s)
    {
      m_robotDrive.ArcadeDrive(0.3, 0.0);
      intakeMotor.Set (-0.75);
    }
    else if (m_timer.Get() > 4_s && m_timer.Get() < 6_s)
    {
      m_robotDrive.ArcadeDrive (0.0, 0.5);
      intakeMotor.Set(0.0);
    }
    else if (m_timer.Get() > 6_s && m_timer.Get() < 8_s)
    {
    shooter1.Set(ControlMode::PercentOutput, 0.47);
    shooter2.Set(ControlMode::PercentOutput, 0.47);
    }
    else if (m_timer.Get() > 8_s && m_timer.Get() < 10_s)
    {
    armMotor.Set(-0.3);
    intakeMotor.Set(-0.3);
    stagingMotor.Set(0.5);
    }
    else
    {
    shooter1.Set(ControlMode::PercentOutput, 0.0);
    shooter2.Set(ControlMode::PercentOutput, 0.0);
    armMotor.Set(0.0);
    intakeMotor.Set(0.0);
    stagingMotor.Set(0.0);
    }
*/
  };

  void TeleopInit() override {

  };
  
  void TeleopPeriodic() override {

     int colorcode = 0;
     int shooterspeed = 0;

     double currentDistance = EstimateDistance();

     shooterspeed = (6380 / 600) * (shooter1.GetSelectedSensorVelocity() / (1.5));
     frc::SmartDashboard::PutNumber("Shooter Speed", shooterspeed);

     //double distanceFunction = EstimateDistance();
     frc::SmartDashboard::PutNumber("Estimated Distance", currentDistance);

    // Turn led off so my eyes don't burn while testing
    //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1);

    
    #pragma region // Drive Code

    // Toggle between tank and arcade
    if (m_rightStick.GetRawButtonPressed(14)) {
      driveCodeToggle = !driveCodeToggle;
    }
    //frc::SmartDashboard::PutBoolean("Drive Toggle", driveCodeToggle);
    
    // Drive Code
    if (driveCodeToggle) 
    {
      // Drive with arcade style
      m_robotDrive.ArcadeDrive(m_rightStick.GetY(), -m_rightStick.GetX());
    /*
      frc::SmartDashboard::PutNumber("Slider Value", m_rightStick.GetRawAxis(3));
      float sliderRawValue = m_rightStick.GetRawAxis(3);
      float powerValue = (sliderRawValue + 1) / 2;
      frc::SmartDashboard::PutNumber("Power Value", powerValue);

      int driveInt = 0;

      if (m_rightStick.GetY() >= 0.25) 
      {
        driveInt = -1;
      }
      else if (m_rightStick.GetY() <= -0.25) 
      {
        driveInt = 1;
      }
      else 
      {
        driveInt = 0;
      }

      if (driveInt == 1) 
      {
        m_robotDrive.ArcadeDrive(-powerValue, -m_rightStick.GetX(), false);
        frc::SmartDashboard::PutString("Drive Direction", "Forward");
      }
      else if (driveInt == -1) 
      {
        m_robotDrive.ArcadeDrive(powerValue, -m_rightStick.GetX(), false);
        frc::SmartDashboard::PutString("Drive Direction", "Backward");
      }
      else 
      {
        m_robotDrive.ArcadeDrive(0, -m_rightStick.GetX(), false);
        frc::SmartDashboard::PutString("Drive Direction", "N/A");
      }
      */
    }
    else 
    {
      // Drive with backwards arcade style
      m_robotDrive.ArcadeDrive(-m_leftStick.GetY(), m_rightStick.GetY());
    }
    
    #pragma endregion

    #pragma region // Shooter Control By Percent

    double shooterPercent;

    // Shooter control basic
    shooterPercent = 0.002244 * (currentDistance) + 0.2469; //Needs Tuned with values from limelight - plot distance vs. percent output and find best fit line
    
    if (shooterPercent > 0.6)
      {
        shooterPercent = 0.6;
      }
    
    if (m_rightStick.GetRawButton(3)) 
    {
      shooter1.Set(ControlMode::PercentOutput, shooterPercent);
      shooter2.Set(ControlMode::PercentOutput, shooterPercent);
    }
    else if (m_rightStick.GetRawButton(2)) //low dump set when intake against fender
    {
      shooter1.Set(ControlMode::PercentOutput, 0.26);
      shooter2.Set(ControlMode::PercentOutput, 0.26);
    }
    else if (m_rightStick.GetRawButton(4)) 
    {
      shooter1.Set(ControlMode::PercentOutput, 0.47);
      shooter2.Set(ControlMode::PercentOutput, 0.47);
    }
    else 
    {
      shooter1.Set(ControlMode::PercentOutput, 0.0);
      shooter2.Set(ControlMode::PercentOutput, 0.0);
    }
    
   #pragma endregion

   #pragma region //shooter control PID

    // max percent output - 0.58
    /*
    if (m_rightStick.GetRawButton(7))
    {
      shooter1.Set(ControlMode::PercentOutput, 0.58);
      shooter2.Set(ControlMode::PercentOutput, 0.58);
    }
    else
    {
      shooter1.Set(ControlMode::PercentOutput, 0.0);
      shooter2.Set(ControlMode::PercentOutput, 0.0);
    }
    
    if (shooterspeed > maxSpeed)
    {
      maxSpeed = shooterspeed;
    }
    frc::SmartDashboard::PutNumber("Max Speed", maxSpeed);
    /*
    if (m_rightStick.GetRawButtonPressed(3)) //test shooter RPM
    {
      shooterspeed = shooterspeed - 100; 
    }
    else if (m_rightStick.GetRawButtonPressed(4))
    {
      shooterspeed = shooterspeed + 100;
    }
  
    bool shoot = false;

    if (m_rightStick.GetRawButton(2))
    {
      shoot = true;
    }
    else
    {
      shoot = false;
    }

    if (shoot = true)
    {
    shooter1.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity, shooterspeed);
    }
    else
    {
    shooter1.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,0);
    }
    
     frc::SmartDashboard::PutNumber("Shooter RPM", shooterspeed);
    */

    #pragma endregion

    #pragma region // Color Match Code

    // Color Match Code
    frc::Color detectedColor = m_colorSensor.GetColor();

    std::string ballColor;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget && confidence > 0.85)  // I hate that I can't use a switch statement
    {  
      ballColor = "Blue";
      colorcode = 1;
    }
    else if (matchedColor == kRedTarget && confidence > 0.85) 
    {
      ballColor = "Red";
      colorcode = 2;
    }
    else if (confidence < 0.85)
    {
      ballColor = "Unknown";
      colorcode = 0;
    }

    // Use detected RGB values to calibrate
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", ballColor);

    #pragma endregion

    #pragma region //intake code
 
    //arms up and down
    if (xboxController.GetRawButton(1))
    {
      armMotor.Set(-0.25);
    }
    else if(xboxController.GetRawButton(2))
    {
      armMotor.Set(0.25);
    }
    else if (xboxController.GetRawButton(6))
    {
      armMotor.Set(-0.3);
    }
    else 
    {
      armMotor.Set(0.0);
    }

    if (xboxController.GetRawButton(3))
    {
      intakeMotor.Set(-0.77);
    }
    else if (xboxController.GetRawButton(4))
    {
      intakeMotor.Set(0.77);
    }
    else if (xboxController.GetRawButton(6))
    {
      intakeMotor.Set(-0.3);
    }
    else
    {
      intakeMotor.Set(0.0);
    }

    if (xboxController.GetRawButton(7))
    {
      stagingMotor.Set(0.5);
    }
    else if (xboxController.GetRawButton(8))
    {
      stagingMotor.Set(-0.5);
    }
    else if (xboxController.GetRawButton(6))
    {
      stagingMotor.Set(0.5);
    }
    else
    {
      stagingMotor.Set(0.0);
    }
  

    #pragma endregion

    #pragma region //intake with color sorter

    bool currentTeamColor = frc::SmartDashboard::GetBoolean("IsRedAlliance", true);
    frc::SmartDashboard::PutBoolean("Team Alliance Bool", currentTeamColor);
    std::string teamColor = "n/a";
    if (currentTeamColor)
    {
      teamColor = "Red"; 
    }
    else {
      teamColor = "Blue";
    }

/*
    if (xboxController.GetRawButton(10))
    {
      // No ball
      if (ballColor == "Unknown") // This case could cause issues. Requires testing
      {
        intakeMotor.Set(-0.75);
        stagingMotor.Set(0.5);
      }
      // Bad ball
      else if (ballColor != teamColor)
      {
        intakeMotor.Set(-0.75);
        stagingMotor.Set(0.5);
      }
      // Good ball
      else if (ballColor == teamColor)
      {
        intakeMotor.Set(-0.75);
        stagingMotor.Set(0.0);
      }
    }
    else if (xboxController.GetRawButton(4))
    {
      intakeMotor.Set(0.75);
      stagingMotor.Set(-0.5);
    }
    else if (xboxController.GetRawButton(6))
    {
      stagingMotor.Set(0.5);
    }
    else
    {
      intakeMotor.Set(0.0);
      stagingMotor.Set(0.0);
    }
    
    */
    
    
    /*

    if (xboxController.GetRawButton(3))
    {
      if (colorcode = 0)
      {
        intakeMotor.Set(-0.75);
        stagingMotor.Set(0.5);
      }
      else if (colorcode = 1)
      {
        intakeMotor.Set(-0.75);
        stagingMotor.Set(0.5);
      }
      else if (colorcode = 2)
      {
        intakeMotor.Set(-0.75);
        stagingMotor.Set(0.0);
      }
    }
    else if (xboxController.GetRawButton(4))
    {
      intakeMotor.Set(0.75);
      stagingMotor.Set(-0.5);
    }
    else if (xboxController.GetRawButton(6))
    {
      stagingMotor.Set(0.5);
    }
    else
    {
      intakeMotor.Set(0.0);
      stagingMotor.Set(0.0);
    }
    
    //arms up and down
    if (xboxController.GetRawButton(1))
    {
      armMotor.Set(-0.25);
    }
    else if(xboxController.GetRawButton(2))
    {
      armMotor.Set(0.25);
    }
    else if (xboxController.GetRawButton(6))
    {
      armMotor.Set(-0.25);
    }
    else 
    {
      armMotor.Set(0.0);
    }
    */

    #pragma endregion
    
    #pragma region // Limelight code

    // Tracking
    if (m_rightStick.GetRawButton(1)) 
    {
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",0);
      double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      double ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      double tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);
      double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      double minDistance = 100;
      double maxDistance = 150;
      double currentDistance = EstimateDistance();
      float kpDistance = -0.5f;

      std::string s = std::to_string(currentDistance);
      frc::SmartDashboard::PutString("DB/String 0", s);

      /*
      * I came up with 2 ways of limelight tracking.
      * I'm unsure which would be better in practice.
      * My first impression is the first way would be more jerky, but more reliable.
      * While the second may be smoother to drive, but could run into issues if bumped by another robot.
      */
      
      // distance and rotation test
      /*
      * - rotates to face target
      * - executes distance driving
      * - pauses distance driving if robot drifts and is no longer aligned
      */
      int rotationError = -1;

      if (!(tx < rotationError && tx > -rotationError)) 
      {
        tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
        if (tx > rotationError) 
        {
          tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
          m_robotDrive.TankDrive(-0.55,0.55);
        }
        else if (tx < -rotationError) 
        {
          tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
          m_robotDrive.TankDrive(0.55,-0.55);
        }
      }
      // unsure if this is the cleanest way to nest these
      else if (currentDistance > maxDistance)
      {
        m_robotDrive.TankDrive(-0.55, -0.55);
      }
      else if (currentDistance < minDistance)
      {
        m_robotDrive.TankDrive(0.55, 0.55);
      }
      else
      {
        m_robotDrive.TankDrive(0, 0);
      }

      /*
      * - rotates to face target
      * - drives to set distance
      * - loops until both are correct
      */
      // I want to see how this idea could work, but I don't see a practical way of doing it outside of a 'while()' loop.
      // and that is a very bad practice in a situation like this


      // Limelight docs
      /*
      float distanceError = 120 - currentDistance;
      double driveCommand = kpDistance * distanceError;

      m_robotDrive.TankDrive(-driveCommand, -driveCommand);
      */
    }  

    #pragma endregion


  }

  double EstimateDistance() {
    double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty",0.0);
    double h2 = 104; // Height of target
    double h1 = 23.75; // Height of camera from floor
    double a1 = 31;  // Yaw of camera

    // Based off limelight docs
    double angleToGoalDegrees = a1 + ty;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    double d = (h2 - h1)/ tan(angleToGoalRadians);

    //double d = (h2 - h1) / tan(a1 + ty);
    return d;
    //D Returns in inches

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
