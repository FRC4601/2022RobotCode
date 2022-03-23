// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/controller/PIDController.h>
#include <cameraserver/CameraServer.h>
#include <frc/DigitalInput.h>

// CTRE Docs - https://docs.ctre-phoenix.com/en/stable/ch05a_CppJava.html
#include <ctre/Phoenix.h>

#include <iostream>
#include <memory>
#include <string.h>
#include <cstdlib>
#include <cmath>

// Limelight directives
// API - https://docs.limelightvision.io/en/latest/getting_started.html
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include "wpi/span.h"
#include <wpi/PortForwarder.h>

// Rev directives
// API - https://codedocs.revrobotics.com/cpp/namespacerev.html -/- Relative Encoder Class https://codedocs.revrobotics.com/cpp/classrev_1_1_relative_encoder.html
#include <rev/CANSparkMax.h>
#include <rev/RelativeEncoder.h>  // useful doc ~ https://github.com/REVrobotics/SPARK-MAX-Examples/issues/15

#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>

#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>

// Pathplanner only helps write paths. In order to execute follow the wpilib trajectory
// https://docs.wpilib.org/en/stable/docs/software/pathplanning/trajectory-tutorial/trajectory-tutorial-overview.html
//FRC Pathplanner
#include <pathplanner/lib/PathPlanner.h>

// I don't know what i need to keep with these so I'm not going to delete these yet
// WPILIB Trajectory
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/Encoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <units/voltage.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/trajectory/constraint/TrajectoryConstraint.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <wpi/SymbolExports.h>

//Gyro https://juchong.github.io/ADIS16470-RoboRIO-Driver/classfrc_1_1_a_d_i_s16470___i_m_u.html
#include <frc/ADIS16470_IMU.h>

//driverstation
#include <frc/DriverStation.h>
#include <hal/DriverStation.h>
#include <hal/DriverStationTypes.h>

#include <wpi/StringMap.h>
#include <wpi/mutex.h>
#include <wpi/sendable/SendableRegistry.h>

#pragma region // Initialization

// Initialization
bool shooterArmPosition = false;  // false - up ~ true - down
bool driveCodeToggle = true;  // true - tank // false - arcade // (currently true - arcade forward //false - arcade backwards)
bool flyWheelToggle = true;
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
frc::Spark blinkin {0};
frc::PWMVictorSPX m_leftMotor{1};
frc::PWMVictorSPX m_rightMotor{2};
frc::PWMVictorSPX armMotor{3};
frc::PWMVictorSPX intakeMotor{4};
frc::PWMVictorSPX stagingMotor{5};
frc::PWMVictorSPX shooter3{6};

// Robot Drive
frc::DifferentialDrive m_robotDrive{m_leftMotor, m_rightMotor};

// Joysticks
frc::Joystick m_leftStick{0};
frc::Joystick m_rightStick{1};
frc::XboxController xboxController{2};
frc::PIDController pidController{0, 0, 0};

int maxSpeed = 0;

//Pathplanner

//timer
frc::Timer m_timer; 

//Encoder
frc::Encoder ArmEncoder{0,1};

//Gyro
frc::ADIS16470_IMU imu{frc::ADIS16470_IMU::IMUAxis::kZ, frc::SPI::Port::kOnboardCS0, frc::ADIS16470_IMU::CalibrationTime::_4s};

//limit switches
frc::DigitalInput lSwitch1{3}; 
frc::DigitalInput lSwitch2{4};

//Sendable chooser
//std::string autonList = {"Forward", "Backward"};
//wpi::span<const std::__cxx11::string> test1 = {"Forward", "Backward"};
//frc::SendableChooser<autonTest> m_chooser;

#pragma endregion


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {

    //Sendable chooser
    //m_chooser.AddOption("Forward", autonTest::FORWARD);
    //m_chooser.AddOption("Backward", autonTest::BACKWARDS);
    //frc::SmartDashboard::PutStringArray("Auto List", {"Forward", "Backward"});  //This is exactly whats in their own fucking docs. And it doesn't work. Every fucking year shit like this happens. Yet we are just supposed to work with it. If you can't maintain ur own docs, you shouldnt have them up acting like they work. go fuck yourselves frc. 

    // Motor inverts
    m_leftMotor.SetInverted(true);
    armMotor.SetInverted(true);
    intakeMotor.SetInverted(true);
    shooter3.SetInverted(true);

    // Color Match Targets
    m_colorMatcher.AddColorMatch(kBlueTarget);
    m_colorMatcher.AddColorMatch(kRedTarget);

    //shooter PID control
    shooter1.ConfigFactoryDefault();
    shooter2.ConfigFactoryDefault();
    shooter2.Follow(shooter1);
    shooter1.SetInverted(TalonFXInvertType::CounterClockwise);
    shooter2.SetInverted(TalonFXInvertType::Clockwise);
    shooter1.ConfigNominalOutputForward(0);
    shooter1.ConfigNominalOutputReverse(0);
    shooter1.ConfigPeakOutputForward(1);
    shooter1.ConfigPeakOutputReverse(-1);
    shooter1.Config_kF(0,0.07); 
    shooter1.Config_kP(0, 0.1);
    shooter1.Config_kI(0,0);
    shooter1.Config_kD(0,0);

    //limelight
    wpi::PortForwarder::GetInstance().Add(5800, "limelight.local", 5800);
    wpi::PortForwarder::GetInstance().Add(5801, "limelight.local", 5801);
    wpi::PortForwarder::GetInstance().Add(5802, "limelight.local", 5802);
    wpi::PortForwarder::GetInstance().Add(5803, "limelight.local", 5803);
    wpi::PortForwarder::GetInstance().Add(5804, "limelight.local", 5804);
    wpi::PortForwarder::GetInstance().Add(5805, "limelight.local", 5805);
    wpi::PortForwarder::GetInstance().Add(1181, "wpilibpi.local", 1181);

    
  };
  
  
  void AutonomousInit() override {
    m_timer.Reset();
    m_timer.Start();
  };
  
  void AutonomousPeriodic() override {
    
    #pragma region // Pathplanner auton
    // idea: can create multiple paths with shoot commands added inbetween them

    pathplanner::PathPlannerTrajectory testPath = pathplanner::PathPlanner::loadPath("Test Path", 7_mps, 4_mps_sq);

    #pragma endregion


    #pragma region  // 2 ball auton

    //autonTest startPosition = m_chooser.GetSelected();

    /*
    switch(startPosition) {
      case FORWARD:
        if (m_timer.Get() < 2_s)
        {
          m_robotDrive.TankDrive(0.5, 0.5);
        }
        else 
        {
          m_robotDrive.TankDrive(0.0, 0.0);
        }
        break;
      case BACKWARDS:
        if (m_timer.Get() < 2_s)
        {
          m_robotDrive.TankDrive(-0.5, -0.5);
        }
        else 
        {
          m_robotDrive.TankDrive(0.0, 0.0);
        }
        break;
      default:
      break;
    }
    */

    

    

/*
    #pragma region // 1 ball auton

    if (m_timer.Get() < 3_s) //set up 3 feet off fender
    {
      shooter1.Set(ControlMode::Velocity, 2215);
      shooter3.Set(.6);
    }
    else if (m_timer.Get() >= 3_s && m_timer.Get() < 6_s)
    {
      shooter3.Set(1.0);
      stagingMotor.Set(0.5);
    }
    else if (m_timer.Get() >= 6_s && m_timer.Get() < 7_s)
    {
      shooter1.Set(ControlMode::Velocity, 0);
      shooter3.Set(0);
      stagingMotor.Set(0);
    }
    else if (m_timer.Get() >= 7_s && m_timer.Get() < 9_s)
    {
      m_robotDrive.ArcadeDrive(0.6, 0.0);

    }
    else if (m_timer.Get() >= 10_s && m_timer.Get() < 11_s)
    {
      m_robotDrive.ArcadeDrive(0.0, 0.0);

    }

    #pragma endregion
*/

  };

  void TeleopInit() override {
      

  };
  
  void TeleopPeriodic() override {

    int shooterspeed = 0;
     
    double currentDistance = EstimateDistance();

    frc::SmartDashboard::PutNumber("Shooter Speed", shooterspeed);
    frc::SmartDashboard::PutNumber("Estimated Distance", currentDistance);

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",0);

 
    #pragma region // Drive Code

    // Toggle between tank and arcade
    if (m_rightStick.GetRawButtonPressed(14)) 
    {
      driveCodeToggle = !driveCodeToggle;
    }
    //frc::SmartDashboard::PutBoolean("Drive Toggle", driveCodeToggle);
    
    frc::SmartDashboard::PutNumber("Axis X", (float)imu.GetAngle());

    // Drive straight
    //frc docs
    /*
    if (m_rightStick.GetRawButton(4)) {
      float kP = 0.05f;
      units::degree_t error = -imu.GetAngle();
      float turnPower = kP * (float)error;
      m_robotDrive.ArcadeDrive(m_rightStick.GetY(), turnPower, false);
    }
    */

    /*
    if (driveCodeToggle) 
    {
      // Drive with arcade style
      m_robotDrive.ArcadeDrive(m_rightStick.GetY(), -m_rightStick.GetX());

      #pragma region //drive with arcade and speed slider
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
      
     #pragma endregion
   
    }
    else 
    {
      // Drive with backwards arcade style
      m_robotDrive.ArcadeDrive(-m_leftStick.GetY(), m_rightStick.GetX());
    }
    */
    
    #pragma endregion

    #pragma region // Shooter Control

    double shooterVelocity;

    if (xboxController.GetRawButtonPressed(12))
    {
      flyWheelToggle = !flyWheelToggle;
    }

    //shooterVelocity = 26.78 * (currentDistance) + 4493; //Could be Tuned Better//Needs Better Data//Equation for flywheel speed
    
    if (shooterVelocity > 9500) //limit shootervelocity to 9500 units/100ms
      {
        shooterVelocity = 9500;
      }
    
    if (m_rightStick.GetRawButton(3)) 
    {
      shooter1.Set(ControlMode::Velocity, 3500); //tuned at limelight estimate distance 115
      shooter3.Set(1);
    }
    else if (m_rightStick.GetRawButton(2))
    {
      shooter1.Set(ControlMode::Velocity, 2600);
      shooter3.Set(.15);
    }
    else 
    {
     if (flyWheelToggle){
      shooter1.Set(ControlMode::Velocity, 0); //2500 base run
      shooter3.Set(0.0);
     }
     else{
      shooter1.Set(ControlMode::Velocity, 2000);
      shooter3.Set(0.15);
     }
    }
    
    #pragma endregion


    #pragma region // Color Match Code

    // Color Match Code
    frc::Color detectedColor = m_colorSensor.GetColor();

    std::string ballColor;
    double confidence = 0.0;
    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    if (matchedColor == kBlueTarget && confidence > 0.85)  
    {  
      ballColor = "Blue";
    }
    else if (matchedColor == kRedTarget && confidence > 0.85) 
    {
      ballColor = "Red";
    }
    else if (confidence < 0.85)
    {
      ballColor = "Unknown";
    }

    // Use detected RGB values to calibrate
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("Confidence", confidence);
    frc::SmartDashboard::PutString("Detected Color", ballColor);

    #pragma endregion


    #pragma region //intake arm code
    
    int armAngle;
    armAngle = ArmEncoder.GetRaw();
    ArmEncoder.SetDistancePerPulse(360/8192);
    ArmEncoder.SetMinRate(10);
    ArmEncoder.SetSamplesToAverage(5);
    ArmEncoder.SetReverseDirection(true);
    frc::SmartDashboard::PutNumber("EncoderValue", armAngle);

   
    // armMotor
    if (xboxController.GetRawButton(1))
    {
      if(armAngle > 1000)
      {
      armMotor.Set(0.25);
      }
      else if(armAngle < 1000)
      {
        armMotor.Set(0.0);
      }
      
    }
    else if(xboxController.GetRawButton(2))
    {
      if (armAngle < 1400)
      {
        armMotor.Set(-0.25);
      }
      else if (armAngle > 1400)
      {
        armMotor.Set(0.0);
      }
    }
    else if (xboxController.GetRawButton(6))
    {
      if(armAngle > 1000)
      {
      armMotor.Set(0.25);
      }
      else if(armAngle < 1000)
      {
        armMotor.Set(0.0);
      }
    }
    else 
    {
      armMotor.Set(0.0);
    }

    #pragma endregion


    #pragma region //intake with color sorter (staging and intake wheels)

    std::string teamColor = "n/a";
    frc::DriverStation::Alliance alliance;
    alliance = frc::DriverStation::GetInstance().GetAlliance();
    frc::SmartDashboard::PutString("Alliance Color", teamColor);

    if (alliance == frc::DriverStation::Alliance::kRed)
    {
      teamColor = "Red"; 
    }
    else if (alliance == frc::DriverStation::Alliance::kBlue)
    {
      teamColor = "Blue";
    }
    else
    {
      teamColor = "n/a";
    }


    if (xboxController.GetRawButton(3))
    {
      // No ball
      if (ballColor == "Unknown") 
      {
        intakeMotor.Set(1.0);
        stagingMotor.Set(0.3);
      }
      // Bad ball
      else if (ballColor != teamColor)
      {
        intakeMotor.Set(1.0);
        stagingMotor.Set(0.3);
      }
      // Good ball
      else if (ballColor == teamColor)
      {
        intakeMotor.Set(1.0);
        stagingMotor.Set(0.0);
      }
    }
    else if (xboxController.GetRawButton(4))
    {
      intakeMotor.Set(-0.75);
      stagingMotor.Set(-0.5);
    }
    else if (xboxController.GetRawButton(6))
    {
      stagingMotor.Set(0.5);
      intakeMotor.Set(0.3);
    }
    else
    {
      intakeMotor.Set(0.0);
      stagingMotor.Set(0.0);
    }

    #pragma endregion
    
    
    #pragma region // Limelight code

    // Tracking
    if (m_rightStick.GetRawButton(1)) 
    {
      nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("pipeline",0);
      //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 1);
      double tx = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tx", 0.0);
      double ta = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ta", 0.0);
      double tv = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0.0);
      double ty = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("ty", 0.0);
      double minDistance = 100;
      double maxDistance = 125;
      double currentDistance = EstimateDistance();
      float kpDistance = -0.5f;

      std::string s = std::to_string(currentDistance);
      frc::SmartDashboard::PutString("DB/String 0", s);


      pidController.SetP(0.0365); //Warren known decent values 0.0365, 0.007, 0.0
      pidController.SetI(0.007);
      pidController.SetD(0.0);
      pidController.SetSetpoint(0);

      // rotational pid loop
      //float offset = pidController.Calculate(tx);
      if (abs(tx) > 1)  // adding offset to this could cause issues. added to try to prevent issue between this and distance
      {
        float offset = pidController.Calculate(tx);
        m_robotDrive.TankDrive(offset, -offset);
      }
      /*
      why is this causing build issues?
      // idea: use offset to calculate when robot is aligned. then execute distance code based on that
      float offset = pidController.Calculate(tx);
      if (offset < 0.25)  // value needs calibrating // should execute when aligned
      {
        // min/max distance code
        if (currentDistance > maxDistance)
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
      }
      */
    }  

    if (m_rightStick.GetRawButton(4)) {
      // min/max distance code
        if (currentDistance > 100)
        {
          m_robotDrive.TankDrive(-0.55, -0.55);
        }
        else if (currentDistance < 100)
        {
          m_robotDrive.TankDrive(0.55, 0.55);
        }
        else
        {
          m_robotDrive.TankDrive(0, 0);
        }
    }
    
    #pragma endregion


    #pragma region //Color Codes

    if (m_rightStick.GetRawButton(1)) //limelight tracking
    {
      blinkin.Set(0.73);
    }
    else if (m_rightStick.GetRawButton(2)) //low dump flywheel
    {
      blinkin.Set(-0.57);
    }
    else if (m_rightStick.GetRawButton(3)) //high shot flywheel
    {
       blinkin.Set(-0.05);

    }
    else if (xboxController.GetRawButton(6)) //fire
    {
      blinkin.Set(-0.07);
    }
    else
    {
      if (alliance == frc::DriverStation::Alliance::kRed)
        {
          blinkin.Set(-0.11);
        } 
      else if (alliance == frc::DriverStation::Alliance::kBlue)
      {
        blinkin.Set(-0.09);
      }
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
void DisabledInit() override
  {
  };

void DisabledPeriodic() override
  {
  //Turn limelight led off
  //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode",1); 
  //addressable LED Color Settings found at https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf
  };

};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif