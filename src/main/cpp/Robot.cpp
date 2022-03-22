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

//Gyro https://juchong.github.io/ADIS16470-RoboRIO-Driver/classfrc_1_1_a_d_i_s16470___i_m_u.html
#include <frc/ADIS16470_IMU.h>

//driverstation
#include <frc/DriverStation.h>
#include <hal/DriverStation.h>
#include <hal/DriverStationTypes.h>

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

//WPILIB Trajectory
//constexpr auto ks = 0.0_V; 
// Writing in here causes issues and idk why

#pragma endregion


#pragma region  // wpilib trajectory code

// idk if this is the best place for these.
// I would place them inside their own header file but I run into issues
//WPILIB Trajectory
// DO NOT USE THESE VALUES!! THESE ARE PLACEHOLDERS UNTIL WE HAVE TIME TO CALCULATE OUR OWN
constexpr auto ks = 0.22_V;
constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;
    
constexpr double kPDriveVel = 8.5;
    
constexpr auto kTrackWidth = 0.69_m;
extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;

// These values should work with most robots.
// If having issues, tune them here - https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html#constructing-the-ramsete-controller-object
constexpr double kRamseteB = 2;
constexpr double kRamseteZeta = 0.7;


class DriveSubsystem : public frc2::SubsystemBase {
  public:
    DriveSubsystem();

    void Periodic() override;

    // Subsystem methods go here

    // Drives the robot using arcade controls
    void ArcadeDrive(double fwd, double rot);

    // Controls each side of the robot directly with voltage
    void TankDriveVolts(units::volt_t left, units::volt_t right);

    // Resets the drive encoders to read a position of 0
    void ResetEncoders();

    // Gets the average distance of the TWO encoders
    double GetAverageEncoderDistance();

    // Gets the left drive encoder
    frc::Encoder& GetLeftEncoder();

    // Gets the right drive encoder
    frc::Encoder& GetRightEncoder();

    // Sets the max output of the drive. Useful for scaling the drive to drive more slowly
    void SetMaxOutput(double maxOutput);

    // Returns the heading of the robot
    units::degree_t GetHeading() const;

    // Return the turn rate of the robot
    double GetTurnRate();

    // Returns the currently-estimated pose of the robot
    frc::Pose2d GetPose();

    // Returns the current wheel speeds of the robot
    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    // Resets the odometry of the specified pose
    void ResetOdometry(frc::Pose2d pose);

  private:
    // Components (motor controllers and sensors) should generally be
    // declared private and exposed only through public methods

    // Motor controllers
    frc::PWMVictorSPX m_leftMotor{1};
    frc::PWMVictorSPX m_rightMotor{2};

    // Motors on the left side of the drive
    frc::MotorControllerGroup m_leftMotorGroup{m_leftMotor};

    // Motors on the right side of the drive
    frc::MotorControllerGroup m_rightMotorGroup{m_rightMotor};

    // Robot drive
    frc::DifferentialDrive m_drive{m_leftMotorGroup, m_rightMotorGroup};  // idk if I need groups since we only have two motor controllers total

    // Left-side drive encoder
    frc::Encoder m_leftEncoder;

    // Right-side drive encoder
    frc::Encoder m_rightEncoder;

    // Gyro sensor
    frc::ADIS16470_IMU m_gyro;

    // Odometry class for tracking robot pose
    frc::DifferentialDriveOdometry m_odometry;
    
};

DriveSubsystem::DriveSubsystem()
  : m_leftMotor{1},
    m_rightMotor{2},
    m_leftEncoder{0, 1},
    m_rightEncoder{0, 1},
    m_odometry{m_gyro.GetAngle()} {  // By default GetAngle() calculates Y axis. This could be wrong angle for this purpose idk

      // Depending on our drivetrain, may need to invert left instead
      m_rightMotorGroup.SetInverted(true);

      // Set the distance per pulse for the encoders
      m_leftEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);
      m_rightEncoder.SetDistancePerPulse(kEncoderDistancePerPulse);

      ResetEncoders();
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here
  m_odometry.Update(m_gyro.GetAngle(),
                    units::meter_t(m_leftEncoder.GetDistance()),
                    units::meter_t(m_rightEncoder.GetDistance()));
}

void DriveSubsystem::ArcadeDrive(double fwd, double rot) {
  m_drive.ArcadeDrive(fwd, rot);
}

void DriveSubsystem::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMotorGroup.SetVoltage(left);
  m_rightMotorGroup.SetVoltage(right);
  m_drive.Feed();
}

void DriveSubsystem::ResetEncoders() {
  m_leftEncoder.Reset();
  m_rightEncoder.Reset();
}

double DriveSubsystem::GetAverageEncoderDistance() {
  return (m_leftEncoder.GetDistance() + m_rightEncoder.GetDistance()) / 2.0;
}

frc::Encoder& DriveSubsystem::GetLeftEncoder() {
  return m_leftEncoder;
}

frc::Encoder& DriveSubsystem::GetRightEncoder() {
  return m_rightEncoder;
}

void DriveSubsystem::SetMaxOutput(double maxOutput) {
  m_drive.SetMaxOutput(maxOutput);
}

units::degree_t DriveSubsystem::GetHeading() const {
  return m_gyro.GetAngle(); // This could cause issues if it doesn't return in degrees
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
  return {units::meters_per_second_t(m_leftEncoder.GetRate()),
          units::meters_per_second_t(m_rightEncoder.GetRate())};
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_odometry.ResetPosition(pose, m_gyro.GetAngle());
}

#pragma endregion


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override {

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

    #pragma region //1 ball auton

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
      */
     #pragma endregion
   
    }
    else 
    {
      // Drive with backwards arcade style
      m_robotDrive.ArcadeDrive(-m_leftStick.GetY(), m_rightStick.GetX());
    }
    
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
      float offset = pidController.Calculate(tx);
      if (abs(tx) > 1 && offset >= 0.25)  // adding offset to this could cause issues. added to try to prevent issue between this and distance
      {
        float offset = pidController.Calculate(tx);
        m_robotDrive.TankDrive(offset, -offset);
      }
      
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