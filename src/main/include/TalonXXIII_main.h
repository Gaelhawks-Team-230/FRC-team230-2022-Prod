// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <string>
#include <thread>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>
#include <frc/DriverStation.h>
#include "ctre/Phoenix.h"
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/motorcontrol/VictorSP.h>
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include <cameraserver/CameraServer.h>
#include "cscore_oo.h"
#include <cscore_cpp.h>
#include "Autonomous.h"
#include "TrajPlan.h"
#include "Sample.h"
#include "JoystickState.h"
#include "Limelight.h"
#include "Gatherer.h"
#include "Shooter.h"
#include "Drivetrain.h"
#include "Climber.h"

#include "ctre/phoenix/motorcontrol/TalonFXSensorCollection.h"
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"


class TalonXXIII : public frc::TimedRobot 
{
 public:
  TalonXXIII();
  frc::PowerDistribution *pdp;
  Sample *test;
  TrajPlan *trajPlan;
  JoystickState *userInput;
  Limelight *limelight;
  Shooter *shooter;
  Drivetrain *drive;
  Gatherer *gatherer;
  Climber *climb;

  double addedDrive;
  double addedRotate;
  bool isAuto;
  bool isTraj;
  
  void RobotInit() override;
  void InitializeAlliance();
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void ServiceDash(void);
  void CommunityService(void);
  void Omnicide(void);
  static double Limit(double min, double max, double curValue);
  static double Sign(double);
  void RobotStartingConfig(void);
  void TestPeriodic() override;
  void TestInit() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;

  void DrivePath(int pathNum);
  void DoNothing(void);
  void OffTarmac(void);
  void OneBall(void);
  void TwoBall(void);
  void ThreeBall(void);
  void FourBall(void);
  void FiveBall(void);
  void TwoBallDefense(void);
  void ModeSelection(void);

 private:

  frc::SendableChooser<int> *AutoModeChooser;
  int dashCounter;
  bool isBlueAlliance;
  bool firstTime;
  double driveCmd;
  double isGyroOn;
  bool isInTeleop;
  double robotStartTime;
  double robotEndTime;
  double robotRunCount;
  int loopCount;
  
  double rotateCmd;
  double velCmd;
  int trajIndex;
  bool useTargetRotate;

  int autoMode;
  int autoStage;
  int mode;
  bool isPathRead;
  int autoFile;
  double deltaTargetAlignment;

  int climbCmd;

  int DO_NOTHING = 0;
  int OFF_TARMAC = 1;
  int ONE_BALL = 2;
  int TWO_BALL = 3;
  int THREE_BALL = 4;
  int FOUR_BALL = 5;
  int FIVE_BALL = 6;
  int TWO_BALL_DEFENSE = 7;

  static void CameraThread()
  {
    //cs::UsbCamera usbCamera("USB Camera 0", 0);
    // cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    // //cs::MjpegServer mjpegServer1("serve_USB Camera 0", 1185);
    // cs::MjpegServer mjpegServer1 = frc::CameraServer::AddServer("serve_USB Camera 0", 1182);
    // mjpegServer1.SetSource(camera);

    // cs::CvSink cvSink("opencv_USB Camera 0");
    // cvSink.SetSource(usbCamera);

    // cs::CvSource outputStream("Blur", cs::VideoMode::kMJPEG, 320, 240, 15);
    // cs::MjpegServer mjpegServer2("serve_Blur", 1182);
    // mjpegServer2.SetSource(outputStream);

    
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
    camera.SetVideoMode(cs::VideoMode::kMJPEG, 320, 240, 15);
    // printf("here\n");
    // printf("Camera Mode: %d, FPS: %d\n", camera.GetVideoMode().pixelFormat, camera.GetVideoMode().fps);


    // camera.SetResolution(160, 120);
    // camera.SetFPS(20);
  }

};
