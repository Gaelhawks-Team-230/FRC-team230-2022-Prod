#include <frc/Timer.h>
#include <iostream>
#include "TalonXXIII_main.h"
#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/PowerDistribution.h>

using namespace ctre::phoenix::motorcontrol;

TalonXXIII::TalonXXIII():TimedRobot(LOOPTIME)
{
  pdp = new frc::PowerDistribution();
  //frc::CameraServer::StartAutomaticCapture();
  trajPlan = new TrajPlan(this);
  test = new Sample(this);
  userInput = new JoystickState(this);
  limelight = new Limelight(this);
  shooter = new Shooter(this);
  drive = new Drivetrain(this);
  gatherer = new Gatherer(this);
  climb = new Climber(this);
  loopCount = 0;

  AutoModeChooser = new frc::SendableChooser<int>;

  dashCounter = 0;
  isBlueAlliance = false;
  firstTime = true;
  driveCmd = 0.0;
  rotateCmd = 0.0;
  isGyroOn = false;
  isInTeleop = false;

  addedDrive = 0.0;
  addedRotate = 0.0;

  climbCmd = 0;

  deltaTargetAlignment = 0.0;
  
  //RobotStartingConfig();
}

void TalonXXIII::RobotInit() 
{
  // std::thread cameraThread(CameraThread);
  // cameraThread.detach();

  AutoModeChooser->SetDefaultOption("Do Nothing", DO_NOTHING);
  AutoModeChooser->AddOption("Off Tarmac", OFF_TARMAC);
  AutoModeChooser->AddOption("Two Ball", TWO_BALL);
  AutoModeChooser->AddOption("Four Ball", FOUR_BALL);
  AutoModeChooser->AddOption("Five Ball", FIVE_BALL);
  AutoModeChooser->AddOption("Defense Two Ball", TWO_BALL_DEFENSE);

  frc::SmartDashboard::PutData("Auto Mode:", AutoModeChooser);

  ServiceDash();
}

void TalonXXIII::RobotPeriodic()
{
  if(firstTime)
  {
    InitializeAlliance();
    firstTime = false;
  }
}

void TalonXXIII::DisabledInit() 
{
  robotEndTime = (double)frc::GetTime();
  robotRunCount = (double)(robotEndTime-robotStartTime)/LOOPTIME;
  //printf("\nStartTime:%f,EndTime:%f,RunCount:%f, Loopcount:%d",robotStartTime,robotEndTime,robotRunCount,loopCount);
  Omnicide();
  limelight->TurnOffLED();
}

void TalonXXIII::InitializeAlliance() 
{
  if(frc::DriverStation::GetAlliance() == frc::DriverStation::kBlue)
		isBlueAlliance = true;
	else
		isBlueAlliance = false;
}

void TalonXXIII::DisabledPeriodic() 
{
  limelight->TurnOffLED();
}

void TalonXXIII::AutonomousInit() 
{
  isAuto = true;
  ModeSelection();
  shooter->LocalReset();
  drive->LocalReset();
  limelight->TurnOnLED();
  loopCount = 0;
  autoStage = 0;
  trajPlan->ReadAllFiles();
  //trajPlan->PrintAllPaths();
}

void TalonXXIII::AutonomousPeriodic() 
{
  loopCount++;
  limelight->Analyze();

  //printf ("Mode:%d\n", autoMode);

  switch(autoMode)
  {
    case 0:
      DoNothing();
      break;

    case 1:
      OffTarmac();
      break;

    case 2:
      OneBall();
      break;

    case 3:
      TwoBall();
      break;

    case 4:
      // ThreeBall();
      break;

    case 5:
      FourBall();
      break;

    case 6:
      FiveBall();
      break;

    case 7:
      TwoBallDefense();
      break;
  }

  gatherer->GathererToFloor();
  drive->DriveControl(driveCmd, rotateCmd, 0.0, 0.0, true, isTraj, velCmd);
  
  if(autoMode != 7)
  {
    shooter->TargetingUpper();
    shooter->StartShooterTargeting();
  }
  // shooter->TargetingUpper();
  // shooter->StartShooterTargeting();
  
  CommunityService();
  ServiceDash();
  // printf("Auto mode:%d\n", autoMode);
}

void TalonXXIII::TeleopInit() 
{
  loopCount = 0;
  robotStartTime = (double)frc::GetTime();
  shooter->LocalReset();
  drive->LocalReset();
  climb->StartingConfig();
  limelight->TurnOffLED();
}

void TalonXXIII::TeleopPeriodic() 
{
  loopCount++;
  userInput->Analyze();
  limelight->Analyze();

  if(drive->IsDriveTargeting())
  {
    limelight->TurnOnLED();
    // limelight->Analyze();
  }

  if(userInput->FlightCtrlBtnPushed(ENABLE_GYRO_BUTTON) || drive->IsDriveTargeting())
  {
    drive->GyroOn();
  }
  else
  {
    drive->GyroOff();
  }

  if(userInput->GamepadBtnPushed(GATHER_BALL_BUTTON))
  {
    gatherer->EnableGatherer();
    gatherer->GathererToFloor();
  }
  else if(shooter->IsShooting())
  {
    gatherer->EnableGatherer();
  }
  else if(userInput->GamepadBtnPushed(EJECT_BALL_BUTTON))
  {
    gatherer->EjectGatherer();
  }
  else
  {
    gatherer->DisableGatherer();
  }

  if(userInput->GamepadBtnPushed(IN_FRAME_BUTTON))
  {
    gatherer->FeederInFrame();
  }
  
  if(userInput->GamepadBtnPushed(LOAD_POS_BUTTON))
  {
    gatherer->FeedGatherer();
  }
  
  if(userInput->GamepadBtnPushed(FLOOR_POS_BUTTON))
  {
    gatherer->GathererToFloor();
  }

  if(userInput->FlightCtrlBtnPushed(FCONT_TARGET_BUTTON_ONE) || userInput->FlightCtrlBtnPushed(FCONT_TARGET_BUTTON_TWO))
  {
    drive->StartDriveTarget();
    shooter->StartShooterTargeting();
    shooter->TargetingUpper();
  }
  else if(userInput->GamepadBtnPushed(TARGET_UPPER_BUTTON))
  {
    drive->StartDriveTarget();
    shooter->TargetingUpper();
    shooter->StartShooterTargeting();
  }
  else if(userInput->GamepadBtnPushed(TARGET_LOWER_BUTTON) || userInput->FlightCtrlBtnPushed(FCONT_LOW_BUTTON_ONE) || userInput->FlightCtrlBtnPushed(FCONT_LOW_BUTTON_TWO))
  {
    shooter->TargetingLower();
  }
  else
  {
    limelight->TurnOffLED();
    drive->StopDriveTarget();
    shooter->StopTargeting();
    /*if(!(userInput->GamepadBtnPushed(FEED_SHOOTER_BUTTON) || userInput->FlightCtrlBtnPushed(FCONT_FEED_BUTTON_ONE) || userInput->FlightCtrlBtnPushed(FCONT_FEED_BUTTON_TWO)))
    {
      shooter->GiveIndexerCmd(0.0);
    }*/
  }

  if(userInput->GamepadBtnPushed(FEED_SHOOTER_BUTTON) || userInput->FlightCtrlBtnPushed(FCONT_FEED_BUTTON_ONE) || userInput->FlightCtrlBtnPushed(FCONT_FEED_BUTTON_TWO) || userInput->FlightCtrlBtnPushed(FCONT_TARGET_BUTTON_ONE) || userInput->FlightCtrlBtnPushed(FCONT_TARGET_BUTTON_TWO))
  {
    shooter->LoadShooter();
  }
  else
  {
    if(!shooter->IsLowerShooting())
    {
      shooter->GiveIndexerCmd(0.0);
    }
  }

  if(userInput->FlightCtrlBtnPushed(CLIMB_EXTRA_EXTEND_BUTTON))
  {
    climb->StartExtraExtend();
  }
  else
  {
    climb->EndExtraExtend();
  }

  if(userInput->GetDpadUpPushed())
  {
    climb->StartAutoPartialExtend();
  }
  else if(userInput->GetDpadRightPushed())
  {
    climb->StartAutoFinishExtend();
  }
  else if(userInput->GetDpadDownPushed())
  {
    climb->StartAutoStationaryRetract();
  }
  else
  {
    climb->StopAutoClimb();
  }

  if(userInput->GetFlightControllerButton(RECAL_BUTTON))
  {
    climb->WinchStartCal();
  }

  climb->GiveGoalTrackVel(userInput->GetGamepadAxis(CLIMB_TRACK_AXIS)*0.4);
  if(userInput->GetGamepadAxis(CLIMB_WINCH_AXIS)<0.0)
  {
    climb->GiveGoalWinchVel(userInput->GetGamepadAxis(CLIMB_WINCH_AXIS)*450.0);
  }
  else
  {
    climb->GiveGoalWinchVel(userInput->GetGamepadAxis(CLIMB_WINCH_AXIS)*225.0);
  }
  /*if(userInput->GetGamepadAxis(CLIMB_WINCH_AXIS)<-0.5)
  {
    climbCmd = 1;
  }
  else if(userInput->GetGamepadAxis(CLIMB_WINCH_AXIS)>0.5)
  {
    climbCmd = -1;
  }
  else
  {
    climbCmd = 0;
  }
  climb->AutoFullExtend(climbCmd);*/
  drive->DriveControl((userInput->GetFlightControllerAxis(SPEED_AXIS)), (userInput->GetFlightControllerAxis(ROTATE_AXIS)), addedDrive, addedRotate, false, false, 0.0);
  CommunityService();
  ServiceDash();
}

void TalonXXIII::ServiceDash()
{
  if(dashCounter == 30)
  {
    dashCounter = 0;
    userInput->UpdateDash();
    shooter->UpdateDash();
    drive->UpdateDash();
    gatherer->UpdateDash();
    climb->UpdateDash();
    limelight->UpdateDash();
  }
  else
  {
    dashCounter++;
  }
}

void TalonXXIII::CommunityService() 
{
  shooter->Service();
  drive->Service();
  gatherer->Service();
  climb->Service();
}

void TalonXXIII::Omnicide() 
{
  userInput->StopAll();
  shooter->StopAll();
  drive->StopAll();
  gatherer->StopAll();
  climb->StopAll();
}

double TalonXXIII::Limit(double min, double max, double curValue) 
{
  if (curValue > max)
		return max;
	if (curValue < min)
		return min;
	return curValue;
}

double TalonXXIII::Sign(double curValue)
{
  if(curValue < 0.0)
    return (-1.0);
  else
    return (1.0);
}

void TalonXXIII::RobotStartingConfig() 
{
  test->StartingConfig();
  shooter->StartingConfig();
  drive->StartingConfig();
  gatherer->StartingConfig();
  climb->StartingConfig();
}

void TalonXXIII::TestInit() 
{

}

void TalonXXIII::TestPeriodic() 
{

}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<TalonXXIII>();
}
#endif
