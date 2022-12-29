#include "Common.h"
#include "Climber.h"



Climber::Climber(TalonXXIII* pRobot)
{
    mainRobot = pRobot;

    climbWinch = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(CAN_CLIMB_WINCH); 
    climbTrack = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX(CAN_CLIMB_TRACK);

    winchEncoder = new frc::DutyCycleEncoder(WINCH_ENCODER);
   
    winchStatorCurrent.currentLimit = WINCH_STATOR_CUR_LIMIT;
    winchStatorCurrent.enable = true;
    climbWinch->ConfigStatorCurrentLimit(winchStatorCurrent);

    climbWinch->SetNeutralMode(NeutralMode::Brake); //motor break declaration
    climbTrack->SetNeutralMode(NeutralMode::Brake);

    mainRobot->userInput->SetGamepadAxisParameter(CLIMB_WINCH_AXIS, 0.10, 0.0); 
    mainRobot->userInput->SetGamepadAxisParameter(CLIMB_TRACK_AXIS, 0.10, 0.0); 

    isWinchCalibrating = false;
    isCalDone = false;
    LocalReset();
}

void Climber::LocalReset()
{ 
    loopCount = 0;

    winchCurVel = GetCurWinchVel(); 
    winchCurPos = GetCurWinchPos();
    winchGoalVel = 0.0;
    winchVelCmd = 0.0;
    winchVelErr = 0.0;
    winchVelErrInt = 0.0;
    winchCmd = 0.0;

    trackGoalVel = 0.0;
    trackCmd = 0.0;
    trackCmdz = 0.0;
    trackCurVel = 0.0;
    trackVelCmd = 0.0;
    trackVelErr = 0.0;
    trackVelErrInt = 0.0;

    isPartialExtend = false;
    isFinishExtend = false;
    isStationaryRetract = false;
    isExtraExtend = false;

    extendMode = 0;
    oldExtendMode = 0;

    calState = 0;
       
    climbCount = 0;
    climbStage = 0;
}

void Climber::StartingConfig()
{
    printf("starting config\n");
    LocalReset();
    isWinchCalibrating = true;
}

void Climber::StopAll()
{
    LocalReset();
}

void Climber::WinchCalibrate()
{
    switch(calState)
    {
        case 0:
        {
            winchStatorCurrent.currentLimit = 15.0;
            winchStatorCurrent.enable = true;
            climbWinch->ConfigStatorCurrentLimit(winchStatorCurrent);
            calState++;
        }
        break;

        case 1:
        {
            GiveGoalWinchVel(50.0);
            //printf("%f, %f\n", climbWinch->GetStatorCurrent(), winchCurVel);
            if((climbWinch->GetStatorCurrent()>10.0) && (fabs(winchCurVel)<5.0))
            {
                calState++;
            }
        }
        break;

        case 2:
        {
            GiveGoalWinchVel(0.0);
            winchEncoder->Reset();
            calState++;
        }
        break;

        case 3:
        {
            GiveGoalWinchVel(-200.0);
            if(GetCurWinchPos()>10.0)
            {
                winchStatorCurrent.currentLimit = WINCH_STATOR_CUR_LIMIT;
                winchStatorCurrent.enable = true;
                climbWinch->ConfigStatorCurrentLimit(winchStatorCurrent);
                printf("\nHere I am");
                calState = 0;
                isWinchCalibrating = false;
                isCalDone = true;
            }
        }
        break;
    }
}

void Climber::WinchStartCal()
{
    isWinchCalibrating = true;
    isCalDone = false;
}

double Climber::GetCurWinchVel()
{
    return ((double)climbWinch->GetSelectedSensorVelocity())*(2.0*PI/ENCODER_CNT)*(10);
}

double Climber::GetCurWinchPos()
{
    return (double)winchEncoder->Get()*WINCH_IN_PER_COUNT;
}

void Climber::GiveGoalWinchVel(double goal)
{
    winchGoalVel = -goal;
}

void Climber::GiveGoalTrackVel(double cmd)
{
    //trackGoalVel = cmd;
    trackCmd = cmd;
}

bool Climber::IsClimbing()
{
    return (fabs(trackCmd)>0.1);
}

void Climber::StartAutoPartialExtend()
{
    isPartialExtend = true;
    isFinishExtend = false;
    isStationaryRetract = false;
}

void Climber::AutoPartialExtend()
{
    if(GetCurWinchPos()>PARTIAL_EXTEND)
    {
        isPartialExtend = false;
        GiveGoalWinchVel(0.0);
    }
    else
    {
        GiveGoalWinchVel(-450.0);
    }
}

void Climber::StartAutoFinishExtend()
{
    if(GetCurWinchPos()>PARTIAL_EXTEND)
    {
        isFinishExtend = true;
        isPartialExtend = false;
        isStationaryRetract = false;
    }
}

void Climber::AutoFinishExtend()
{
    if(GetCurWinchPos()>FULL_EXTEND)
    {
        isFinishExtend = false;
        GiveGoalWinchVel(0.0);
    }
    else
    {
        GiveGoalWinchVel(-450.0);
    }
}

void Climber::StartAutoStationaryRetract()
{
    isStationaryRetract = true;
    isFinishExtend = false;
    isPartialExtend = false;
}

void Climber::AutoStationaryRetract()
{
    if(GetCurWinchPos()<STATIONARY_RETRACT)
    {
        isStationaryRetract = false;
        GiveGoalWinchVel(0.0);
    }
    else
    {
        GiveGoalWinchVel(225.0);
    }
}

void Climber::AutoFullExtend(int mode)
{
    oldExtendMode = extendMode;
    extendMode = mode;
    if(oldExtendMode != 1 && extendMode == 1)
    {
        if(GetCurWinchPos()>PARTIAL_EXTEND)
        {
            StartAutoFinishExtend();
        }
        else
        {
            StartAutoPartialExtend();
        }
    }
    else if(extendMode == -1)
    {
        StartAutoStationaryRetract();
    }
    else if(extendMode == 0)
    {
        GiveGoalWinchVel(0.0);
    }
}

void Climber::StopAutoClimb()
{
    isStationaryRetract = false;
    isFinishExtend = false;
    isPartialExtend = false;
}

double Climber::GetTrackCurrent()
{
    return climbTrack->GetStatorCurrent();
}

double Climber::GetTrackVel()
{
    return ((trackCmdz*TRACK_BATTERY_VOLTAGE - GetTrackCurrent()*TRACK_RESISTANCE)*100.0/TRACK_BATTERY_VOLTAGE);
}

void Climber::ControlTrack()
{
    trackVelCmd = trackGoalVel;
    trackVelErr = trackVelCmd - trackCurVel;
    trackVelErrInt = trackVelErrInt + trackVelErr*LOOPTIME;
    trackVelErrInt = TalonXXIII::Limit(-TRACK_K_ROBOT/TRACK_K_VEL, TRACK_K_ROBOT/TRACK_K_VEL, trackVelErrInt);
    trackCmd = TRACK_K_VEL/TRACK_K_ROBOT*(TRACK_TAU*trackVelErr + trackVelErrInt);
}

void Climber::ControlWinch()
{
    winchVelCmd = winchGoalVel;
    winchVelErr = winchVelCmd - winchCurVel;
    winchVelErrInt = winchVelErrInt + winchVelErr*LOOPTIME;
    winchVelErrInt = TalonXXIII::Limit(-WINCH_K_ROBOT/WINCH_K_VEL*WINCH_CMD_LIMIT, WINCH_K_ROBOT/WINCH_K_VEL*WINCH_CMD_LIMIT, winchVelErrInt);
    winchCmd = WINCH_K_VEL/WINCH_K_ROBOT*(WINCH_TAU*winchVelErr + winchVelErrInt);
}

void Climber::ClimbStop() 
{
    LocalReset();
}

void Climber::StartExtraExtend()
{
    isExtraExtend = true;
}

void Climber::EndExtraExtend()
{
    isExtraExtend = false;
}

void Climber::UpdateDash() 
{ 
    //frc::SmartDashboard::PutNumber("winch unwrapped", winchEncoder->GetAbsolutePosition());
    frc::SmartDashboard::PutNumber("winch pos", GetCurWinchPos());
    frc::SmartDashboard::PutNumber("winch cmd", winchCmd);
    frc::SmartDashboard::PutNumber("track cmd", trackCmd);
}

void Climber::Service()
{
    loopCount++;
    winchCurVel = GetCurWinchVel();
    trackCurVel = GetTrackVel();
    
    if(isWinchCalibrating && !isCalDone)
    {
        WinchCalibrate();
    }

    if(isStationaryRetract)
    {
        AutoStationaryRetract();
    }
    if(isPartialExtend)
    {
        AutoPartialExtend();
    }
    if(isFinishExtend)
    {
        AutoFinishExtend();
    }

    ControlWinch();
    
    if(!isExtraExtend && isCalDone)
    {
        if(GetCurWinchPos() > (WINCH_MAX_HEIGHT))
        {
            winchCmd = fmin(0.0, winchCmd);
            winchVelErrInt = fmin(0.0, winchVelErrInt);
        }
        else if(GetCurWinchPos() < (WINCH_MIN_HEIGHT+0.5))
        {
            winchCmd = fmax(0.0, winchCmd);
            winchVelErrInt = fmax(0.0, winchVelErrInt);
        }
    }
    else if(isCalDone)
    {
        if(GetCurWinchPos() > (WINCH_MAX_HEIGHT)+6.0)
        {
            winchCmd = fmin(0.0, winchCmd);
            winchVelErrInt = fmin(0.0, winchVelErrInt);
        }
        else if(GetCurWinchPos() < (WINCH_MIN_HEIGHT+6.5))
        {
            winchCmd = fmax(0.0, winchCmd);
            winchVelErrInt = fmax(0.0, winchVelErrInt);
        }
    }
    /*if(isCalDone && GetCurWinchPos() > (WINCH_MAX_HEIGHT-0.25))
    {
        winchGoalVel = fmin(0.0, winchGoalVel);
        winchVelErrInt = fmin(0.0, winchVelErrInt);
    }
    else if(isCalDone && GetCurWinchPos() < (WINCH_MIN_HEIGHT+0.5))
    {
        winchGoalVel = fmax(0.0, winchGoalVel);
        winchVelErrInt = fmax(0.0, winchVelErrInt);
    }

    ControlWinch();*/

    winchCmd = TalonXXIII::Limit(-WINCH_CMD_LIMIT, WINCH_CMD_LIMIT, winchCmd);
    trackCmd = TalonXXIII::Limit(-TRACK_CMD_LIMIT, TRACK_CMD_LIMIT, trackCmd);

    climbWinch->Set(ControlMode::PercentOutput, winchCmd);
    climbTrack->Set(ControlMode::PercentOutput, trackCmd);
    trackCmdz = trackCmd;
    /*
    timez = time;
    time = (double)frc::GetTime();
    */
    //printf("\nvel:%f, cmd:%f, current:%f", winchCurVel, winchCmd, winchCurrent);
    //printf("\n%d, cmd:%f, current:%f, Timer: %f", loopCount, trackCmd, GetTrackCurrent(), time-timez);
    //printf("\ncmd:%f, current:%f", trackCmd, GetTrackCurrent());
    //printf("\nvelCmd:%f, curVel:%f, velErrInt:%f, cmd:%f, current:%f", trackVelCmd, trackCurVel, trackVelErrInt, trackCmd, GetTrackCurrent());
}