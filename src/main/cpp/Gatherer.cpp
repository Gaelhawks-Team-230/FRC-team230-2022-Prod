#include "Common.h"
#include "Gatherer.h"

Gatherer::Gatherer(TalonXXIII* pRobot)
{
    mainRobot = pRobot;
    gathererMotor = new frc::PWMTalonSRX(PWM_GATHERER_MOTOR); //motor for spinning the collection/feeding bit
    feederMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_FEEDER_MOTOR); //motor for moving the whole gatherer
    feederMotor->SetNeutralMode(NeutralMode::Brake);
    feederEncoderInput = new frc::DigitalInput(FEEDER_ENCODER); 
    feederEncoder = new frc::DutyCycle(feederEncoderInput);

    feederStatorCurrent.currentLimit = FEEDER_STATOR_LIMIT;
    feederStatorCurrent.enable = true;
    feederMotor->ConfigStatorCurrentLimit(feederStatorCurrent);

    LocalReset();
}

void Gatherer::LocalReset() //used to stop/reset everything on the gatherer
{
    count = 0;
    stage = 0;
    gathererSpeed = 0.0;

    pLoFeeder = GetFeederAbsolutePosition();
    pLoDotFeeder = 0.0;
    pHiFeeder = GetFeederVel();
    pHiDotFeeder = 0.0;
    pHatFeeder = 0.0;
    tauCompFeeder = 0.1;

    feederErrPos = 0.0;
    feederOldCmdPos = LOADING_POS;
    feederCmdPos = LOADING_POS;
    feederCmdPosTJP = feederCmdPos;
    feederCurPos = GetFeederAbsolutePosition();
    feederPosz = feederCurPos;
    feederErrVel = 0.0;
    feederCmdVel = 0.0;
    feederCurVel = GetFeederVel();
    feederErrVelInt = 0.0;
    feederErrVelIntz = 0.0;
    feederCmd = 0.0;
    autoLoadActive = false;
    isControlStart = true;


    SetFeederPosition(LOADING_POS);
}

void Gatherer::StartingConfig() //puts arm in frame
{
    LocalReset();
}

void Gatherer::StopAll() //uses the local reset to stop the robot
{
    LocalReset();
}

void Gatherer::SetFeederPosition(double feederAngle) //used to change the angular position of the feeder arm
{
    feederOldCmdPos = feederCmdPos;
    feederCmdPos = feederAngle;
}

bool Gatherer::IsFeederMoving() //determines whether the feeder arm is in motion
{
    if(fabs(GetFeederVel()) == 0.0)
    {
        return false;
    }
    else
    {
        return true;
    }
}

//feeder positions (collecting- on ground, loading- in frame for starting and loading shooter)
void Gatherer::GathererToFloor()
{
    SetFeederPosition(COLLECTING_POS);
}

void Gatherer::FeederInFrame()
{
    SetFeederPosition(IN_FRAME_POS);
}

void Gatherer::FeedGatherer()
{
    SetFeederPosition(LOADING_POS);
}

void Gatherer::EnableGatherer() //used to start the gatherer to pull in cargo
{
    gathererSpeed = COLLECT_SPEED;
    /*if(mainRobot->shooter->IsLoaded())
    {
        mainRobot->shooter->GiveIndexerCmd(0.0);
    }
    else
    {
        mainRobot->shooter->GiveIndexerCmd(INDEXER_POSITION_CMD);
    }*/
}

void Gatherer::DisableGatherer() //stops gatherer
{
    gathererSpeed = 0.0;
}

void Gatherer::EjectGatherer() //runs gatherer motor in reverse to push cargo out of the robot
{
    gathererSpeed = COLLECT_SPEED * (-1.0);
}

double Gatherer::GetFeederAbsolutePosition() //find where feeder is using the encoder
{
    double pos = ((( ((double)(feederEncoder->GetOutput()) - DMIN/PERIOD)*PERIOD/DMAX - 0.5)*360.0) - (FEEDER_ENCODER_OFFSET));
    if(pos>180.0)
    {
        pos -= 360.0;
    }
    else if(pos<-180.0)
    {
        pos +=360;
    }
    return pos;
}

double Gatherer::GetFeederVel()
{
    /*#define FEEDER_VEL_LIMIT                                (200.0)
    double vel = ((feederCurPos - feederPosz) / LOOPTIME);
    vel = TalonXXIII::Limit(-FEEDER_VEL_LIMIT, FEEDER_VEL_LIMIT, vel);*/
    double falconVel = ((double)feederMotor->GetSelectedSensorVelocity())*(360.0/ENCODER_CNT)*10.0/80.0;
    return falconVel;
}

void Gatherer::AutoLoad() //checks if the autoload service is being run, and if it isn't, begins it.
{
    if(!autoLoadActive)
    {
        autoLoadActive = true;
        stage = 0;
    }
}

bool Gatherer::IsAutoLoad()
{
    return autoLoadActive;
}

void Gatherer::OverrideAutoLoad()
{
    autoLoadActive = false;
    stage = 0;
    DisableGatherer();
    mainRobot->shooter->GiveIndexerCmd(0.0);
}

void Gatherer::AutoLoadService() //autoload service. Used to collect cargo and then load it into the shooter.
{
    switch(stage)
    {
        case 0:
        {
            GathererToFloor();
            mainRobot->shooter->GiveIndexerCmd(INDEXER_POSITION_CMD);
            stage++;
            break;
        }

        case 1:
        {
            EnableGatherer();
            count = 0;
            stage++;
        }
        break;

        case 2: //if CargoLoaded is true, sets it into the loading pos, then switch stages
        {
            if(mainRobot->shooter->IsLoaded() || count == AUTO_LOAD_TIME)
            {
                DisableGatherer();
                FeedGatherer(); // set feeder position to load
                mainRobot->shooter->GiveIndexerCmd(0.0);
                autoLoadActive = false;
                stage = 0;
            }
        }
        break;

    }
}

void Gatherer::UpdateDash()
{
    frc::SmartDashboard::PutNumber("feeder pos", GetFeederAbsolutePosition());
    frc::SmartDashboard::PutNumber("feeder cmd", feederCmd);
}

void Gatherer::GatherManualAdjust(double cmd) //used to allow a human to move the gatherer by themselves
{
    feederCmdPos = cmd;
}

//Called every loop
void Gatherer::Service()
{
    count++;
    if(mainRobot->climb->IsClimbing())
    {
        FeederInFrame();
    }
    feederPosz = feederCurPos;
    feederCurPos = GetFeederAbsolutePosition();
    feederCurVel = GetFeederVel();
    GathererControl();
    if(autoLoadActive)
    {
        AutoLoadService();
    }
    //printf("\npLo: %f, pHi: %f, pHat: %f, encoder: %f", pLoFeeder, pHiFeeder, pHatFeeder, GetFeederAbsolutePosition());
    //printf("\nCount:%d, Goal:%f, CurrPos:%f, PosErr:%f, CurrVel:%f, VelCmd:%f, VelErr:%f, VelInt:%f, Cmd:%f", count, feederCmdPos, feederCurPos, feederErrPos, feederCurVel, feederCmdVel, feederErrVel, feederErrVelInt, feederCmd);
    feederCmd = TalonXXIII::Limit(-FEEDER_CMD_LIMIT, FEEDER_CMD_LIMIT, feederCmd);
    feederMotor->Set(feederCmd);
    gathererSpeed = TalonXXIII::Limit(-GATHER_SPEED_LIMIT, GATHER_SPEED_LIMIT, gathererSpeed);
    gathererMotor->Set(-gathererSpeed);
    //printf("\ngatherCmd:%f, current:%f supply: %f", feederCmd, feederMotor->GetStatorCurrent(), feederMotor->GetSupplyCurrent());
    //printf("\nCount:%d, Goal:%f, CurrPos:%f, PosErr:%f, CurrVel:%f, VelCmd:%f, VelErr:%f, VelInt:%f, Cmd:%f", count, feederCmdPos, feederCurPos, feederErrPos, feederCurVel, feederCmdVel, feederErrVel, feederErrVelInt, feederCmd);
}

void Gatherer::GathererControl()
{
    #define FEEDER_SPEED_LIMIT          (50.0)//lowered from 125 because of broken gatherer
    if(isControlStart)
    {
        pLoFeeder = GetFeederAbsolutePosition();
        pHiFeeder = GetFeederVel();
        feederCmdPosTJP = feederCurPos;
        feederCmdVel = feederCurVel;
        isControlStart = false;
    }

    pLoDotFeeder = 1/tauCompFeeder*(GetFeederAbsolutePosition() - pLoFeeder);
    pLoFeeder = pLoFeeder + LOOPTIME*pLoDotFeeder;
    pHiDotFeeder = 1/tauCompFeeder*(tauCompFeeder*GetFeederVel() - pHiFeeder);
    pHiFeeder = pHiFeeder + LOOPTIME*pHiDotFeeder;
    pHatFeeder = pLoFeeder + pHiFeeder;

    //math for the control system.
    //Error Positions are the difference between where we want to be and where we actually are (want - are = error)
    //feederErrVelInt is the old ErrorVelInt value in addition to the base * height, where base is the looptime, and height is the error value of the velocity.
    //the feederErrVelInt has a limit in order to prevent windup.

    //feederCmdPosTJP = feederCmdPosTJP + TalonXXIII::Limit(-1*(FEEDER_SPEED_LIMIT*LOOPTIME), (FEEDER_SPEED_LIMIT*LOOPTIME), feederCmdPos - feederCmdPosTJP);
    //feederErrPos = feederCmdPosTJP - pHatFeeder; 
    feederErrPos = feederCmdPos - pHatFeeder;
    feederCmdVel = K_POS * feederErrPos;
    feederCmdVel = TalonXXIII::Limit(-FEEDER_SPEED_LIMIT, FEEDER_SPEED_LIMIT, feederCmdVel);
    feederErrVel = feederCmdVel - feederCurVel;
    feederErrVelInt = feederErrVelInt + (feederErrVel * LOOPTIME);
    feederErrVelInt = TalonXXIII::Limit(-K_ROBOT/K_VEL*FEEDER_CMD_LIMIT, K_ROBOT/K_VEL*FEEDER_CMD_LIMIT, feederErrVelInt);
    feederCmd = ((K_VEL / K_ROBOT) * (feederErrVel * TAU + feederErrVelInt));

    /*if((feederOldCmdPos > feederCmdPos) && (feederCmdPos==LOADING_POS))
    {
        feederErrVelInt = -0.4*K_ROBOT/K_VEL;
        feederOldCmdPos = feederCmdPos;
    }*/
    if(GetFeederAbsolutePosition()<35.0)
    {
        feederErrVelInt = fmax(feederErrVelInt, -0.067*K_ROBOT/K_VEL);
        feederCmd = fmax(feederCmd, -0.067);
    }
    /*else if(GetFeederAbsolutePosition()>85.0)
    {
        feederErrVelInt = fmin(feederErrVelInt, 0.0);
        feederCmd = fmin(feederCmd, 0.0);
    }*/
}