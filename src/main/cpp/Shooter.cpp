/*
Shooter.cpp
Diya Patel
January 15, 2022
*/

#include "Common.h"
#include "Shooter.h"

Shooter::Shooter(TalonXXIII* pRobot)
{
    mainRobot = pRobot;
    shooterMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(CAN_SHOOTER_MOTOR);
    rollerMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX(CAN_ROLLER_MOTOR);
    shroudMotor = new frc::PWMTalonSRX(PWM_SHROUD_MOTOR);
    indexerMotor = new frc::PWMTalonSRX(PWM_INDEX_MOTOR);
    shroudEncoderInput = new frc::DigitalInput(SHROUD_ENCODER); 
    shroudEncoder = new frc::DutyCycle(shroudEncoderInput);
    flywheelBeamBreak = new frc::DigitalInput(DIGIN_FLYWHEEL);

    shooterSupplyCurrent.currentLimit = SHOOTER_SUPPLY_CUR_LIMIT;
    shooterSupplyCurrent.enable = true;
    shooterMotor->ConfigSupplyCurrentLimit(shooterSupplyCurrent);
    shooterStatorCurrent.currentLimit = SHOOTER_STATOR_CUR_LIMIT;
    shooterStatorCurrent.enable = true;
    shooterMotor->ConfigStatorCurrentLimit(shooterStatorCurrent);

    rollerSupplyCurrent.currentLimit = ROLLER_SUPPLY_CUR_LIMIT;
    rollerSupplyCurrent.enable = true;
    rollerMotor->ConfigSupplyCurrentLimit(rollerSupplyCurrent);
    rollerStatorCurrent.currentLimit = ROLLER_STATOR_CUR_LIMIT;
    rollerStatorCurrent.enable = true;
    rollerMotor->ConfigStatorCurrentLimit(rollerStatorCurrent);

    localVision = mainRobot->limelight;

    loopCount = 0;

    LocalReset();
}

//Sets initial values for all objects and variables
void Shooter::LocalReset()
{
    ShroudReset();
    ShooterReset();
    RollerReset();
    isTargeting = false;
    seesTarget = false;
    firstTargetLost = false;
    targetYPos = 0.0;
    targetYPosz = 0.0;
    indexerCmd = 0.0;
    targetWatchdog = 0;
    loopCount = 0;
    time = (double)frc::GetTime();
    timez = time;
}

void Shooter::ShroudReset()
{
    shroudCmd = 0.0;

    shroudCurPos = GetShroudPos();
    shroudPosz = shroudCurPos;
    shroudGoalPos = SHROUD_Z1_HIGH_ANGLE;
    shroudPosCmd = 0.0;
    shroudCurVel = 0.0;
    shroudVelCmd = 0.0;
    shroudPosErr = 0.0;
    shroudVelErr = 0.0;
    shroudVelErrInt = 0.0;
    
    isShroudAtGoal = true;
    isShroudControlStart = true;
}

void Shooter::ShooterReset()
{
    shooterCmd = 0.0;
    shooterAccelerLim = 0.0;
    shooterGoalVel = 0.0;
    shooterVelCmd = 0.0;

    shooterCurVel = GetShooterVel();
    shooterCurVelz = shooterCurVel;
    shooterCurVelzz = shooterCurVel;
    shooterCurVelzzz = shooterCurVel;
    shooterVelOutput = shooterCurVel;
    shooterVelOutputz = shooterVelOutput;
    shooterVelOutputzz = shooterVelOutputz;
    shooterVelOutputzzz = shooterVelOutputzz;

    shooterErr = 0.0;
    shooterErrInt = 0.0;
    shooterErrIntz = shooterErrInt;
    shooterErrIntzz = shooterErrInt;
    shooterErrIntzzz = shooterErrInt;
    shooterErrIntzzzz = shooterErrInt;

    errCount = 0;

    isShooterAtGoal = true;
    isShooterControlStart = true;

    shooterWatchdog = 0;

    isCameraActive = true;

}

void Shooter::RollerReset()
{
    rollerCmd = 0.0;
    rollerAccelerLim = 0.0;
    rollerGoalVel = 0.0;
    rollerVelCmd = 0.0;

    rollerCurVel = GetRollerVel();
    rollerCurVelz = rollerCurVel;
    rollerCurVelzz = rollerCurVel;
    rollerCurVelzzz = rollerCurVel;
    rollerVelOutput = rollerCurVel;
    rollerVelOutputz = rollerVelOutput;
    rollerVelOutputzz = rollerVelOutputz;

    rollerErr = 0.0;
    rollerErrInt = 0.0;
    rollerErrIntz = rollerErrInt;
    rollerErrIntzz = rollerErrInt;
    rollerErrIntzzz = rollerErrInt;
    rollerErrIntzzzz = rollerErrInt;

    isRollerAtGoal = true;
}

void Shooter::StopAll()
{
    LocalReset();
}

void Shooter::StartingConfig()
{
    GiveShooterGoalVel(0.0);
    GiveShroudGoalAngle(SHROUD_Z1_HIGH_ANGLE);
    GiveRollerGoalVel(0.0);
}

void Shooter::LoadShooter()
{
    if(!isTargeting && !isTargetingLower)
    {
        GiveShooterGoalVel(SHOOTER_Z2_HIGH_VEL);
        GiveShroudGoalAngle(SHROUD_Z2_HIGH_ANGLE);
        GiveRollerGoalVel(ROLLER_Z2_HIGH_VEL);
    }
    if(IsReadyToShoot())
    {
        GiveIndexerCmd(INDEXER_SHOOT_CMD);
    }
}

void Shooter::PositionCargo()
{
    if(IsLoaded())
    {
        GiveIndexerCmd(0.0);
    }
    else
    {
        GiveIndexerCmd(INDEXER_POSITION_CMD);
    }
}

bool Shooter::IsShooting()
{
    if(indexerCmd!=0.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool Shooter::IsLoaded()
{
    bool flag = false;
    if(!flywheelBeamBreak->Get())
    {
        flag = true;
    }
    return flag;
}

void Shooter::GiveIndexerCmd(double cmd)
{
    indexerCmd = cmd;
}

void Shooter::TargetingUpper()
{
    GetDisToHub();

    if(disToHub >= Z1_DISTANCE && disToHub < Z2_DISTANCE)
    {
        shooterGoalVel = SHOOTER_Z1_HIGH_VEL + (SHOOTER_Z2_HIGH_VEL - SHOOTER_Z1_HIGH_VEL) * (disToHub - Z1_DISTANCE) / (Z2_DISTANCE - Z1_DISTANCE);
        GiveShooterGoalVel(shooterGoalVel);
        
        shroudGoalPos = SHROUD_Z1_HIGH_ANGLE + (SHROUD_Z2_HIGH_ANGLE - SHROUD_Z1_HIGH_ANGLE) * (disToHub - Z1_DISTANCE) / (Z2_DISTANCE - Z1_DISTANCE);
        GiveShroudGoalAngle(shroudGoalPos);

        rollerGoalVel = ROLLER_Z1_HIGH_VEL + (ROLLER_Z2_HIGH_VEL - ROLLER_Z1_HIGH_VEL) * (disToHub - Z1_DISTANCE) / (Z2_DISTANCE - Z1_DISTANCE);
        GiveRollerGoalVel(rollerGoalVel);
    }
    else if(disToHub >= Z2_DISTANCE && disToHub < Z3_DISTANCE)
    {
        shooterGoalVel = SHOOTER_Z2_HIGH_VEL + (SHOOTER_Z3_HIGH_VEL - SHOOTER_Z2_HIGH_VEL) * (disToHub - Z2_DISTANCE) / (Z3_DISTANCE - Z2_DISTANCE);
        GiveShooterGoalVel(shooterGoalVel);
        
        shroudGoalPos = SHROUD_Z2_HIGH_ANGLE + (SHROUD_Z3_HIGH_ANGLE - SHROUD_Z2_HIGH_ANGLE) * (disToHub - Z2_DISTANCE) / (Z3_DISTANCE - Z2_DISTANCE);
        GiveShroudGoalAngle(shroudGoalPos);

        rollerGoalVel = ROLLER_Z2_HIGH_VEL + (ROLLER_Z3_HIGH_VEL - ROLLER_Z2_HIGH_VEL) * (disToHub - Z2_DISTANCE) / (Z3_DISTANCE - Z2_DISTANCE);
        GiveRollerGoalVel(rollerGoalVel);
    }
    else if(disToHub >= Z3_DISTANCE && disToHub < Z4_DISTANCE)
    {
        shooterGoalVel = SHOOTER_Z3_HIGH_VEL + (SHOOTER_Z4_HIGH_VEL - SHOOTER_Z3_HIGH_VEL) * (disToHub - Z3_DISTANCE) / (Z4_DISTANCE - Z3_DISTANCE);
        GiveShooterGoalVel(shooterGoalVel);
        
        shroudGoalPos = SHROUD_Z3_HIGH_ANGLE + (SHROUD_Z4_HIGH_ANGLE - SHROUD_Z3_HIGH_ANGLE) * (disToHub - Z3_DISTANCE) / (Z4_DISTANCE - Z3_DISTANCE);
        GiveShroudGoalAngle(shroudGoalPos);

        rollerGoalVel = ROLLER_Z3_HIGH_VEL + (ROLLER_Z4_HIGH_VEL - ROLLER_Z3_HIGH_VEL) * (disToHub - Z3_DISTANCE) / (Z4_DISTANCE - Z3_DISTANCE);
        GiveRollerGoalVel(rollerGoalVel);
    }
    else if(disToHub >= Z4_DISTANCE && disToHub <= Z5_DISTANCE)
    {
        shooterGoalVel = SHOOTER_Z4_HIGH_VEL + (SHOOTER_Z5_HIGH_VEL - SHOOTER_Z4_HIGH_VEL) * (disToHub - Z4_DISTANCE) / (Z5_DISTANCE - Z4_DISTANCE);
        GiveShooterGoalVel(shooterGoalVel);
        
        shroudGoalPos = SHROUD_Z4_HIGH_ANGLE + (SHROUD_Z5_HIGH_ANGLE - SHROUD_Z4_HIGH_ANGLE) * (disToHub - Z4_DISTANCE) / (Z5_DISTANCE - Z4_DISTANCE);
        GiveShroudGoalAngle(shroudGoalPos);

        rollerGoalVel = ROLLER_Z4_HIGH_VEL + (ROLLER_Z5_HIGH_VEL - ROLLER_Z4_HIGH_VEL) * (disToHub - Z4_DISTANCE) / (Z5_DISTANCE - Z4_DISTANCE);
        GiveRollerGoalVel(rollerGoalVel);
    }
    /*else if(disToHub >= Z5_DISTANCE && disToHub <= Z6_DISTANCE)
    {
        shooterGoalVel = SHOOTER_Z5_HIGH_VEL + (SHOOTER_Z6_HIGH_VEL - SHOOTER_Z5_HIGH_VEL) * (disToHub - Z5_DISTANCE) / (Z6_DISTANCE - Z5_DISTANCE);
        GiveShooterGoalVel(shooterGoalVel);
        
        shroudGoalPos = SHROUD_Z5_HIGH_ANGLE + (SHROUD_Z6_HIGH_ANGLE - SHROUD_Z5_HIGH_ANGLE) * (disToHub - Z5_DISTANCE) / (Z6_DISTANCE - Z5_DISTANCE);
        GiveShroudGoalAngle(shroudGoalPos);

        rollerGoalVel = ROLLER_Z5_HIGH_VEL + (ROLLER_Z6_HIGH_VEL - ROLLER_Z5_HIGH_VEL) * (disToHub - Z5_DISTANCE) / (Z5_DISTANCE - Z4_DISTANCE);
        GiveRollerGoalVel(rollerGoalVel);
    }*/
    //printf("%d, y value: %f, dis: %f, shroud: %f, shooter: %f\n", loopCount, targetYPos, disToHub, shroudGoalPos, shooterGoalVel);
}

void Shooter::TargetingLower()
{
    isTargetingLower = true;
    GiveShooterGoalVel(125.0);
    GiveRollerGoalVel(20.0);
    GiveShroudGoalAngle(75.0);
    isTargeting = true;
    if(IsReadyToShoot())
    {
        GiveIndexerCmd(INDEXER_SHOOT_CMD);
    }
}

bool Shooter::IsLowerShooting()
{
    return isTargetingLower;
}

void Shooter::GetDisToHub()
{
    if(!seesTarget && firstTargetLost)
    {
        firstTargetLost = false;
        targetWatchdog = 0;
    }
    if(!seesTarget && targetWatchdog <= 25)
    {
        targetYPos = targetYPosz;
    }

    targetYPos = TalonXXIII::Limit(LIMELIGHT_Y_Z1, LIMELIGHT_Y_Z7, targetYPos);

    if(targetYPos >= LIMELIGHT_Y_Z1 && targetYPos < LIMELIGHT_Y_Z2)
    {
        disToHub =  DISTANCE_1 + (DISTANCE_2 - DISTANCE_1) * (targetYPos - LIMELIGHT_Y_Z1) / (LIMELIGHT_Y_Z2 - LIMELIGHT_Y_Z1);
    }
    if(targetYPos >= LIMELIGHT_Y_Z2 && targetYPos < LIMELIGHT_Y_Z3)
    {
        disToHub =  DISTANCE_2 + (DISTANCE_3 - DISTANCE_2) * (targetYPos - LIMELIGHT_Y_Z2) / (LIMELIGHT_Y_Z3 - LIMELIGHT_Y_Z2);
    }
    if(targetYPos >= LIMELIGHT_Y_Z3 && targetYPos < LIMELIGHT_Y_Z4)
    {
        disToHub =  DISTANCE_3 + (DISTANCE_4 - DISTANCE_3) * (targetYPos - LIMELIGHT_Y_Z3) / (LIMELIGHT_Y_Z4 - LIMELIGHT_Y_Z3);
    }
    if(targetYPos >= LIMELIGHT_Y_Z4 && targetYPos < LIMELIGHT_Y_Z5)
    {
        disToHub =  DISTANCE_4 + (DISTANCE_5 - DISTANCE_4) * (targetYPos - LIMELIGHT_Y_Z4) / (LIMELIGHT_Y_Z5 - LIMELIGHT_Y_Z4);
    }
    if(targetYPos >= LIMELIGHT_Y_Z5 && targetYPos < LIMELIGHT_Y_Z6)
    {
        disToHub =  DISTANCE_5 + (DISTANCE_6 - DISTANCE_5) * (targetYPos - LIMELIGHT_Y_Z5) / (LIMELIGHT_Y_Z6 - LIMELIGHT_Y_Z5);
    }
    if(targetYPos >= LIMELIGHT_Y_Z6 && targetYPos <= LIMELIGHT_Y_Z7)
    {
        disToHub =  DISTANCE_6 + (DISTANCE_7 - DISTANCE_6) * (targetYPos - LIMELIGHT_Y_Z6) / (LIMELIGHT_Y_Z7 - LIMELIGHT_Y_Z6);
    }

    if(!seesTarget && targetWatchdog > 25)
    {
        disToHub = Z3_DISTANCE;
    }
    if(seesTarget)
    {
        firstTargetLost = true;
        targetYPosz = targetYPos;
    }
}

void Shooter::StartShooterTargeting()
{
    isTargeting = true;
}

void Shooter::StopTargeting()
{
    GiveShooterGoalVel(0.0);
    GiveRollerGoalVel(0.0);
    isTargetingLower = false;
    isTargeting = false;
}

bool Shooter::IsReadyToShoot()
{
    if(IsShooterAtGoal() && IsShooterWaitDone())
    {
        return true;
    }
    return false;
}


bool Shooter::IsShroudAtGoal()
{
    return (fabs((shroudCurPos - shroudGoalPos) < SHROUD_AT_GOAL_ALLOWANCE));
}

double Shooter::GetShroudPos()
{
    double pos = ((( ((double)(shroudEncoder->GetOutput()) - DMIN/PERIOD)*PERIOD/DMAX - 0.5)*360.0) - SHROUD_ENCODER_OFFSET);
    if(pos>180.0)
    {
        pos -= 360.0;
    }
    else if(pos<-180.0)
    {
        pos +=360;
    }
    return -pos;
}

double Shooter::GetShroudVel()
{
    #define SHROUD_VEL_LIMIT                                    (1000.0)
    double vel = ((shroudCurPos - shroudPosz) / LOOPTIME);
    vel = TalonXXIII::Limit(-SHROUD_VEL_LIMIT, SHROUD_VEL_LIMIT, vel);
    return vel;
}

double Shooter::GetShroudGoal()
{
    return shroudGoalPos;
}

void Shooter::ShroudControl()
{
    if(isShroudControlStart)
    {
        shroudPosCmd = shroudCurPos;
        shroudVelCmd = shroudCurVel;
        isShroudControlStart = false;
    }
    shroudPosCmd = shroudPosCmd + TalonXXIII::Limit(-1*(SHROUD_SPEED_LIMIT*LOOPTIME), (SHROUD_SPEED_LIMIT*LOOPTIME), shroudGoalPos - shroudPosCmd);
    shroudPosErr = shroudPosCmd - shroudCurPos;
    shroudVelCmd = SHROUD_K_POS * shroudPosErr;
    shroudVelErr = shroudVelCmd - shroudCurVel;
    shroudVelErrInt = shroudVelErrInt + (shroudVelErr * LOOPTIME);
    shroudVelErrInt = TalonXXIII::Limit(-SHROUD_K_ROBOT/SHROUD_K_VEL, SHROUD_K_ROBOT/SHROUD_K_VEL, shroudVelErrInt);
    shroudCmd = ((SHROUD_K_VEL / SHROUD_K_ROBOT) * (shroudVelErr * SHROUD_TAU + shroudVelErrInt));
    if(disableShroud)//disables shroud while not shooting
    {
        shroudCmd = 0.0;
    }
}

void Shooter::GiveShroudGoalAngle(double goal)
{
    shroudGoalPos = goal;
}

void Shooter::StopShroud()
{
    ShroudReset();
}

bool Shooter::IsShooterAtGoal()
{
    return ((fabs(shooterCurVel - shooterGoalVel) < SHOOTER_AT_GOAL_ALLOWANCE) && shooterGoalVel != 0.0);
}

void Shooter::ShooterWaitTime()
{
    if(IsShooterAtGoal())
    {
        shooterWatchdog++;
    }
    else
    {
        shooterWatchdog = 0;
    }
}

bool Shooter::IsShooterWaitDone()
{
    if(shooterWatchdog >= 25)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double Shooter::GetShooterVel()
{
    return -((double)shooterMotor->GetSelectedSensorVelocity())*(2.0*PI/ENCODER_CNT)*(10.0);
}

double Shooter::GetShooterGoal()
{
    return shooterGoalVel;
}

void Shooter::ShooterControl()
{
    if(isShooterControlStart)//starts from wherever its coasting
    {
        shooterVelCmd = shooterCurVel;
        rollerVelCmd = rollerCurVel;
        isShooterControlStart = false;
    }
    shooterAccelerLim = BATTERY_CURRENT_LIM*K_T_MOTOR/J_WHEEL/(K_V_MOTOR*shooterVelCmd/2/BATTERY_VOLTAGE
     + sqrt(BATTERY_CURRENT_LIM*R_MOTOR/BATTERY_VOLTAGE + 
     (K_V_MOTOR*shooterVelCmd/2/BATTERY_VOLTAGE)*(K_V_MOTOR*shooterVelCmd/2/BATTERY_VOLTAGE)));//helps it get up to speed quicker
     
    shooterVelCmd = shooterVelCmd + TalonXXIII::Limit(-1*(shooterAccelerLim*LOOPTIME), (shooterAccelerLim*LOOPTIME), shooterGoalVel - shooterVelCmd);//traj planner
    shooterErr = shooterVelCmd - shooterVelOutput;

    rollerAccelerLim = BATTERY_CURRENT_LIM*K_T_MOTOR/J_ROLLER_WHEEL/(K_V_MOTOR*rollerVelCmd/2/BATTERY_VOLTAGE
     + sqrt(BATTERY_CURRENT_LIM*R_MOTOR/BATTERY_VOLTAGE + 
     (K_V_MOTOR*rollerVelCmd/2/BATTERY_VOLTAGE)*(K_V_MOTOR*rollerVelCmd/2/BATTERY_VOLTAGE)));//helps it get up to speed quicker
     
    rollerVelCmd = rollerVelCmd + TalonXXIII::Limit(-1*(rollerAccelerLim*LOOPTIME), (rollerAccelerLim*LOOPTIME), rollerGoalVel - rollerVelCmd);//traj planner
    rollerErr = rollerVelCmd - rollerVelOutput;

    if((shooterVelOutput - shooterVelOutputzzz) < SHOOTER_DELTA_VEL)//disturbance model
    {
        errCount = SHOOTER_HOLD_TIME;
        shooterErrInt = shooterErrIntzzzz;
        rollerErrInt = rollerErrIntzzzz;
        //printf("\ndisturbance engaged");
    }
    if(errCount > 0)
    {
        errCount--;
        shooterErrInt = shooterErrIntz = shooterErrIntzz = shooterErrIntzzz = shooterErrIntzzzz;
        rollerErrInt = rollerErrIntz = rollerErrIntzz = rollerErrIntzzz = rollerErrIntzzzz;
    }
    else 
    {
        shooterErrIntzzzz = shooterErrIntzzz;
        shooterErrIntzzz = shooterErrIntzz;
        shooterErrIntzz = shooterErrIntz;
        shooterErrIntz = shooterErrInt;
        shooterErrInt = shooterErrInt + shooterErr*LOOPTIME;

        rollerErrIntzzzz = rollerErrIntzzz;
        rollerErrIntzzz = rollerErrIntzz;
        rollerErrIntzz = rollerErrIntz;
        rollerErrIntz = rollerErrInt;
        rollerErrInt = rollerErrInt + rollerErr*LOOPTIME;
    }

    shooterErrInt = TalonXXIII::Limit(SHOOTER_MIN_VELOCITY_ERROR_INT, SHOOTER_MAX_VELOCITY_ERROR_INT, shooterErrInt);
    rollerErrInt = TalonXXIII::Limit(ROLLER_MIN_VELOCITY_ERROR_INT, ROLLER_MAX_VELOCITY_ERROR_INT, rollerErrInt);

    if(errCount > 0)
    {
        shooterCmd = SHOOTER_K_BANDWIDTH/SHOOTER_K * (SHOOTER_TAU * SHOOTER_BANDWIDTH_MULTIPLIER * shooterErr + shooterErrInt);
        rollerCmd = ROLLER_K_BANDWIDTH/ROLLER_K * (ROLLER_TAU * ROLLER_BANDWIDTH_MULTIPLIER * rollerErr + rollerErrInt);
    }
    else
    {
        shooterCmd = SHOOTER_K_BANDWIDTH/SHOOTER_K * (SHOOTER_TAU * shooterErr + shooterErrInt);
        rollerCmd = ROLLER_K_BANDWIDTH/ROLLER_K * (ROLLER_TAU * rollerErr + rollerErrInt);
    }
    
    if(shooterGoalVel==0.0)//puts it in coast mode
    {
        shooterCmd = 0.0;
        shooterErrInt = 0.0;
        rollerCmd = 0.0;
        rollerErrInt = 0.0;
        isShooterControlStart = true;
    }

    //rollerCmd = 0.5;
    //printf("roller cmd:%f, roller vel:%f\n", rollerCmd, GetRollerVel());

    shooterCurVelzzz = shooterCurVelzz;
    shooterCurVelzz = shooterCurVelz;
    shooterCurVelz = shooterCurVel;
    shooterVelOutputzzz = shooterVelOutputzz;
    shooterVelOutputzz = shooterVelOutputz;
    shooterVelOutputz = shooterVelOutput;

    rollerCurVelzzz = rollerCurVelzz;
    rollerCurVelzz = rollerCurVelz;
    rollerCurVelz = rollerCurVel;
    rollerVelOutputzz = rollerVelOutputz;
    rollerVelOutputz = rollerVelOutput;
    //printf("\n%d, shooter vel: %f, goal: %f, cmd: %f, errInt: %f,\troller vel: %f, goal: %f, cmd: %f", loopCount, shooterCurVel, shooterGoalVel, shooterCmd, shooterErrInt, rollerCurVel, rollerGoalVel, rollerCmd);
}

void Shooter::GiveShooterGoalVel(double goal)
{
    shooterGoalVel = goal;
}

void Shooter::StopShooter()
{
    ShooterReset();
}


bool Shooter::IsRollerAtGoal()
{
    return ((fabs(rollerCurVel - rollerGoalVel) < ROLLER_AT_GOAL_ALLOWANCE) && rollerGoalVel != 0.0);
}

double Shooter::GetRollerVel()
{
    return ((double)rollerMotor->GetSelectedSensorVelocity())*(2.0*PI/ENCODER_CNT)*(10.0)*(30.0/18.0)*(1.0/3.0);
}

double Shooter::GetRollerGoal()
{
    return rollerGoalVel;
}

void Shooter::GiveRollerGoalVel(double cmd)
{
    rollerGoalVel = cmd;
}

void Shooter::StopRoller()
{
    RollerReset();
}

void Shooter::Service()
{   
    loopCount++;
    targetWatchdog++;
    ShooterWaitTime();
    if(mainRobot->climb->IsClimbing())
    {
        disableShroud = true;
    }
    else if(shooterGoalVel!=0.0)
    {
        disableShroud = false;
    }
    seesTarget = localVision->GetTargetVisibility();
    targetYPos = localVision->GetTargetY();
    shroudPosz = shroudCurPos;
    shroudCurPos = GetShroudPos();
    shroudCurVel = GetShroudVel();
    shooterCurVel = GetShooterVel();
    rollerCurVel = GetRollerVel();
    shooterVelOutput = SHOOTER_LEAD_FILTER_C*shooterVelOutputz + SHOOTER_LEAD_FILTER_A*shooterCurVel + SHOOTER_LEAD_FILTER_B*shooterCurVelz;
    rollerVelOutput = ROLLER_LEAD_FILTER_C*rollerVelOutputz + ROLLER_LEAD_FILTER_A*rollerCurVel + ROLLER_LEAD_FILTER_B*rollerCurVelz;
    ShooterControl();
    ShroudControl();
    shroudCmd = TalonXXIII::Limit(-1.0,1.0,shroudCmd);
    shooterCmd = TalonXXIII::Limit(-1.0,1.0,shooterCmd);
    rollerCmd = TalonXXIII::Limit(-1.0, 1.0, rollerCmd);
    indexerCmd = TalonXXIII::Limit(-1.0,1.0,indexerCmd);
    shroudMotor->Set(-shroudCmd);
    shooterMotor->Set(-shooterCmd);
    rollerMotor->Set(rollerCmd);
    indexerMotor->Set(-indexerCmd);
    //printf("y: %f, dis: %f", targetYPos, disToHub);
    //timez = time;
    //time = (double)frc::GetTime();
    //printf("\n%d, Pos: %f, Vel: %f, Cmd: %f", loopCount, shroudCurPos, shroudCurVel, shroudCmd);
    //printf("\n%d, Goal: %f, Err: %f, ErrInt: %f, Cmd: %f, Encoder: %f, Servo: %f", loopCount, shroudGoalPos, shroudErr, shroudErrInt, shroudCmd, shroudCurPos, shroudMotor->GetAngle());
    //printf("\n%d, Goal: %f, Cmd: %f, Encoder: %f, Acceler: %f, Supply: %f, Stator: %f", loopCount, shooterGoalVel, shooterCmd, shooterCurVel, accelerLim, shooterMotor->GetSupplyCurrent(), shooterMotor->GetStatorCurrent());
    //printf("\nRaw: %f, Output: %f", GetShroudPos(), shroudEncoder->GetOutput());
}

bool Shooter::isLimelightTargeting()
{
    isCameraActive = true;
}

void Shooter::UpdateDash()
{
    frc::SmartDashboard::PutBoolean("shooter loaded", IsLoaded());
    frc::SmartDashboard::PutBoolean("ready to shoot", IsReadyToShoot());
    frc::SmartDashboard::PutNumber("shooter vel", shooterCurVel);
    frc::SmartDashboard::PutNumber("shooter goal", shooterGoalVel);
    frc::SmartDashboard::PutNumber("shooter cmd", shooterCmd);
    frc::SmartDashboard::PutNumber("roller vel", rollerCurVel);
    frc::SmartDashboard::PutNumber("roller goal", rollerGoalVel);
    frc::SmartDashboard::PutNumber("roller cmd", rollerCmd);
    frc::SmartDashboard::PutNumber("shroud pos", shroudCurPos);
    frc::SmartDashboard::PutNumber("shroud goal", shroudGoalPos);
    frc::SmartDashboard::PutNumber("shroud cmd", shroudCmd);
    frc::SmartDashboard::PutNumber("limelight y", targetYPos);
    frc::SmartDashboard::PutNumber("dis to hub", disToHub);

}