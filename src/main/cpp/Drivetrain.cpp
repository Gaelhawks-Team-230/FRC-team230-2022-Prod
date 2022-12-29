#include "Common.h"
#include "Drivetrain.h"
#include <iostream>
using namespace std;

Drivetrain::Drivetrain(TalonXXIII* pRobot) 
{
    mainRobot = pRobot; 

    leftDriveFalcon = NULL; 
    rightDriveFalcon = NULL; 


    // gets the front right and left and back right and left motors 
    frontLeftMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_FRONT_LEFT); 
    frontRightMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_FRONT_RIGHT); 
    backLeftMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_BACK_LEFT); 
    backRightMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonFX (CAN_BACK_RIGHT); 


    frontLeftMotor->SetNeutralMode(NeutralMode::Brake); 
    frontRightMotor->SetNeutralMode(NeutralMode::Brake); 
    backLeftMotor->SetNeutralMode(NeutralMode::Brake); 
    backRightMotor->SetNeutralMode(NeutralMode::Brake); 

	gyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);
    gyro->Calibrate();

    InitDriveFalcons(frontLeftMotor, frontRightMotor); 

    mainRobot->userInput->SetFlightControllerAxisParameter(SPEED_AXIS, 0.07, 0.7); 
    mainRobot->userInput->SetFlightControllerAxisParameter(ROTATE_AXIS, 0.07, 1.0);

    localVision = mainRobot->limelight;

    LocalReset(); 
} 


// resets variables 
void Drivetrain::LocalReset() 
{
    isTargeting = false;
    rightMotorCmd = 0.0; 
    leftMotorCmd = 0.0; 

    curVel = 0.0; 
    rotate = 0.0; 

    gyroOn = true;
    gyro->Reset();
    gyroReading = gyro->GetAngle();
    oldGyroReading = gyroReading;
    gyroErrInt = 0.0;
    gyroVel = 0.0;
    gyroAngle = 0.0;
    gyroErr = 0.0;
    gyroErrInt = 0.0;
    gyroCmd = 0.0; 

    driveMod = 0.0; 
    driveModz = 0.0; 

    seesTarget = false;
    targetXPos = 0.0;
    autoTargetingOffest = 0.0;
    targetingCmd = 0.0;

    postShapingRotate = 0.0; 
    leftMotorCmd = 0.0;
    rightMotorCmd = 0.0;
    loopCount = 0;  

    leftEncoderOffset = leftDriveFalcon->GetSelectedSensorPosition();
    rightEncoderOffset = rightDriveFalcon->GetSelectedSensorPosition();
    rightRawReading = 0.0;
    leftRawReading = 0.0;
    leftWheelDisDrive = 0.0;
    rightWheelDisDrive = 0.0;
    averageWheelDisDrive = 0.0;
    driveCurVel = 0.0;
    driveCurErr = 0.0;
    driveErrInt = 0.0; 
    oldDriveDis = 0.0;
}


// holds how the variables should be when the robot starts
void Drivetrain::StartingConfig() 
{

}


// resets variables
void Drivetrain::StopAll() 
{
    LocalReset(); 
}


// enter drive command, rotate command, forced drive, forced rotate, and if in autonomous or not 
/**
 * controls how the robot is driven
 *
 * @param driveCmd the goal drive distance
 * @param rotateCmd the goal rotation in degrees
 * @param forcedDrive how much to drive to reach goal drive distance 
 * @param forcedRotate how much to rotate to reach goal rotation
 * @param isAuto if the game is in autonomous or not 
 * @param isTraj if the robot is being driven with the trajectory planner or not
 * @param velCmd the goal velocity
 * 
 */
void Drivetrain::DriveControl(double driveCmd, double rotateCmd, double forcedDrive, double forcedRotate, bool isAuto, bool isTraj, double velCmd) 
{
    ReadDriveEncoders();
    postShapingRotate = rotateCmd; 
    if(isTargeting && seesTarget)//untested code for targeting, if drive isn't working this may be why
    {
        //printf("isTargeting\n");
        rotateCmd = 0.0;
        forcedRotate = 0.0;
        rotate = GyroControl((targetXPos+TARGETING_OFFSET)*TARGETING_GAIN);//offset is in limelight degrees, not robot degrees
        autoTargetingOffest = autoTargetingOffest+(((targetXPos+TARGETING_OFFSET)*TARGETING_GAIN)*LOOPTIME);
        //printf("autoTargeting %f\n", autoTargetingOffest);
        //printf("\nX pos:%f, cmd:%f, rotate: %f", targetXPos, targetXPos*TARGETING_GAIN, rotate);
    }
    else if(isAuto) 
    {
        //printf("isAuto\n");
        if(gyroOn) 
        {
            rotate = GyroControl(rotateCmd);  
        }
    }
    else 
    {
        if(gyroOn) 
        {
            rotate =  GyroControl(rotateCmd * COMMAND_RATE_MAX ); 
        }
        else 
        {
            rotate = ROTATE_CONSTANT * rotateCmd; 
        }
    }
    
    if(isTraj)
    {
        driveMod = VelControl(velCmd);
    }
    else
    {
        driveMod = driveModz + TalonXXIII::Limit(-DRIVE_MAX_ACCEL, DRIVE_MAX_ACCEL, (driveCmd - driveModz)*DRIVE_OMEGA) * LOOPTIME; // min, max, curValue 
        driveModz = driveMod; // z in driveModz has to do with z-transform 
        driveMod = driveMod + forcedDrive;  
    }
    rotate = rotate + forcedRotate;

    leftMotorCmd = -1.0 * (TalonXXIII::Limit(-1.0, 1.0, (driveMod - rotate))); 
    rightMotorCmd = (TalonXXIII::Limit(-1.0, 1.0, (driveMod + rotate))); 

    frontLeftMotor ->Set(ControlMode::PercentOutput, leftMotorCmd); 
    backLeftMotor->Set(ControlMode::PercentOutput, leftMotorCmd); 
    frontRightMotor->Set(ControlMode::PercentOutput, rightMotorCmd); 
    backRightMotor->Set(ControlMode::PercentOutput, rightMotorCmd); 
    //printf("gyro:%f, cmd:%f, rotate:%f\n",gyro->GetAngle(), rotateCmd, rotate);
    //printf("\nR:%f, L:%f", (double)(frontRightMotor->GetSelectedSensorVelocity()) *(2.0*PI/ENCODER_CNT)*(10), (double)(frontLeftMotor->GetSelectedSensorVelocity()) *(2.0*PI/ENCODER_CNT)*(10));
    loopCount++; 
}

void Drivetrain::GyroOff() 
{
    gyroOn = false; 
    gyroErrInt = 0.0; 
}


void Drivetrain::GyroOn() 
{
    gyroOn = true;
}

double Drivetrain::GetGyroAngle()
{
    double gyroA;
    gyroA = gyro->GetAngle();
    return gyroA;
}

double Drivetrain::GetGyroVelocity()
{
    double gyroV;
    gyroReading = gyro->GetAngle();
    gyroV = (gyroReading - oldGyroReading)/LOOPTIME;
    oldGyroReading = gyroReading;
    //gyroV = TalonXX::Limit(-300.0, 300.0, gyroV);
    return gyroV;
}

// enter goal velocity 
/**
 * uses gyro values modify command
 *
 * @param velCmd velocity command given
 * @return modified velocity 
 */
double Drivetrain::GyroControl(double velCmd) 
{
    gyroErr = velCmd - gyroVel; 
    gyroErrInt = gyroErrInt + (gyroErr * LOOPTIME); 
    gyroErrInt = TalonXXIII::Limit(LOW_LIMIT, HIGH_LIMIT, gyroErrInt); 
    double newCmd = (ROBOT_K_BANDWIDTH/ROBOT_K) * ((gyroErr * ROBOT_TAU) + gyroErrInt); 
    return newCmd; 
}

void Drivetrain::StartDriveTarget()
{
    if(!isTargeting)
    {
        isTargeting = true;
    }
}

void Drivetrain::StopDriveTarget()
{
    isTargeting = false;
    targetingCmd = 0.0;
}

bool Drivetrain::IsDriveTargeting()
{
    return isTargeting;
}

void Drivetrain::ResetAutoTargetBias()
{
    autoTargetingOffest = 0.0;
}

double Drivetrain::GetAutoTargetBias()
{
    return autoTargetingOffest;
}

bool Drivetrain::IsRobotShootReady()
{
    return ((fabs(targetXPos) < HEADING_GOAL_ALLOWANCE));
}

double Drivetrain::VelControl(double driveVelCmd)
{
    driveCurErr = driveVelCmd - driveCurVel;
    driveErrInt = driveErrInt + (driveCurErr*LOOPTIME);
    driveErrInt = TalonXXIII::Limit(DRIVE_LOW_LIMIT, DRIVE_HIGH_LIMIT, driveErrInt);
    double newVelCmd = (DRIVE_ROBOT_K_BANDWIDTH/DRIVE_ROBOT_K)*((driveCurErr*DRIVE_ROBOT_TAU) + driveErrInt);
    //printf("\nErr:%f, velCmd:%f, curVel:%f", driveCurErr, driveVelCmd, driveCurVel);
    return newVelCmd;
}

void Drivetrain::ReadDriveEncoders()
{

    if ((leftDriveFalcon != NULL) && (rightDriveFalcon != NULL))
    {
        leftRawReading = leftDriveFalcon->GetSelectedSensorPosition();
        rightRawReading = rightDriveFalcon->GetSelectedSensorPosition();
        leftWheelDisDrive = -1.0 * (leftRawReading - leftEncoderOffset) * DRIVE_DISTANCE_PER_PULSE;
        rightWheelDisDrive = (rightRawReading - rightEncoderOffset) * DRIVE_DISTANCE_PER_PULSE;
        averageWheelDisDrive = (rightWheelDisDrive + leftWheelDisDrive) / 2;

        driveCurVel = ((averageWheelDisDrive - oldDriveDis) / LOOPTIME)/12.0;
        oldDriveDis = averageWheelDisDrive;
    }
}

// useful for updating the dashboard & testing 
void Drivetrain::UpdateDash() 
{
    frc::SmartDashboard::PutNumber("Drive average drive distance: ", averageWheelDisDrive); 
    frc::SmartDashboard::PutNumber("Drive velocity: ", currentDriveVel); 
    frc::SmartDashboard::PutNumber("Drive gyro reading: ", gyroAngle); 
    frc::SmartDashboard::PutNumber("limelight x: ", targetXPos+TARGETING_OFFSET); 
}


void Drivetrain::Service() 
{
    gyroAngle = GetGyroAngle();
    gyroVel = GetGyroVelocity();

    seesTarget = localVision->GetTargetVisibility();
    targetXPos = localVision->GetTargetX();
}


// initializes the motors 
void Drivetrain::InitDriveFalcons(ctre::phoenix::motorcontrol::can::WPI_TalonFX *leftMotor, ctre::phoenix::motorcontrol::can::WPI_TalonFX *rightMotor) 
{
    leftDriveFalcon = leftMotor; 
    leftDriveFalcon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20);
    rightDriveFalcon = rightMotor; 
    rightDriveFalcon->SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, 20);
}




// notes: 
// some functions such as Service and StartingConfig go unused but remain necessary in every class
// init means initialize or something like that lolol 
// UpdateDash is very useful for testing purposes, such as understanding the robots behavior (in terms of distance, velocity, etc...) 