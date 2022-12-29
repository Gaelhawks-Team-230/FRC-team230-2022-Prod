#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_ 

#include "Common.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/motorcontrol/can/WPI_TalonFX.h"
#include "ctre/phoenix/motorcontrol/TalonFXSensorCollection.h"

//#include "frc/smartdashboard/Smartdashboard.h" 
//#include <frc/Encoder.h>
//#include <frc/DigitalInput.h> 
#include <frc/ADXRS450_Gyro.h>
//#include <frc/AnalogInput.h>
//#include <frc/DutyCycleEncoder.h>
#include <frc/AnalogGyro.h>

// constants here :)
#define WHEEL_CIRCUMFERENCE         (5.0 * PI) // wheel diameter is 6 inches
#define DRIVE_ENCODER_PULSE_COUNT   (2048.0) 
#define DRIVE_GEAR_RATIO            (8.45) 
#define DRIVE_DISTANCE_PER_PULSE    (WHEEL_CIRCUMFERENCE / (DRIVE_ENCODER_PULSE_COUNT * DRIVE_GEAR_RATIO)) 
#define MAX_PERIOD                  (1.0) 

#define ROBOT_K_BANDWIDTH           (12.0) 
#define ROBOT_K                     (780.0) 
#define HIGH_LIMIT                  (ROBOT_K / ROBOT_K_BANDWIDTH) 
#define LOW_LIMIT                   (-HIGH_LIMIT) 
#define ROBOT_TAU                   (0.16) 
#define ROTATE_CONSTANT             (0.5) 
#define TARGETING_GAIN              (5.0)
#define TARGETING_OFFSET            (2.0)
#define COMMAND_RATE_MAX            (200.0) // radians per second
#define MAX_AUTO_ACCELERATION       (120.0 * LOOPTIME) // where LOOPTIME can be found in Common.h and is 20 milliseconds
#define AUTO_ROTATE_NO_GYRO         (0.002) 
#define DRIVE_OMEGA                 (8.0) // the drive omega was commented out in last year's code, but it still included the following line. what is the drive omega? 
#define DRIVE_MAX_ACCEL             (DRIVE_OMEGA / 2) 

#define HEADING_GOAL_ALLOWANCE      (0.1)    

#define DRIVE_ROBOT_K_BANDWIDTH (4.0)
#define DRIVE_ROBOT_K (18.0)
#define DRIVE_HIGH_LIMIT (DRIVE_ROBOT_K/DRIVE_ROBOT_K_BANDWIDTH)
#define DRIVE_LOW_LIMIT (-HIGH_LIMIT)
#define DRIVE_ROBOT_TAU (0.26)

class TalonXXIII; 

class Drivetrain
{
    private: 

        TalonXXIII *mainRobot;

        Limelight *localVision;
        
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontLeftMotor; 
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *frontRightMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *backLeftMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *backRightMotor;
        // these are the four different motors being used 

        frc::ADXRS450_Gyro *gyro;
        double gyroVel;
        double gyroAngle;
        double gyroErr;
        double gyroErrInt;
        double gyroCmd; 
        bool gyroOn;
        double gyroReading;
        double oldGyroReading;

        double postShapingRotate; 
        double leftMotorCmd; // left is multiplied by -1 to be inverted 
        double rightMotorCmd; 

        
        double curVel; 
        double rotate;

        int loopCount; 

        double driveMod; 
        double driveModz; 

        ctre::phoenix::motorcontrol::can::WPI_TalonFX *leftDriveFalcon;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *rightDriveFalcon;

        double leftWheelDisDrive;  
        double rightWheelDisDrive; 
        double averageWheelDisDrive; 
        double oldDriveDis; 
        double currentDriveVel; 
        double leftEncoderOffset; 
        double rightEncoderOffset; 
        double leftRawReading; 
        double rightRawReading; 

        bool seesTarget;
        double targetXPos;
        bool isTargeting;
        double autoTargetingOffest;

        double driveCurErr;
        double driveCurVel;
        double driveErrInt; 

        double targetingCmd;

    public: 
        Drivetrain(TalonXXIII* pRobot); 

        void LocalReset(void); 
        void StartingConfig(void); 
        void StopAll(void); 

        void DriveControl(double driveCmd, double rotateCmd, double forcedDrive, double forcedRotate, bool isAuto, bool isTraj, double velCmd); 

        void UpdateDash(void); 
        void Service(void); 

        void ResetAutoTargetBias(void);
        double GetAutoTargetBias(void);

        void StartDriveTarget(void);
        void StopDriveTarget(void);
        bool IsDriveTargeting(void);
        bool IsRobotShootReady(void);

        void GyroOff(void); 
        void GyroOn(void); 
        double GetGyroVelocity(void);
        double GetGyroAngle(void);
        double GyroControl(double velCmd); 

        double VelControl(double velCmd);
        void ReadDriveEncoders(void);
    
        void InitDriveFalcons(ctre::phoenix::motorcontrol::can::WPI_TalonFX *leftMotor, ctre::phoenix::motorcontrol::can::WPI_TalonFX *rightMotor); 
        inline double GetAverageDriveDis() { return averageWheelDisDrive; }; 
        inline double GetLeftDriveDis() { return leftWheelDisDrive; }; 
        inline double GetRightDriveDis() { return rightWheelDisDrive; }; 
        inline double GetOldDriveDis() { return oldDriveDis; }; 
        inline double GetTargetingCmd() {return targetingCmd; };


        // void ResetDriveEncoders(void); 
        // void ReadDriveEncoders(void); 
};

#endif /*DRIVETRAIN_H_*/
