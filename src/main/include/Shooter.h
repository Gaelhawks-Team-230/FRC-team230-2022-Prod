#ifndef SHOOTER_H_
#define SHOOTER_H_
/*
Shooter.h
January 15, 2022
Diya Patel
*/

#include "Common.h"
#include "Limelight.h"
#include <frc/DigitalInput.h>
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/DutyCycle.h>
#include <cmath>

#define BATTERY_VOLTAGE                       (12.0)
#define BATTERY_CURRENT_LIM                   (35.0)
#define NO_LOAD_SPEED                         (6380.0)
#define NO_LOAD_VEL                           (NO_LOAD_SPEED*2.0*PI/60)
#define T_STALL                               (664.0)
#define I_STALL                               (257.0)
#define K_T_MOTOR                             (((T_STALL/I_STALL)/16.0)/12.0)
#define K_V_MOTOR                             (BATTERY_VOLTAGE/NO_LOAD_VEL)
#define R_MOTOR                               (BATTERY_VOLTAGE/I_STALL)
#define M_WHEEL                               ((3.0*19.66/16.0)/32.2)
#define R_WHEEL                               (3.0/12.0)
#define J_WHEEL                               (0.5*M_WHEEL*(R_WHEEL*R_WHEEL))
#define J_ROLLER_WHEEL                        (J_WHEEL/5.0)

#define INDEXER_SHOOT_CMD                     (0.8)

#define Z1_DISTANCE                           (6.0)
#define Z2_DISTANCE                           (10.0)
#define Z3_DISTANCE                           (15.0)
#define Z4_DISTANCE                           (20.0)
#define Z5_DISTANCE                           (25.0)

#define ROLLER_ACCEL                          (150.0)
#define ROLLER_MAX_DELTA_VEL                  (ROLLER_ACCEL * LOOPTIME)
#define ROLLER_HOLD_TIME                      (0.5*N1SEC)
#define ROLLER_K_BANDWIDTH                    (6.0)
#define ROLLER_K                              (307.0)
#define ROLLER_TAU                            (0.08)

#define ROLLER_MIN_VELOCITY                   (-1000.0)
#define ROLLER_MAX_VELOCITY                   (1000.0)
#define ROLLER_MAX_VELOCITY_ERROR_INT         (fabs(ROLLER_K/ROLLER_K_BANDWIDTH))
#define ROLLER_MIN_VELOCITY_ERROR_INT         (-1.0 * ROLLER_MAX_VELOCITY_ERROR_INT)
#define ROLLER_AT_GOAL_ALLOWANCE              (25.0)

#define ROLLER_LEAD_FILTER_A                  (2.0)
#define ROLLER_LEAD_FILTER_B                  (-1.2)
#define ROLLER_LEAD_FILTER_C                  (0.2)
#define ROLLER_BANDWIDTH_MULTIPLIER           (1.0)
#define ROLLER_DELTA_VEL                      (-15.0)

#define ROLLER_SUPPLY_CUR_LIMIT               (60.0)
#define ROLLER_STATOR_CUR_LIMIT               (100.0)

#define SHROUD_AT_GOAL_ALLOWANCE              (0.5)

#define SHROUD_K_ROBOT                        (300.0)
#define SHROUD_K_VEL                          (8.0)
#define SHROUD_K_POS                          (4.0)
#define SHROUD_TAU                            (0.1)
#define SHROUD_ENCODER_OFFSET                 (87.0)
#define SHROUD_SPEED_LIMIT                    (200.0)

#define SHOOTER_SUPPLY_CUR_LIMIT              (60.0)
#define SHOOTER_STATOR_CUR_LIMIT              (100.0)

#define SHOOTER_ACCEL                         (150.0)
#define SHOOTER_MAX_DELTA_VEL                 (SHOOTER_ACCEL * LOOPTIME)
#define SHOOTER_HOLD_TIME                     (0.5*N1SEC)
#define SHOOTER_K_BANDWIDTH                   (8.0)
#define SHOOTER_K                             (640.0)
#define SHOOTER_TAU                           (0.75)

#define SHOOTER_MIN_VELOCITY                  (-1000.0)
#define SHOOTER_MAX_VELOCITY                  (1000.0)
#define SHOOTER_MAX_VELOCITY_ERROR_INT        (fabs(SHOOTER_K/SHOOTER_K_BANDWIDTH))
#define SHOOTER_MIN_VELOCITY_ERROR_INT        (-1.0 * SHOOTER_MAX_VELOCITY_ERROR_INT)
#define SHOOTER_AT_GOAL_ALLOWANCE             (25.0)

#define SHOOTER_LEAD_FILTER_A                 (2.0)
#define SHOOTER_LEAD_FILTER_B                 (-1.2)
#define SHOOTER_LEAD_FILTER_C                 (0.2)
#define SHOOTER_BANDWIDTH_MULTIPLIER          (1.0)
#define SHOOTER_DELTA_VEL                     (-7.0)
//#define SHOOTER_DELTA_VEL                     (-15.0)

#define SHOOTING_GAIN                         (0.6 * 1.04)
#define ROLLER_GAIN                           (1.0)
#define SHROUD_BIAS                           (-5.0)

#define SHROUD_Z1_HIGH_ANGLE                  (82.0+SHROUD_BIAS)
#define SHROUD_Z2_HIGH_ANGLE                  (71.0+SHROUD_BIAS)
#define SHROUD_Z3_HIGH_ANGLE                  (64.0+SHROUD_BIAS)
#define SHROUD_Z4_HIGH_ANGLE                  (57.0+SHROUD_BIAS)
#define SHROUD_Z5_HIGH_ANGLE                  (55.0+SHROUD_BIAS)

#define ROLLER_Z1_HIGH_VEL                    (240.0*SHOOTING_GAIN*ROLLER_GAIN)
#define ROLLER_Z2_HIGH_VEL                    (256.0*SHOOTING_GAIN*ROLLER_GAIN)
#define ROLLER_Z3_HIGH_VEL                    (295.0*SHOOTING_GAIN*ROLLER_GAIN)
//#define ROLLER_Z4_HIGH_VEL                    (330.0*SHOOTING_GAIN*ROLLER_GAIN)
#define ROLLER_Z4_HIGH_VEL                    (310.0*SHOOTING_GAIN*ROLLER_GAIN)
#define ROLLER_Z5_HIGH_VEL                    (400.0*SHOOTING_GAIN*ROLLER_GAIN)

#define SHOOTER_Z1_HIGH_VEL                   (240.0*SHOOTING_GAIN)
#define SHOOTER_Z2_HIGH_VEL                   (256.0*SHOOTING_GAIN)
#define SHOOTER_Z3_HIGH_VEL                   (295.0*SHOOTING_GAIN)
//#define SHOOTER_Z4_HIGH_VEL                   (330.0*SHOOTING_GAIN)
#define SHOOTER_Z4_HIGH_VEL                   (360.0*SHOOTING_GAIN)
#define SHOOTER_Z5_HIGH_VEL                   (460.0*SHOOTING_GAIN)

//Limelight 1
// #define LIMELIGHT_Y_Z1                        (-17.2)
// #define LIMELIGHT_Y_Z2                        (-5.8)
// #define LIMELIGHT_Y_Z3                        (2.9)
// #define LIMELIGHT_Y_Z4                        (9.6)
// #define LIMELIGHT_Y_Z5                        (14.9)
// #define LIMELIGHT_Y_Z6                        (21.0)
// #define LIMELIGHT_Y_Z7                        (26.3)

//limelight 2
#define LIMELIGHT_Y_Z1                        (-14.26)
#define LIMELIGHT_Y_Z2                        (-2.60)
#define LIMELIGHT_Y_Z3                        (5.32)
#define LIMELIGHT_Y_Z4                        (11.20)
#define LIMELIGHT_Y_Z5                        (17.76)
#define LIMELIGHT_Y_Z6                        (23.59)
#define LIMELIGHT_Y_Z7                        (28.18)

#define DISTANCE_1                            (6.0)
#define DISTANCE_2                            (8.0)
#define DISTANCE_3                            (10.0)
#define DISTANCE_4                            (12.0)
#define DISTANCE_5                            (15.0)
#define DISTANCE_6                            (19.0)
#define DISTANCE_7                            (25.0)


class TalonXXIII;

class Shooter
{
    private:
        TalonXXIII *mainRobot;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *shooterMotor;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *rollerMotor;
        frc::PWMTalonSRX *shroudMotor;
        frc::PWMTalonSRX *indexerMotor;
        frc::DigitalInput *shroudEncoderInput;
        frc::DutyCycle *shroudEncoder;
        frc::DigitalInput *flywheelBeamBreak;

        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration shooterSupplyCurrent;//limits supplu going to and in shooter motor
        ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration shooterStatorCurrent;

        ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration rollerSupplyCurrent;//limits supplu going to and in roller motor
        ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration rollerStatorCurrent;

        Limelight *localVision;

        int loopCount;
        double time;
        double timez;
        
        bool isTargeting;
        bool isTargetingLower;
        bool seesTarget;
        double targetYPos;
        double targetYPosz;
        double disToHub;
        bool firstTargetLost;
        int targetWatchdog;

        double indexerCmd;
        
        double shroudCmd;
        double shroudCurPos;
        double shroudPosz;
        double shroudCurVel;
        double shroudVelCmd;
        double shroudGoalPos;
        double shroudPosCmd;
        double shroudPosErr;
        double shroudVelErr;
        double shroudVelErrInt;

        bool isShroudAtGoal;
        bool isShroudControlStart;
        bool disableShroud;

        double shooterCmd;
        double shooterAccelerLim;
        double shooterCurVel;
        double shooterCurVelz;
        double shooterCurVelzz;
        double shooterCurVelzzz;
        double shooterGoalVel;

        int shooterWatchdog;

        double shooterVelOutput;
        double shooterVelOutputz;
        double shooterVelOutputzz;
        double shooterVelOutputzzz;

        double shooterVelCmd;
        double shooterErr;
        double shooterErrInt;
        double shooterErrIntz;
        double shooterErrIntzz;
        double shooterErrIntzzz;
        double shooterErrIntzzzz;
        bool isShooterAtGoal;
        bool isShooterControlStart;

        int errCount;

        double rollerCmd;
        double rollerAccelerLim;
        double rollerCurVel;
        double rollerCurVelz;
        double rollerCurVelzz;
        double rollerCurVelzzz;
        double rollerGoalVel;

        double rollerVelOutput;
        double rollerVelOutputz;
        double rollerVelOutputzz;

        double rollerVelCmd;
        double rollerErr;
        double rollerErrInt;
        double rollerErrIntz;
        double rollerErrIntzz;
        double rollerErrIntzzz;
        double rollerErrIntzzzz;
        bool isRollerAtGoal;


    public:
        Shooter(TalonXXIII* pRobot);

        void LocalReset(void);
        void ShroudReset(void);//reset shroud variables
        void ShooterReset(void);//reset shooter variables
        void RollerReset(void);//reset roller variables
        void StopAll(void);
        void StartingConfig(void);
        void Service(void);
        void UpdateDash(void);

        void LoadShooter(void);//sends ball up track to flywheel
        void PositionCargo(void);//puts cargo in correct pos
        bool IsLoaded(void);//beam break right before flywheel
        void GiveIndexerCmd(double);//gives indexer cmd
        bool IsShooting(void);//returns true if indexer is moving

        void TargetingUpper(void);//target upper hub
        void TargetingLower(void);//target lower hub
        void GetDisToHub(void);//get distance to hub from limelight y value
        void StartShooterTargeting(void);
        void StopTargeting(void);//stop all targeting
        bool IsReadyToShoot(void);//checks if all target including drivetrain is done and ready to shoot
        bool IsLowerShooting(void);//returns true if targeting lower
        
        bool IsShroudAtGoal(void);//returns true if at goal pos
        double GetShroudPos(void);//returns shroud pos from encoder
        double GetShroudVel(void);//returns calculated shroud vel
        double GetShroudGoal(void);//returns shroud goal pos
        void ShroudControl(void);
        void ShroudCalibrate(void);//calibrate shroud
        void GiveShroudGoalAngle(double);//sets goal angle
        void StopShroud(void);//stops movement

        bool IsShooterAtGoal(void);//true if at goal vel
        void ShooterWaitTime(void);//waits half sec once shooter at goal
        bool IsShooterWaitDone(void);//returns true if it has waited half sec
        double GetShooterVel(void);//gets vel from encoder
        double GetShooterGoal(void);//returns goal vel
        void ShooterControl(void);
        void GiveShooterGoalVel(double);//set goal vel
        void StopShooter(void);//stops shooter

        bool IsRollerAtGoal(void);//true if at goal vel
        double GetRollerVel(void);//gets vel from encoder
        double GetRollerGoal(void);//returns goal vel
        void GiveRollerGoalVel(double);//set goal vel
        void StopRoller(void);//stops roller

};
#endif /*Shooter_H_*/