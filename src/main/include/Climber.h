#ifndef CLIMBER_H_
#define CLIMBER_H_


#include "Common.h"
#include <frc/DutyCycleEncoder.h>
#include <cmath>

#define FULL_EXTEND                         (30.5)
#define PARTIAL_EXTEND                      (21.0)
#define STATIONARY_RETRACT                  (1.5)

#define WINCH_MAX_HEIGHT                    (30.5)
//#define WINCH_MAX_HEIGHT                    (31.0-2.0*PI) for testing
#define WINCH_MIN_HEIGHT                    (0.0)
//#define WINCH_MIN_HEIGHT                    (0.0-2.0*PI) for testing
#define CLIMBER_WINCH_BOTTOM_STOP_HEIGHT    (1.0)

#define WINCH_CMD_LIMIT                     (1.0)
#define TRACK_CMD_LIMIT                     (0.4)

#define TRACK_BATTERY_VOLTAGE               (12.0)
#define TRACK_RESISTANCE                    (TRACK_BATTERY_VOLTAGE/134.0)
#define TRACK_TAU                           (0.05)
#define TRACK_K_VEL                         (8.0)
#define TRACK_K_ROBOT                       (100.0)

#define WINCH_STATOR_CUR_LIMIT              (100.0)
#define WINCH_TAU                           (0.1)
#define WINCH_K_VEL                         (5.0)
#define WINCH_K_ROBOT                       (640.0)

#define WINCH_MANUAL_DELTA                  (0.1)
#define TRACK_MANUAL_DELTA                  (0.1)

#define WINCH_IN_PER_COUNT                  (2.0*PI)

class TalonXXIII;

class Climber
{
    private:
        TalonXXIII *mainRobot;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX *climbWinch; 
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX *climbTrack;
        ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration winchStatorCurrent;

        frc::DutyCycleEncoder *winchEncoder;

        int loopCount;

        double winchCurVel;
        double winchCurPos;
        double winchGoalVel;
        double winchVelCmd;
        double winchVelErr;
        double winchVelErrInt;
        double winchCmd;

        double trackGoalVel;
        double trackCmd;
        double trackCmdz;
        double trackCurVel;
        double trackVelCmd;
        double trackVelErr;
        double trackVelErrInt;

        bool isWinchCalibrating;
        bool isCalDone;
        int calState;

        bool isPartialExtend;
        bool isFinishExtend;
        bool isStationaryRetract;
        bool isExtraExtend;

        int extendMode;
        int oldExtendMode;
       
        int climbCount;
        int climbStage;

    public: 
        Climber(TalonXXIII* pRobot);

        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void ClimbStop(void);
        void UpdateDash(void);
        void Service(void);
        void WinchCalibrate(void);
        void WinchStartCal(void);

        double GetCurWinchVel(void);
        double GetCurWinchPos(void);
        void GiveGoalWinchVel(double);
        bool IsWinchAtGoal(void);
        void ControlWinch(void);

        void GiveGoalTrackVel(double);
        double GetTrackCurrent(void);
        double GetTrackVel(void);
        void ControlTrack(void);
        bool IsClimbing(void);

        void StartAutoPartialExtend(void);
        void AutoPartialExtend(void);

        void StartAutoFinishExtend(void);
        void AutoFinishExtend(void);

        void StartAutoStationaryRetract(void);
        void AutoStationaryRetract(void);

        void AutoFullExtend(int);
        void StopAutoClimb(void);

        void StartExtraExtend(void);
        void EndExtraExtend(void);
};
#endif