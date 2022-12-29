#ifndef GATHERER_H_
#define GATHERER_H_

#include "Common.h"
#include "frc/DigitalInput.h"
#include <frc/motorcontrol/PWMTalonSRX.h>
#include <frc/DutyCycle.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix.h"

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

//position defines 
#define COLLECTING_POS              (88.5)
#define LOADING_POS                 (45.0)
#define IN_FRAME_POS                (10.0)
#define FEEDER_ENCODER_OFFSET       (-157.0)

//Speeds for motion 
#define COLLECT_SPEED           (1.0)
#define GATHER_SPEED_LIMIT      (1.0)
#define FEEDER_CMD_LIMIT        (0.5)

//Control System Constants
#define K_ROBOT                 (500.0)
#define K_POS                   (3.0)
#define K_VEL                   (10.0)
#define TAU                     (0.1)

#define GATHER_TIME             (50)
#define LOAD_TIME               (250)
#define AUTO_LOAD_TIME          (150)
#define FEEDER_STATOR_LIMIT     (40.0)

class TalonXXIII;

class Gatherer
{
    private:
       // Create objects needed by this class
		// example: VictorSP *gathererMotor;

        TalonXXIII *mainRobot;
        frc::DigitalInput *feederEncoderInput;
        frc::DutyCycle *feederEncoder;
        ctre::phoenix::motorcontrol::can::WPI_TalonFX  *feederMotor;
        frc::PWMTalonSRX *gathererMotor;

        ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration feederStatorCurrent;

        //declare member variables
        //example: float height;
        int count;

        bool autoLoadActive; //used to check if the autoload is in use
        double gathererSpeed; //speed fed to gatherer for collection
        double feederAngle; //angle of the feeder arm
        int stage; //for the autoloadservice stages

        //Control System Stuff (delta time will just be LOOPTIME const)
        double feederErrPos; //error position of feeder arm
        double feederCmdPos; //command position of feeder arm
        double feederOldCmdPos;
        double feederCurPos; //where the feeder arm actually is
        double feederPosz; //z is just old position, z is one ago, zz two ago, etc.
        double feederErrVel; //error value of feeder arm's velocity
        double feederCmdVel; //where we want the feeder arm's velocity to be
        double feederCurVel; //what the velocity actually is
        double feederErrVelInt; //Integration
        double feederErrVelIntz; //as above z just means prior
        double feederCmd;
        double feederCmdPosTJP; //Trajectory Planner for the feeder arm 
        bool isControlStart; //checks if control system has been started


        double pLoFeeder;
        double pLoDotFeeder;
        double pHiFeeder;
        double pHiDotFeeder;
        double pHatFeeder;
        double tauCompFeeder;

    public:
        Gatherer(TalonXXIII* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void); //everything off, feeder arm in frame
        void StopAll(void); //stops + resets

        void GathererToFloor(void); //sets feeder to collecting position
        void FeederInFrame(void); //sets feeder to in frame position
        void FeedGatherer(void); //sets feeder to loading position

        void SetFeederPosition(double); //moves the feeder arm to a certain spot
        void EnableGatherer(void); //starts gatherer
        void DisableGatherer(void); //stops gatherer
        void EjectGatherer(void); //starts gatherer in reverse
        bool IsGathererEnabled(void); //checks if gatherer is on
        bool IsFeederMoving(void); //checks if arm is in motion
        double GetFeederAbsolutePosition(void); //uses encoder to find where it is
        double GetFeederVel(void); //finds the velocity of the feeder arm
        void GatherManualAdjust(double); //used to move without automation

        void UpdateDash(void);
        void Service(void);
        void AutoLoadService(void); //used to load and feed the cargo into the shooter
        void AutoLoad(void);
        bool IsAutoLoad(void);
        void OverrideAutoLoad(void);
        void GathererControl(void); //control systems
};
#endif /*GATHERER_H_*/