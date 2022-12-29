#ifndef LIMELIGHT_H_
#define LIMELIGHT_H_

#include "Common.h"
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

class TalonXXIII;

class Limelight
{
    private:
        TalonXXIII *mainRobot;

        std::shared_ptr<nt::NetworkTable> table;
        double seesTarget;
        double targetX;
        double targetY;
        double targetL;
        uint64_t rawCameraL;
        double cameraL;
    public:
        Limelight(TalonXXIII* pRobot);

        void LocalReset(void);
        void UpdateDash(void);
        void Analyze(void);
        void TurnOnLED(void);
        void TurnOffLED(void);
        void TakeSnapshot(void);
        bool TargetVisable(void);
        inline double GetTargetVisibility() { return seesTarget; };
        inline double GetTargetX() { return targetY; };
        inline double GetTargetY() { return targetX; };

};
#endif /*Limelight_H_*/