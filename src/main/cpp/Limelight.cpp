#include "Common.h"
#include "Limelight.h"
#include <wpi/uv/Loop.h>

/**
 * Overides default constructor and sets limelight network table
 *
 * @param TalonXXIII Main robot
 */
Limelight::Limelight(TalonXXIII* pRobot)
{
    mainRobot = pRobot;
    // TODO add table name
    table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("stream", 2.0);
    
    // table->PutNumber("camMode", 0);
    LocalReset();
}
/**
 * Resets target position and sets the target to not seen
 */
void Limelight::LocalReset()
{
    seesTarget = 0.0;
    targetX = 0.0;
    targetY = 0.0;
    targetL = 0.0;
    rawCameraL = 0.0;
    cameraL = 0.0;
}
/**
 * Updated SmartDashboard with current position of the target if found
 */
void Limelight::UpdateDash()
{
    frc::SmartDashboard::PutBoolean("Target Seen:", TargetVisable());
    frc::SmartDashboard::PutNumber("Target X:", targetX);
    frc::SmartDashboard::PutNumber("Target Y:", targetY);
    frc::SmartDashboard::PutNumber("Target Latency (ms)",targetL);
    frc::SmartDashboard::PutNumber("Limelight Latency (ms)",cameraL);
    
}
/**
 * Check to see if the target is visible
 *
 * @returns Whether the Limelight can see the target
 */
bool Limelight::TargetVisable()
{
    if(seesTarget == 1.0)
    {
        return true;
    }
    return false;
}
/**
 * Gets the position of the target from the NetworkTable
 */
void Limelight::Analyze()
{
    seesTarget = table->GetNumber("tv",0.0);
    targetX = table->GetNumber("tx",0.0);
    targetY = table->GetNumber("ty",0.0);

    nt::NetworkTableEntry latencyEntry;
    latencyEntry = table->GetEntry("tl");
    targetL = latencyEntry.GetDouble(0.0);
    rawCameraL = latencyEntry.GetLastChange();

    // uint64_t currTime;
    // currTime = ;
    // printf("%f, %lld\n",rawCameraL,currTime);
    // cameraL =(rawCameraL - currTime);


}
/**
 * Sets the LED to force on and sets the camera mode to vision processing
 */
void Limelight::TurnOnLED()
{
    table->PutNumber("camMode", 0);
    table->PutNumber("ledMode", 3);
}
/**
 * Turns the LED to force off and sets the camera mode to driver camera
 */
void Limelight::TurnOffLED()
{
    table->PutNumber("camMode", 1);
    table->PutNumber("ledMode", 1);
}
/**
 * Takes two snapshots per second during a match
 */
void Limelight::TakeSnapshot()
{
    
    table->PutNumber("snapshot", 1);
} 

