#include "Common.h"
#include "Autonomous.h"
#include "TalonXXIII_main.h"
#include "TrajPlan.h"


void TalonXXIII::DrivePath(int pathNum)
{   
    isTraj = true;
    if(!trajPlan->IsPathComplete(trajIndex, pathNum))
    {
        std::vector<double> cmds = trajPlan->GetCmds(trajIndex, pathNum);
        velCmd = -cmds[0];
        rotateCmd = cmds[1]*180/PI;
        //printf("velCmd: %f, rotateCmd: %f\n", cmds[0], cmds[1]);
        trajIndex++;
    }    
    else
    {
        velCmd = 0.0;
        rotateCmd = 0.0;
        trajIndex = 0;
        loopCount = 0;
        autoStage++;
    }
}

void TalonXXIII::DoNothing()
{
    isTraj = false;
    velCmd = 0.0; 
    rotateCmd = 0.0;
    autoFile = 0;
    autoStage = 0;
    loopCount = 0;
    trajIndex = 0;
    useTargetRotate = false;
}

void TalonXXIII::OffTarmac()
{
    switch(autoStage)
    {
        case 0:
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoFile = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            autoStage++;
            break;
        
        case 1:
            DrivePath(TWO_BALL_PATH_1);
            break;

        case 2:
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoMode = 0;
            autoFile = 0;
            autoStage = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            break;
    }
}

void TalonXXIII::OneBall()
{
    switch(autoStage)
    {
        
    }
}

void TalonXXIII::TwoBall()
{
    switch(autoStage)
    {
        case 0:
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoFile = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            drive->StopDriveTarget();
            drive->ResetAutoTargetBias();
            gatherer->EnableGatherer();
            autoStage++;
            break;

        case 1:
            if(loopCount>QUART_SEC)
            {
                rotateCmd = 0.0;
                velCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;
        
        case 2:
            DrivePath(TWO_BALL_PATH_1);
            // drive->ResetAutoTargetBias();
            break;

        case 3:
            if(loopCount > QUART_SEC)
            {
                loopCount = 0;
                gatherer->EnableGatherer();
                drive->ResetAutoTargetBias();
                autoStage++;
            }
            break;

        case 4:
            drive->StartDriveTarget();
            DrivePath(TWO_BALL_PATH_2);
            //gatherer->EnableGatherer();
            break;

        case 5:
            if(loopCount < SHOOT_1_2)
            {
                shooter->LoadShooter();
            }
            else
            {
                drive->StopDriveTarget();
                shooter->GiveIndexerCmd(0.0);
                deltaTargetAlignment = drive->GetAutoTargetBias();
                // drive->ResetAutoTargetBias();
                loopCount = 0;
                //printf("targetAlignment: %f \n", deltaTargetAlignment);
                autoStage++;
            }
            break;

        // case 6:
        //     if (loopCount < ONE_SEC)
        //     {
        //         velCmd = 0.0;
        //         rotateCmd = -1*deltaTargetAlignment;
        //         //printf("loopCount: %d, rotateCmd: %f \n", loopCount, rotateCmd);
        //     }
        //     else
        //     {
        //         rotateCmd = 0.0;
        //         velCmd = 0.0;
        //         loopCount = 0;
        //         autoStage++;
        //     }
        //     break;

        default:
            //reset stuff
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoMode = 0;
            autoFile = 0;
            autoStage = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            drive->StopDriveTarget();
            break;
    }
}

void TalonXXIII::FourBall()
{
    switch (autoStage)
    {
        case 6:
            if (loopCount < HALF_SEC)
            {
                velCmd = 0.0; rotateCmd = 188.0;
            }
            else
            {
                velCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                shooter->GiveIndexerCmd(0.0);
                autoStage++;
            }
            break;

        case 7:
            DrivePath(FOUR_BALL_PATH_1);
            break;

        case 8:
            drive->ResetAutoTargetBias();
            drive->StartDriveTarget();
            DrivePath(FOUR_BALL_PATH_2);
            break;

        case 9:
            if(loopCount < SHOOT_3_4)
            {
                shooter->LoadShooter();
            }
            else 
            {
                loopCount = 0;
                drive->StopDriveTarget();
                autoStage++;           
            }
            break;

        case 10:
            //reset stuff
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoMode = 0;
            autoFile = 0;
            autoStage = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            break;
    
        default:
            TwoBall();
    }
}

void TalonXXIII::FiveBall()
{
    switch(autoStage)
    {
        case 6:
            if (loopCount < ONE_SEC)
            {
                velCmd = 0.0; rotateCmd = 127.8 - deltaTargetAlignment;
            }
            else
            {
                velCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 7:
            DrivePath(THREE_BALL_PATH_1);
            if (loopCount < THREE_BALL_TARGET_COUNT)
            {
                deltaTargetAlignment = 0.0;
            }
            else
            {
                gatherer->EnableGatherer();
                drive->StartDriveTarget();
                drive->ResetAutoTargetBias();
            }
            break;

        case 8:
            loopCount = 0;
            autoStage++;
            break;

        case 9:
            if(loopCount < SHOOT_3)
            {
                shooter->LoadShooter();
            }
            else
            {
                drive->StopDriveTarget();
                shooter->GiveIndexerCmd(0.0);
                deltaTargetAlignment = drive->GetAutoTargetBias();
                loopCount = 0;
                autoStage++;
            }
            break;
            
        case 10:
            if (loopCount < HALF_SEC)
            {
                velCmd = 0.0;
                rotateCmd = -2*deltaTargetAlignment;
                //printf("loopCount: %d, rotateCmd: %f \n", loopCount, rotateCmd);
            }
            else
            {
                rotateCmd = 0.0;
                velCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 11:
            //printf("driving\n");
            DrivePath(FIVE_BALL_PATH_1);
            break;

        case 12:
            gatherer->EnableGatherer();
            autoStage++;
            break;

        case 13:
            if (loopCount < HALF_SEC)
            {
                velCmd = 0.0; rotateCmd = 0.0;
            }
            else
            {
                loopCount = 0;
                autoStage++;
            }
            break;

        case 14:
            if (loopCount < RIA_LOAD_TIME)
            {
                velCmd = -2.0; rotateCmd = 0.0;
            }
            else
            {
                velCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 15:
            drive->ResetAutoTargetBias();
            drive->StartDriveTarget();
            // if(loopCount<HALF_SEC)
            // {
            //     double accel = 1.0;
            //     //velCmd = -fmin(accel*loopCount, accel*(HALF_SEC-loopCount));
            //     velCmd = -10.0;
            // }
            // else
            // {
            //     loopCount = 0;
            //     velCmd = 0.0;
            //     autoStage++;
            // }
            DrivePath(FIVE_BALL_PATH_2); //added 14, double deceleration
            break;

        case 16:
            if(loopCount>ONE_SEC)
            {
                rotateCmd = 0.0;
                velCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 17:
            if(loopCount < SHOOT_4_5)
            {
                shooter->LoadShooter();
            }
            else
            {
                loopCount = 0;
                drive->StopDriveTarget();
                autoStage++;
            }
            break;

        case 18:
            //reset stuff
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoMode = 0;
            autoFile = 0;
            autoStage = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            break;

        default:
            TwoBall();
    }
}

void TalonXXIII::TwoBallDefense()
{
    switch(autoStage)
    {
        case 0:
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoFile = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            drive->StopDriveTarget();
            drive->ResetAutoTargetBias();
            shooter->TargetingUpper();
            shooter->StartShooterTargeting();
            gatherer->EnableGatherer();
            autoStage++;
            break;

        case 1:
            if(loopCount>HALF_SEC)
            {
                rotateCmd = 0.0;
                velCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;
        
        case 2:
            DrivePath(TWO_BALL_PATH_1);
            // drive->ResetAutoTargetBias();
            break;

        case 3:
            if(loopCount > QUART_SEC)
            {
                loopCount = 0;
                gatherer->EnableGatherer();
                drive->ResetAutoTargetBias();
                autoStage++;
            }
            break;

        case 4:
            drive->StartDriveTarget();
            DrivePath(TWO_BALL_PATH_2);
            //gatherer->EnableGatherer();
            break;

        case 5:
            if(loopCount < SHOOT_1_2)
            {
                shooter->LoadShooter();
            }
            else
            {
                drive->StopDriveTarget();
                shooter->StopTargeting();
                shooter->GiveIndexerCmd(0.0);
                deltaTargetAlignment = drive->GetAutoTargetBias();
                // drive->ResetAutoTargetBias();
                loopCount = 0;
                //printf("targetAlignment: %f \n", deltaTargetAlignment);
                autoStage++;
            }
            break;

        case 6:
            if (loopCount < HALF_SEC)
            {
                velCmd = 0.0; rotateCmd = 130.0 - deltaTargetAlignment;
            }
            else
            {
                velCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                shooter->GiveIndexerCmd(0.0);
                drive->StopDriveTarget();
                shooter->StopTargeting();
                autoStage++;
            }
            break;

        case 7:
            DrivePath(TWO_BALL_PATH_3);
            break;

        case 8:
            if (loopCount < HALF_SEC)
            {
                velCmd = 0.0; rotateCmd = 180.0;
            }
            else
            {
                velCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                shooter->GiveIndexerCmd(0.0);
                autoStage++;
            }
            break;

        case 9:
            if(loopCount < SHOOT_DEFENSE)
            {
                shooter->TargetingLower();
                shooter->LoadShooter();
            }
            else
            {
                loopCount = 0;
                // drive->StopDriveTarget();
                autoStage++;
            }
            break;

        default:
            isTraj = false;
            velCmd = 0.0; 
            rotateCmd = 0.0;
            autoMode = 0;
            autoFile = 0;
            autoStage = 0;
            loopCount = 0;
            trajIndex = 0;
            useTargetRotate = false;
            drive->StopDriveTarget();
            break;

    }
}

void TalonXXIII::ModeSelection()
{
    mode = (AutoModeChooser->GetSelected());

    if (mode == DO_NOTHING)
    {
        autoMode = 0;
    }
    else if (mode == OFF_TARMAC)
    {
        autoMode = 1;
    }
    else if (mode == ONE_BALL)
    {
        autoMode = 2;
    }
    else if (mode == TWO_BALL)
    {
        autoMode = 3;
    }
    else if (mode == THREE_BALL)
    {
        autoMode = 4;
    }
    else if (mode == FOUR_BALL)
    {
        autoMode = 5;
    }
    else if (mode == FIVE_BALL)
    {
        autoMode = 6;
    }
    else if (mode == TWO_BALL_DEFENSE)
    {
        autoMode = 7;
    }
}