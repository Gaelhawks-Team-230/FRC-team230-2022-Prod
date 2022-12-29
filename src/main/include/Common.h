#ifndef SRC_COMMON_H_
#define SRC_COMMON_H_

#include "TalonXXIII_main.h"

//#define TEST_DRIVE
//#define FALCONTEST   1
//#define BUTTONTEST   1
//#define SHOOTERTEST		1
//#define USE_GYRO
#define TEST_MODE
//#define USING_SOLENOID
//#define PRACTICE_BOT

/*******************\
 *  Global Values  *
\*******************/
#define PI	 						(3.14159265)
#define RAD_TO_DEG 					(180.0/PI)
#define DEG_TO_RAD					(PI/180.0)
#define ENCODER_CNT                 (2048.0)

#define LOOPTIME                    (0.02)//test .019
#define SAMPLES_PER_SECOND			(1.0/LOOPTIME)
#define N1SEC  						((int) SAMPLES_PER_SECOND)
#define ONE_SEC						(N1SEC)
#define HALF_SEC					((int)(0.5*N1SEC))
#define QUART_SEC                   ((int)(0.25*N1SEC))
#define THREE_QUART_SEC             ((int)(0.75*N1SEC))


//absolute encoder constants
#define DMAX                        (1024.0)
#define DMIN                        (1.0)
#define PERIOD                      (1025.0)

#define INDEXER_POSITION_CMD                  (0.4)

#define TWO_BALL_PATH_1             1
#define TWO_BALL_PATH_2             2
#define THREE_BALL_PATH_1           3
#define THREE_BALL_PATH_2           4
#define FOUR_BALL_PATH_1            5
#define FOUR_BALL_PATH_2            6
// #define FOUR_BALL_PATH_3            7
#define FIVE_BALL_PATH_1            8
#define FIVE_BALL_PATH_2            9
#define TWO_BALL_PATH_3             10



typedef enum
{
	PWM_SHROUD_MOTOR		= 0,
    PWM_GATHERER_MOTOR      = 1,
    PWM_INDEX_MOTOR         = 2,
} pwms;


typedef enum
{
    EXAMPLE_CURRENT			= 11,
} pdp;

typedef enum
{
    CAN_SHOOTER_MOTOR		= 1,
    CAN_FRONT_RIGHT         = 2,
    CAN_FRONT_LEFT          = 3, 
    CAN_BACK_RIGHT          = 4, 
    CAN_BACK_LEFT           = 5,
    CAN_ROLLER_MOTOR        = 9,
    CAN_FEEDER_MOTOR        = 11,
    CAN_CLIMB_WINCH         = 7,
    CAN_CLIMB_TRACK         = 13,
} can;


typedef enum
{
	EXAMPLE_ANALOG		    = 0,
} analogs;

typedef enum
{
    PCM_EXAMPLE 	        = 0,
} pcm;


typedef enum
{
	DIGIN_FLYWHEEL		        = 0,
    SHROUD_ENCODER              = 1,
    FEEDER_ENCODER              = 2,
    DIGIN_GATHERER              = 9,
    WINCH_ENCODER               = 5,
} digitals;

/*
** List of gamepad (USB_GAMEPAD below) button and axis assignments
*/

typedef enum
{
    ENABLE_GYRO_BUTTON 		    = 1,
    CLIMB_MOTION_BUTTON         = 2,
    FCONT_LOW_BUTTON_ONE        = 6,
    FCONT_LOW_BUTTON_TWO        = 7,
    FCONT_TARGET_BUTTON_ONE     = 8,
    FCONT_TARGET_BUTTON_TWO     = 9,
    FCONT_FEED_BUTTON_ONE       = 10,
    FCONT_FEED_BUTTON_TWO       = 11,
    RECAL_BUTTON                = 14,
    CLIMB_EXTRA_EXTEND_BUTTON   = 16,
} joystick_buttons;

typedef enum
{
    SPEED_AXIS					= 1,
	ROTATE_AXIS					= 5,
} joystick_axes;


typedef enum
{
    CLIMB_WINCH_AXIS			= 1,
	CLIMB_TRACK_AXIS			= 3,
} gamepad_axes;

typedef enum
{
    AUTO_LOAD_POV               = 0,
    AUTO_LOAD_STOP_POV          = 180,
} gamepad_pov;

typedef enum
{
    IN_FRAME_BUTTON             = 1,
    LOAD_POS_BUTTON             = 2,
    EJECT_BALL_BUTTON           = 3,
    FLOOR_POS_BUTTON            = 4,
    TARGET_UPPER_BUTTON         = 5,
    GATHER_BALL_BUTTON          = 6,
    TARGET_LOWER_BUTTON         = 7,
    FEED_SHOOTER_BUTTON         = 8,
} gamepad_buttons;

#endif /*Common_H_*/