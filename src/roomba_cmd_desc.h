// ------------------------------
// OPCODE DECLARATION FOR ROOMBA
// ------------------------------

#define OPCODE_RESET             7              // This command resets the robot, as if you had removed and reinserted the battery.
#define OPCODE_START             128            // Starts the OI. You must always send the Start command before sending any other commands to the OI.
#define OPCODE_STOP              173            // This command stops the OI. All streams will stop and the robot will no longer respond to commands.

//#define OPCODE_BAUDS             129            // This command sets the baud rate in bits per second (bps) from 0 to 11 (300bds to 115200)

#define OPCODE_SAFE              131
#define OPCODE_FULL              132
#define OPCODE_SLEEP             133
#define OPCODE_SPOT              134
#define OPCODE_CLEAN             135
#define OPCODE_MAX_TIME_CLEAN    136
#define OPCODE_DRIVE             137            // Veloc H (byte 1), Veloc L (byte 2), Radius H (byte 3) Radius L (byte 4) ->  -500 to +500 mm/s and -2000 to 2000 mm Radius
                                                // Straight = 32768 or 32767 = 0x8000 or 0x7FFF, Turn in place clockwise = -1 = 0xFFFF, Turn in place counter-clockwise = 1 = 0x0001
#define OPCODE_MOTOR_STATE       138
#define OPCODE_LED_STATE         139            // LED b (byte 1) bit check, Dock, Spot, Debris AND Color (byte 2) 0.255 Green to Red AND intensity (byte 3)
#define OPCODE_SONG_SELECT       140
#define OPCODE_SONG_PLAY         141
#define OPCODE_SENSOR_STATUS     142
#define OPCODE_SEEK_DOCK         143

#define OPCODE_PWM_MOTORS        144            // PWM main brush (byte 1), side brush (byte 2), vaccum (byte 3)
#define OPCODE_DRIVE_PWM         146        
#define OPCODE_DRIVE_DIRECT      145            // Right Veloc H (byte 1) Right Veloc L (byte 2) Left Veloc H (byte 3) Left Veloc L (byte 4)
#define OPCODE_STREAM            148            // This command starts a stream of data packets. The list of packets requested is sent every 15 ms.
#define OPCODE_STREAM_PAUSE      150            // This command lets you stop and restart the steam without clearing the list of requested packets. (byte 1 ON/OFF)
#define OPCODE_QUERY             149            // This command lets you ask for a list of sensor packets.
#define OPCODE_SCHEDULE_LED_STATE  162          // ON/OFF for Monday..Sunday LEDS (byte 1) Schedule, Clock, AM, PM, Colon: (byte2)
#define OPCODE_7SEG_LED_STATE    163            // 7 Segments for Digit 0 (byte 4), 1 (byte 3), 2 (byte 2), 3 (byte 1) bits G, F, E, D, C, B, A
#define OPCODE_7SEG_LED_ASCII    164            // 7 Segments for Digit 0 (byte 4), 1 (byte 3), 2 (byte 2), 3 (byte 1) in ASCII
#define OPCODE_BUTTON_PUSH       165            // Simulate a button push for Clock, Schedule, Day, Hour, Minute, Dock, Spot, Clean (auto-release after 1/6Sec)
#define OPCODE_SCHEDULING        167            // Schedule programming
#define OPCODE_SET_DAYTIME       168            // Set DAY (byte 1) from 0 Sunday to 6 Saturday HOUR (byte 2) 0..23 MINUTES (byte 3) 0..59

// SENSORS STATUS GROUP SELECTION FOR COMMAND 142

#define GROUP_0             0
#define GROUP_1             1
#define GROUP_2             2
#define GROUP_3             3


// ----------------------------------
// BIT VALUE DEFINITION FOR COMMANDS
// ----------------------------------

// BYTE POSITION IN SENSOR FRAME (REPLY TO OPCODE 142)
#define SENS_GR1_BUMPSWHEELDROP     0
#define SENS_GR1_WALL               1
#define SENS_GR1_CLIFF_LEFT         2
#define SENS_GR1_CLIFF_FRONT_LEFT   3
#define SENS_GR1_CLIFF_FRONT_RIGHT  4
#define SENS_GR1_CLIFF_RIGHT        5
#define SENS_GR1_VIRTUAL_WALL       6
#define SENS_GR1_MOT_OVERCURRENT    7
#define SENS_GR1_DIRT_DETECTOR_LEFT 8
#define SENS_GR1_DIRT_DETECTOR_LEFT 9

#define SENS_GR2_REMOTE_OPCODE      0
#define SENS_GR2_BUTTONS            1
#define SENS_GR2_DISTANCE_HIGH      2
#define SENS_GR2_DISTANCE_LOW       3
#define SENS_GR2_ANGLE_HIGH         4
#define SENS_GR2_ANGLE_LOW          5

#define SENS_GR3_CHARGING_STATE     0
#define SENS_GR3_VOLTAGE_HIGH       1
#define SENS_GR3_VOLTAGE_LOW        2
#define SENS_GR3_CURRENT_HIGH       3
#define SENS_GR3_CURRENT_LOW        4
#define SENS_GR3_TEMPERATURE        5
#define SENS_GR3_CHARGE_HIGH        6
#define SENS_GR3_CHARGE_LOW         7
#define SENS_GR3_CAPACITY_HIGH      8
#define SENS_GR3_CAPACITY_LOW       9


// CHARGING STATE VALUE MASK (REPLY TO OPCODE 142)
#define CHARGING_NOT_CHARGING   0   // No charging
#define CHARGING_RECOVERY       1   // Charging recovery
#define CHARGING_CHARGING       2   // No charging
#define CHARGING_TRICKLE        3   // Trickle charging
#define CHARGING_WAITING        4   // Waiting
#define CHARGING_ERROR          5   // Charging error


// ----------------------------------
// PACKET DEFINITION FOR QUERY COMMAND 149
// ----------------------------------
#define  BUMPS_WHEELDROPS 	7
#define  WALL 				8           // 1 bit , DEPRECATED and replaced by light bumnper (45)
#define  CLIFF_LEFT 		9
#define  CLIFF_FRONT_LEFT 	10
#define  CLIFF_FRONT_RIGHT	11
#define  CLIFF_RIGHT 		12
#define  VIRTUAL_WALL 		13
#define  OVERCURRENTS 		14
#define  DIRT_DETECT 		15
#define  UNUSED 			16
#define  IR_OPCODE 			17
#define  BUTTONS            18
#define  DISTANCE           19
#define  ANGLE              20
#define  CHARGING_STATE     21
#define  VOLTAGE            22
#define  CURRENT            23
#define  TEMPERATURE        24
#define  BATTERY_CHARGE     25
#define  BATTERY_CAPACITY   26
#define  WALL_SIGNAL        27          // 1 bit , DEPRECATED and replaced by Light Bump Right Signal (51)
#define  CLIFF_LEFT_SIGNAL         28
#define  CLIFF_FRONT_LEFT_SIGNAL   29
#define  CLIFF_FRONT_RIGHT_SIGNAL  30
#define  CLIFF_RIGHT_SIGNAL        31
//#define  CLIFF_REAR_RIGHT_SIGNAL   32
//#define  CLIFF_REAR_LEFT_SIGNAL    33
#define  UNUSED             32
#define  UNUSED_             33
#define  CHARGER_AVAILABLE  34
#define  OPEN_INTERFACE     35
#define  SONG_NUMBER        36
#define  SONG_PLAYING       37
#define  OI_STREAM          38
#define  VELOCITY           39
#define  RADIUS             40
#define  VELOCITY_RIGHT     41
#define  VELOCITY_LEFT      42
#define  ENCODER_COUNTS_LEFT        43
#define  ENCODER_COUNTS_RIGHT       44
#define  LIGHT_BUMPER               45
#define  LIGHT_BUMP_LEFT            46
#define  LIGHT_BUMP_FRONT_LEFT      47
#define  LIGHT_BUMP_CENTER_LEFT     48
#define  LIGHT_BUMP_CENTER_RIGHT    49
#define  LIGHT_BUMP_FRONT_RIGHT     50
#define  LIGHT_BUMP_RIGHT           51
#define  IR_OPCODE_LEFT             52
#define  IR_OPCODE_RIGHT            53
#define  LEFT_MOTOR_CURRENT         54
#define  RIGHT_MOTOR_CURRENT        55
#define  MAIN_BRUSH_CURRENT         56
#define  SIDE_BRUSH_CURRENT         57
#define  STASIS_CASTER              58