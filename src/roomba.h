// SENSOR REGISTER VALUE MASK (REPLY TO OPCODE 142 OR QUERY 149 xxx)
#define BUMP_RIGHT          0x01    // bit 0
#define BUMP_LEFT           0x02    // bit 1
#define WHEELDROP_RIGHT     0x04    // bit 2
#define WHEELDROP_LEFT      0x08    // bit 3
#define WHEELDROP_CASTER    0x10    // bit 4
#define BUMP_FRONT          0x03    // Combinaison of bit 0 & bit 1 mask

// BUTTON REGISTER  VALUE (OPCODE 165)
#define BUTTON_CLEAN        0x01    // bit 0
#define BUTTON_SPOT         0x02    // bit 1
#define BUTTON_DOCK         0x04    // bit 2
#define BUTTON_MINUTE       0x08    // bit 3
#define BUTTON_HOUR         0x10    // bit 4
#define BUTTON_DAY          0x20    // bit 5
#define BUTTON_SCHEDULE     0x40    // bit 6
#define BUTTON_CLOCK        0x80    // bit 7

// LEDS CONTROL REGISTER VALUE (OPCODE 139)
#define LED_DEBRIS           0x01    // bit 0
#define LED_SPOT             0x02    // bit 1
#define LED_DOCK             0x04    // bit 2
#define LED_CHECK            0x08    // bit 3
#define LED_CLEAN            0x10    // bit 4
#define LED_BROSS            0x20    // bit 5
                                     // bit 6 N/A
#define LED_OK               0x80    // bit 7

// LEDS SCHEDULE CONTROL REGISTER VALUE (OPCODE 162)
#define LED_SCHEDULE_SUNDAY    0x01    // bit 0
#define LED_SCHEDULE_MONDAY    0x02    // bit 1
#define LED_SCHEDULE_TUESDAY   0x04    // bit 2
#define LED_SCHEDULE_WEDNESDAY 0x08    // bit 3
#define LED_SCHEDULE_THURSDAY  0x10    // bit 4
#define LED_SCHEDULE_FRIDAY    0x20    // bit 5
#define LED_SCHEDULE_SATURDAY  0x40    // bit 6
                                        // bit 7 N/A
#define LED_SCHEDULE_COLON     0x0100    // bit 0
#define LED_SCHEDULE_PM        0x0200    // bit 1
#define LED_SCHEDULE_AM        0x0400    // bit 2
#define LED_SCHEDULE_CLOCK     0x0800    // bit 3
#define LED_SCHEDULE_SCHEDULE  0x1000    // bit 3
#define LED_SCHEDULE_MINUTE    0x2000    // bit 3
#define LED_SCHEDULE_HOUR      0x4000    // bit 3
#define LED_SCHEDULE_DAY       0x8000    // bit 3

// MOTOR ENABLE CONTROL REGISTER VALUE bit 0..2 (OPCODE 138)
#define MOTOR_SIDE_BRUSH          0x01    // bit 0
#define MOTOR_VACCUM              0x02    // bit 1
#define MOTOR_MAIN_BRUSH          0x04    // bit 2
#define MOTOR_SIDE_BRUSH_REVERSE  0x09    // bit 3
#define MOTOR_MAIN_BRUSH_REVERSE  0x14    // bit 4


// VARIOUS DEFINE
#define LEFT         0
#define FRONT_LEFT   1
#define CENTER_LEFT  2
#define CENTER_RIGHT 3
#define FRONT_RIGHT  4
#define RIGHT        5

#define OFF          0
#define ON           1

typedef struct roomba_sensors{
    char bump_right;
    char bump_left;
    char wheeldrop_right;
    char wheeldrop_left;
    char wheeldrop_caster;
    char wall;
    char cliff_left;
    char cliff_frontLeft;
    char cliff_frontRight;
    char cliff_right;
    char virtual_wall;
    char dirtDetector_left;
    char dirtDetector_right;
} RSENSORS;

typedef struct roomba_buttons{
    unsigned char clean;
    unsigned char schedule;
    unsigned char spot;
    unsigned char clock;
    unsigned char dock;
} RBUTTONS;

typedef struct roomba_battery{
    unsigned int voltage;
    int current;
    unsigned int charging_state;
    unsigned int charging_current;
    unsigned int capacity;
} RBATTERY;

typedef struct roomba_mesures{
    char motor_overCurrent;
    int distance_mm;
    int angle_deg;
    RBATTERY battery;

} RMEASURES;

typedef struct Roomba{
    RSENSORS sensor;
    RBUTTONS button;
    RMEASURES measure;
} ROOMBA;


/**
 * @brief This command controls the four 7 segment displays on the Roomba 560 and 570 using ASCII character
 * codes. Because a 7 segment display is not sufficient to display alphabetic characters properly, all
 * characters are an approximation, and not all ASCII codes are implemented.
 * 
 * @param digitStr like "test"
 * @return int 
 */
int Roomba_set_digits_text(char * digitStr);


/**
 * @brief START & FULL MODE
*This command starts the OI. You must always send the Start command before sending any other
*commands to the OI.
*
*This command gives you complete control over Roomba by putting the OI into Full mode, and turning off
*the cliff, wheel-drop and internal charger safety features. That is, in Full mode, Roomba executes any
*command that you send it, even if the internal charger is plugged in, or command triggers a cliff or wheel
*drop condition.
*
*Start Opcode: 128 Data Bytes: 0
*Full Opcode: 132 Data Bytes: 0
 * 
 * @return int 
 */
int Roomba_init(void);


/**
 * @brief This command stops the OI. All streams will stop and the robot will no longer respond to commands.
 * Use this command when you are finished working with the robot.
 * 
 * @return int 
 */
int Roomba_stop(void);

int Roomba_update_gr0_status(ROOMBA * roombaStatus);

/**
 * @brief Define the state of all the rooma LED (LEDS, Schedule, 7 segment display)
 * 
 * @param state (ON (1)/OFF(0))
 * @return int 
 */
int Roomba_set_all_led_state(unsigned char state);

/**
 * @brief This command controls the LEDs common to all models of Roomba 600. The power LED is specified by
 * two data bytes: one for the color and the other for the intensity.
 * 
 * @param ledBit 
 * LED_DEBRIS           0x01    // bit 0
 * LED_SPOT             0x02    // bit 1
 * LED_DOCK             0x04    // bit 2
 * LED_CHECK            0x08    // bit 3
 * LED_CLEAN            0x10    // bit 4
 * LED_BROSS            0x20
 * LED_OK               0x80    // bit 7
 * @param state (ON (1), OFF (0))
 * @return int 
 */
int Roomba_set_led_state(unsigned char ledBit, unsigned char state);

/**
 * @brief This command controls the scheduling LEDs common to all models of Roomba 600. The power LED is specified by
 * two data bytes: one for the color and the other for the intensity.
 * 
 * @param ledBit 
 * LED_SCHEDULE_SUNDAY    0x01    // bit 0
 * LED_SCHEDULE_MONDAY    0x02    // bit 1
 * LED_SCHEDULE_TUESDAY   0x04    // bit 2
 * LED_SCHEDULE_WEDNESDAY 0x08    // bit 3
 * LED_SCHEDULE_THURSDAY  0x10    // bit 4
 * LED_SCHEDULE_FRIDAY    0x20    // bit 5
 * LED_SCHEDULE_SATURDAY  0x40    // bit 6
 * 
 * LED_SCHEDULE_COLON     0x0100    // bit 0
 * LED_SCHEDULE_PM        0x0200    // bit 1
 * LED_SCHEDULE_AM        0x0400    // bit 2
 * LED_SCHEDULE_CLOCK     0x0800    // bit 3
 * LED_SCHEDULE_SCHEDULE  0x1000    // bit 3
 * LED_SCHEDULE_MINUTE    0x2000    // bit 3
 * LED_SCHEDULE_HOUR      0x4000    // bit 3
 * LED_SCHEDULE_DAY       0x8000    // bit 3
 * @param state 
 * @return int 
 */
int Roomba_set_led_schedule_state(int ledBit, unsigned char state);

/**
 * @brief This command lets you ask for a packet. The result is returned once
 * @param packetNb The number of the requiered packet ()
 * @return int packet return value
 */
int Roomba_query_packet(unsigned char packetNb);

/**
 * @brief Lets you control the forward and backward motion of Roomba’s main brush, side brush,
 * and vacuum independently. Motor velocity cannot be controlled with this command, all motors will run at
 * maximum speed when enabled. The main brush and side brush can be run in either direction. The
 * vacuum only runs forward.
 * 
 * @param MotorAndReverseBit 
 * MOTOR_SIDE_BRUSH (XOR) MOTOR_SIDE_BRUSH_REVERSE |
 * MOTOR_MAIN_BRUSH (XOR) MOTOR_MAIN_BRUSH_REVERSE |
 * MOTOR_VACCUM
 * 
 * @param state (ON (1), OFF (0))
 * @return int 
 */
int Roomba_set_motors_state(unsigned char MotorAndReverseBit, unsigned char state);

/**
 * @brief The state of the Roomba buttons are sent as individual bits (0 = button not pressed, 1 = button
*pressed). The day, hour, minute, clock, and scheduling buttons that exist only on Roomba 560 and 570
*will always return 0 on a Roomba 510 or 530 robot.
*Range: 0 – 255
*Buttons Packet ID: 18 Data Bytes: 1, unsigned
 * 
 * @param buttonMask 
 * BUTTON_CLEAN        0x01 | 
 * BUTTON_SPOT         0x02 | 
 * BUTTON_DOCK         0x04 | 
 * BUTTON_SCHEDULE     0x40 | 
 * BUTTON_CLOCK        0x80
 * @return int 
 */
int Roomba_get_button_state(char buttonMask);

/**
 * @brief Left Encoder Counts Packet ID: 43 Data Bytes 2, signed\n 
 * 
*The cumulative number of raw left encoder counts is returned as a signed 16-bit number, high byte first.
*This number will roll over if it passes the max value (at approx. 14.5 meters).
*Range: 0 - 65535 counts
*NOTE: These encoders are square wave, not quadrature, so they rely on the robot’s commanded velocity
*direction to know when to count up/down. So if the robot is trying to drive forward, and you force the
*wheels to spin in reverse, the encoders will count up, (and vice-versa). Additionally, the encoders will
*count up when the commanded velocity is zero and the wheels spin.
*To convert counts to distance, simply do a unit conversion using the equation for circle circumference.
*N counts * (mm in 1 wheel revolution / counts in 1 wheel revolution) = mm
*N counts * (π * 72.0 / 508.8) / 2 = mm
 * 
 * @param encoder [LEFT 0 /RIGHT 1]
 * @return int 
 */
int Roomba_get_encoder(unsigned char encoder);

/**
 * @brief Roomba’s speed controller can only control the velocity of the wheels in steps of about 28.5 mm/s.
* Serial sequence: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
* Available in modes: Safe or Full
* Changes mode to: No Change
* Velocity (-500 – 500 mm/s)
* Radius (-2000 – 2000 mm)
 * 
 * @param velocity +/- [mm/sec]
 * @param radius +/- [mm]
 * @return int 
 */
int Roomba_drive(int velocity, int radius );

/**
 * @brief This command lets you control the forward and backward motion of Roomba’s drive wheels
 * independently. It takes four data bytes, which are interpreted as two 16-bit signed values using two’s
 * complement. The first two bytes specify the velocity of the right wheel in millimeters per second (mm/s),
 * with the high byte sent first. The next two bytes specify the velocity of the left wheel, in the same
 * format. A positive velocity makes that wheel drive forward, while a negative velocity makes it drive
 * backward.
 * 
 * @param rightVelocity [mm/sec]
 * @param leftVelocity [mm/sec]
 * @return int 
 */
int Roomba_drive_direct(int rightVelocity, int leftVelocity);

/**
 * @brief *The state of the bump (0 = no bump, 1 = bump) and wheeldrop
*sensors (0 = wheel up, 1 = wheel dropped) are sent as individual
*bits.
*BUMP_RIGHT          0x01    |
*BUMP_LEFT           0x02    |
*WHEELDROP_RIGHT     0x04    |
*WHEELDROP_LEFT      0x08    |
*WHEELDROP_CASTER    0x10    |
 * 
 * @param sensorMask 
 * @return int 
 */
int Roomba_get_bumpsAndWheeldrops(char sensorMask);

/**
 * @brief Get the strength of the cliff (ground) signal. Returned as an unsigned 16-bit value. (Range: 0-4095)
* @param cliffSensor 
* LEFT         0,
* FRONT_LEFT   1,
* FRONT_RIGHT  4,
* RIGHT        5
* @return int 
*/
int Roomba_get_CliffSignal(unsigned char cliffSensor);

/**
 * @brief The strength of the light bump (wall) signal. Returned as an unsigned 16-bit value. Range: 0-4095
 * 
 * @param wallBumpSensor
* LEFT         0,
* FRONT_LEFT   1,
* CENTER_LEFT  2,
* CENTER_RIGHT 3,
* FRONT_RIGHT  4,
* RIGHT        5
 * @return int 
 */
int Roomba_get_WallBumpSignal(unsigned char wallBumpSensor);

/**
 * @brief The stasis caster sensor returns 1 when the robot is making forward progress and 0 when it is not. It
 * always returns 0 when the robot is turning, driving backward, or not driving. If the stasis wheel is too
 * dirty to be read, a value of 2 is returned. If this happens, remove the stasis wheel and clean it with a
 * damp cloth, then dry it thoroughly before reinstalling the wheel.
 * 
 * @return int 
 */
int Roomba_get_statsis_caster_state(void);