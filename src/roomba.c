
#include "roomba_cmd_desc.h"
#include "roomba.h"
#include "serial.h"

//#define ROOMBA_DEBUG

char roombaCommand[32];
unsigned char roombaResponse[128];

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
int Roomba_init(void){

    serial_init("COM10", 115200);                   // Init serial port

    // Loading start sequence bytes
    roombaCommand[0] = OPCODE_START;
    roombaCommand[1] = OPCODE_FULL;

    serial_write(roombaCommand, 2);                 // Send init Sequence
}

/**
 * @brief This command stops the OI. All streams will stop and the robot will no longer respond to commands.
 * Use this command when you are finished working with the robot.
 * 
 * @return int 
 */
int Roomba_stop(void){

    // Loading start sequence bytes
    roombaCommand[0] = OPCODE_STOP;

    serial_write(roombaCommand, 1);                 // Send init Sequence

    return (-1);
}

int Roomba_power(unsigned char State){

}

int Roomba_update_gr0_status(ROOMBA * roombaStatus){
    int byteReceived;

    roombaCommand[0] = OPCODE_SENSOR_STATUS;
    roombaCommand[1] = GROUP_0;                     // Group 0 corresponding to full status (26 byte)
    serial_write(roombaCommand, 2);                 // Send opcode for status

    byteReceived=serial_read(roombaResponse);
    
    #ifdef ROOMBA_DEBUG
        printf("RECEIVED DATA FROM ROOMBA [%d]: ", byteReceived);
        int index = 0;
        for (index = 0; index < byteReceived; ++index){
            printf("|0x%.2x", roombaResponse[index]);
        }
        printf_s("\n\n");
    #endif

    // Get button state
    roombaStatus->button.clean = roombaResponse[SENS_GR2_BUTTONS+10] & BUTTON_CLEAN;
    roombaStatus->button.schedule = roombaResponse[SENS_GR2_BUTTONS+10] & BUTTON_SCHEDULE;
    roombaStatus->button.spot = roombaResponse[SENS_GR2_BUTTONS+10] & BUTTON_SPOT;
    roombaStatus->button.clock = roombaResponse[SENS_GR2_BUTTONS+10] & BUTTON_CLOCK;
    roombaStatus->button.dock = roombaResponse[SENS_GR2_BUTTONS+10] & BUTTON_DOCK;
    
    roombaStatus->sensor.bump_left = roombaResponse[SENS_GR1_BUMPSWHEELDROP] & BUMP_LEFT;
    roombaStatus->sensor.bump_right = roombaResponse[SENS_GR1_BUMPSWHEELDROP] & BUMP_RIGHT;
    roombaStatus->sensor.wheeldrop_left = roombaResponse[SENS_GR1_BUMPSWHEELDROP] & WHEELDROP_LEFT;
    roombaStatus->sensor.wheeldrop_right = roombaResponse[SENS_GR1_BUMPSWHEELDROP] & WHEELDROP_RIGHT;
    roombaStatus->sensor.wheeldrop_caster = roombaResponse[SENS_GR1_BUMPSWHEELDROP] & WHEELDROP_CASTER;

    roombaStatus->measure.battery.voltage = (roombaResponse[SENS_GR3_VOLTAGE_HIGH + 16] << 8);
    roombaStatus->measure.battery.voltage = (roombaStatus->measure.battery.voltage & 0xFF00) | roombaResponse[SENS_GR3_VOLTAGE_LOW + 16];

    roombaStatus->measure.battery.capacity = (roombaResponse[SENS_GR3_CAPACITY_HIGH + 16] << 8);
    roombaStatus->measure.battery.capacity = (roombaStatus->measure.battery.capacity & 0xFF00) | roombaResponse[SENS_GR3_CAPACITY_LOW + 16];

    roombaStatus->measure.battery.current = (char)(roombaResponse[SENS_GR3_CURRENT_HIGH + 16] << 8);
    roombaStatus->measure.battery.current = (char)(roombaStatus->measure.battery.current & 0xFF00) |  roombaResponse[SENS_GR3_CURRENT_LOW + 16];

    roombaStatus->measure.battery.charging_current = (roombaResponse[SENS_GR3_CHARGE_HIGH + 16] << 8);
    roombaStatus->measure.battery.charging_current = (roombaStatus->measure.battery.charging_current & 0xFF00) | roombaResponse[SENS_GR3_CHARGE_LOW + 16];

    roombaStatus->measure.battery.charging_state = roombaResponse[SENS_GR3_CHARGING_STATE + 16];

    roombaStatus->measure.distance_mm = (roombaResponse[SENS_GR2_DISTANCE_HIGH + 16] << 8);
    roombaStatus->measure.distance_mm = (roombaStatus->measure.distance_mm & 0xFF00) | roombaResponse[SENS_GR2_DISTANCE_LOW + 16];
    return 0;
}

/**
 * @brief Define the state of all the rooma LED (LEDS, Schedule, 7 segment display)
 * 
 * @param state (ON (1)/OFF(0))
 * @return int 
 */

int Roomba_set_all_led_state(unsigned char state){
    unsigned char dataValue = 0;

    // Set all byte to 1 for leds ON
    if(state)
        dataValue = 0xFF;
    else dataValue = 0x00;

    roombaCommand[0] = OPCODE_LED_STATE;
    roombaCommand[1] = dataValue;
    roombaCommand[2] = dataValue;                     // Group 0 corresponding to full status (26 byte)
    roombaCommand[3] = dataValue;                     // Group 0 corresponding to full status (26 byte)
    serial_write(roombaCommand, 4);                   // Send opcode for LED command

    roombaCommand[0] = OPCODE_SCHEDULE_LED_STATE;
    roombaCommand[1] = dataValue;
    roombaCommand[2] = dataValue;                     // Group 0 corresponding to full status (26 byte)
    roombaCommand[3] = dataValue;                     // Group 0 corresponding to full status (26 byte)
    serial_write(roombaCommand, 4);                   // Send opcode for SCHEDULE LED command

    // 7seg LED state are inverted
    roombaCommand[0] = OPCODE_7SEG_LED_STATE;
    roombaCommand[1] = ~dataValue;
    roombaCommand[2] = ~dataValue;
    roombaCommand[3] = ~dataValue;
    roombaCommand[4] = ~dataValue;
    serial_write(roombaCommand, 5);                   // Send opcode for 7Seg
}

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
int Roomba_set_led_state(unsigned char ledBit, unsigned char state){
    unsigned char dataValue = 0;

    // Set all byte to 1 for leds ON
    if(state)
        dataValue |= ledBit;
    else 
        dataValue &= ~ledBit;

    roombaCommand[0] = OPCODE_LED_STATE;
    roombaCommand[1] = dataValue;
    roombaCommand[2] = 0;                            // Led color 0..255
    roombaCommand[3] = 0xFF;                         // Intensity

    serial_write(roombaCommand, 4);                  // Send opcode for LED command
    return (-1);
}

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

int Roomba_set_motors_state(unsigned char MotorAndReverseBit, unsigned char state){
    unsigned char dataValue = 0;

    // Set all byte to 1 for leds ON
    if(state)
        dataValue |= MotorAndReverseBit;
    else
        dataValue &= ~MotorAndReverseBit;

    roombaCommand[0] = OPCODE_MOTOR_STATE;
    roombaCommand[1] = dataValue;
                        
    serial_write(roombaCommand, 2);                  // Send opcode for LED command

    return (-1);
}

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

int Roomba_set_led_schedule_state(int ledBit, unsigned char state){
    int dataValueH = 0;
    int dataValueL = 0;

    // Set all byte to 1 for leds ON
    if(state){
        dataValueL |= (ledBit & 0x00FF);
        dataValueH |= (ledBit & 0xFF00) >> 8;
    }
    else{
        dataValueL &= (~ledBit & 0x00FF);
        dataValueH &= (~ledBit & 0xFF00) >> 8;
    }

    roombaCommand[0] = OPCODE_SCHEDULE_LED_STATE;
    roombaCommand[1] = dataValueL;
    roombaCommand[2] = dataValueH;                            
                        
    serial_write(roombaCommand, 3);                  // Send opcode for LED command
    return (-1);
}

/**
 * @brief This command controls the four 7 segment displays on the Roomba 560 and 570 using ASCII character
 * codes. Because a 7 segment display is not sufficient to display alphabetic characters properly, all
 * characters are an approximation, and not all ASCII codes are implemented.
 * 
 * @param digitStr like "test"
 * @return int 
 */

int Roomba_set_digits_text(char * digitStr){
    char digitValues[4] = {' ', ' ', ' ', ' '};
    int lenght=0;

    lenght = strlen(digitStr);
    if(lenght > 4)
        lenght = 4;

    strncpy (digitValues, digitStr, lenght);        
    
    roombaCommand[0] = OPCODE_7SEG_LED_ASCII;
    roombaCommand[1] = digitValues[0];
    roombaCommand[2] = digitValues[1];                            
    roombaCommand[3] = digitValues[2]; 
    roombaCommand[4] = digitValues[3]; 
                        
    serial_write(roombaCommand, 5);                  // Send opcode for LED command
}


/**
 * @brief Roomba’s speed controller can only control the velocity of the wheels in steps of about 28.5 mm/s.
* Serial sequence: [137] [Velocity high byte] [Velocity low byte] [Radius high byte] [Radius low byte]
* Available in modes: Safe or Full
* Changes mode to: No Change
* Velocity (-500 – 500 mm/s)
* Radius (-2000 – 2000 mm)
 * 
 * @param velocity [mm/sec]
 * @param radius [mm]
 * @return int 
 */
int Roomba_drive(int velocity, int radius ){
    if(velocity < -500)
        velocity = -500;
    else if(velocity > 500)
            velocity = 500;

    if(radius < -2000)
        radius = -2000;
    else if(radius > 2000)
            radius = 2000;   

    // Drive Roomba velocity from -500 to 500mmS and radius from -2000 to +2000mm
    roombaCommand[0] = OPCODE_DRIVE;
    roombaCommand[1] = (velocity & 0xFF00) >> 8;
    roombaCommand[2] = velocity & 0xFF;
    roombaCommand[3] = (radius & 0xFF00) >> 8;
    roombaCommand[4] = radius & 0xFF;
    
    serial_write(roombaCommand, 5);                 // Send command

    #ifdef ROOMBA_DEBUG
    printf ("Roomba drive velocity 0x%.2x 0x%.2x  Radius 0x%.2x 0x%.2x", (velocity & 0xFF00) >> 8, velocity & 0xFF, (radius & 0xFF00) >> 8, radius & 0xFF);
    #endif

    return -1;
}

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
int Roomba_drive_direct(int rightVelocity, int leftVelocity){
    if(rightVelocity < -500)
        rightVelocity = -500;
    else if(rightVelocity > 500)
            rightVelocity = 500;

    if(leftVelocity < -500)
        leftVelocity = -500;
    else if(leftVelocity > 500)
            leftVelocity = 500;   

    // Drive Roomba velocity from -500 to 500mmS and radius from -2000 to +2000mm
    roombaCommand[0] = OPCODE_DRIVE_DIRECT;
    roombaCommand[1] = (rightVelocity & 0xFF00) >> 8;
    roombaCommand[2] = rightVelocity & 0xFF;
    roombaCommand[3] = (leftVelocity & 0xFF00) >> 8;
    roombaCommand[4] = leftVelocity & 0xFF;
    
    serial_write(roombaCommand, 5);                 // Send command

    #ifdef ROOMBA_DEBUG
    printf ("Roomba drive velocity 0x%.2x 0x%.2x  Radius 0x%.2x 0x%.2x", (velocity & 0xFF00) >> 8, velocity & 0xFF, (radius & 0xFF00) >> 8, radius & 0xFF);
    #endif

    return -1;
}

/**
 * @brief This command lets you ask for packets. The result is returned once
 * @param packetNb The number of the requiered packet
 * @return int packet return value
 */

int Roomba_query_packet(unsigned char packetNb){
    int byteReceived;

    roombaCommand[0] = OPCODE_QUERY;
    roombaCommand[1] = 1;
    roombaCommand[2] = packetNb;

    serial_write(roombaCommand, 5);                 // Send command

    byteReceived=serial_read(roombaResponse);
    
    #ifdef ROOMBA_DEBUG
        printf("RECEIVED DATA FROM ROOMBA [%d] for Packet #%d: ", byteReceived, packetNb);
        int index = 0;
        for (index = 0; index < byteReceived; ++index){
            printf("|0x%.2x", roombaResponse[index]);
        }
        printf_s("\n");
    #endif
}

/**
 * @brief The state of the Roomba buttons are sent as individual bits (0 = button not pressed, 1 = button
*pressed). The day, hour, minute, clock, and scheduling buttons that exist only on Roomba 560 and 570
*will always return 0 on a Roomba 510 or 530 robot.
*Range: 0 – 255
*Buttons Packet ID: 18 Data Bytes: 1, unsigned
 * 
 * @param buttonMask 
 * BUTTON_CLEAN        0x01 | 
*BUTTON_SPOT         0x02 | 
*BUTTON_DOCK         0x04 | 
*BUTTON_SCHEDULE     0x40 | 
*BUTTON_CLOCK        0x80
 * @return int 
 */
int Roomba_get_button_state(char buttonMask){
    int byteReceived;

    roombaCommand[0] = OPCODE_QUERY;
    roombaCommand[1] = 1;
    roombaCommand[2] = BUTTONS;

    serial_write(roombaCommand, 3);                 // Send command QUERY

    byteReceived=serial_read(roombaResponse);
    
    #ifdef ROOMBA_DEBUG
        printf("RECEIVED DATA FROM ROOMBA [%d] for Button: ", byteReceived);
        int index = 0;
        for (index = 0; index < byteReceived; ++index){
            printf("|0x%.2x", roombaResponse[index]);
        }
        printf_s("\n");
    #endif

    return roombaResponse[0] & buttonMask;
}



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
int Roomba_get_bumpsAndWheeldrops(char sensorMask){
    int byteReceived;

    roombaCommand[0] = OPCODE_QUERY;
    roombaCommand[1] = 1;
    roombaCommand[2] = BUMPS_WHEELDROPS;

    serial_write(roombaCommand, 3);                 // Send command QUERY

    byteReceived=serial_read(roombaResponse);
    
    #ifdef ROOMBA_DEBUG
        printf("RECEIVED DATA FROM ROOMBA [%d] for Button: ", byteReceived);
        int index = 0;
        for (index = 0; index < byteReceived; ++index){
            printf("|0x%.2x", roombaResponse[index]);
        }
        printf_s("\n");
    #endif

    return roombaResponse[0] & sensorMask;
}

/**
 * @brief Get the strength of the cliff (ground) signal. Returned as an unsigned 16-bit value (Range: 0-4095)
* @param cliffSensor 
* LEFT         0,
* FRONT_LEFT   1,
* FRONT_RIGHT  4,
* RIGHT        5
* @return int 
*/

int Roomba_get_CliffSignal(unsigned char cliffSensor){
    int byteReceived;
    char packet = -1;
    int result;

    switch(cliffSensor){
        case LEFT  : packet = CLIFF_LEFT_SIGNAL; break;
        case RIGHT : packet = CLIFF_RIGHT_SIGNAL; break;
        case FRONT_LEFT : packet = CLIFF_FRONT_LEFT_SIGNAL; break;
        case FRONT_RIGHT : packet = CLIFF_FRONT_RIGHT_SIGNAL; break;
        default : packet = -1; break;
    }

    if(packet != -1){
        roombaCommand[0] = OPCODE_QUERY;
        roombaCommand[1] = 1;
        roombaCommand[2] = packet;

        serial_write(roombaCommand, 3);                 // Send command QUERY

        byteReceived=serial_read(roombaResponse);
        
        #ifdef ROOMBA_DEBUG
            printf("RECEIVED DATA FROM ROOMBA [%d] for Cliff: ", byteReceived);
            int index = 0;
            for (index = 0; index < byteReceived; ++index){
                printf("|0x%.2x", roombaResponse[index]);
            }
            printf_s("\n");
        #endif

        result = (roombaResponse[0]<< 8) | roombaResponse[1];
    }else{
        result = -1;
    }   
    return result;
}

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
int Roomba_get_WallBumpSignal(unsigned char wallBumpSensor){
    int byteReceived;
    char packet = -1;
    int result;

    switch(wallBumpSensor){
        case LEFT  : packet = LIGHT_BUMP_LEFT; break;
        case FRONT_LEFT : packet = LIGHT_BUMP_FRONT_LEFT; break;
        case CENTER_LEFT : packet = LIGHT_BUMP_CENTER_LEFT; break;
        case CENTER_RIGHT  : packet = LIGHT_BUMP_CENTER_RIGHT; break;
        case FRONT_RIGHT : packet = LIGHT_BUMP_FRONT_RIGHT; break;
        case RIGHT : packet = LIGHT_BUMP_RIGHT; break;
        default : packet = -1; break;
    }

    if(packet != -1){
        roombaCommand[0] = OPCODE_QUERY;
        roombaCommand[1] = 1;
        roombaCommand[2] = packet;

        serial_write(roombaCommand, 3);                 // Send command QUERY

        byteReceived=serial_read(roombaResponse);
        
        #ifdef ROOMBA_DEBUG
            printf("RECEIVED DATA FROM ROOMBA [%d] for Wall bump light signal: ", byteReceived);
            int index = 0;
            for (index = 0; index < byteReceived; ++index){
                printf("|0x%.2x", roombaResponse[index]);
            }
            printf_s("\n");
        #endif

        result = (roombaResponse[0]<< 8) | roombaResponse[1];
    }else{
        result = -1;
    }   
    return result;
}

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
int Roomba_get_encoder(unsigned char encoder){
    int byteReceived;
    int packetCode;
    int encoderValue=-1;

    if(encoder == LEFT)
        packetCode = ENCODER_COUNTS_LEFT;
    else
        if(encoder == RIGHT)
            packetCode = ENCODER_COUNTS_RIGHT;
        else 
            packetCode = -1;

    if(packetCode >= 0){
        roombaCommand[0] = OPCODE_QUERY;
        roombaCommand[1] = 1;
        roombaCommand[2] = packetCode;

        serial_write(roombaCommand, 3);                 // Send command QUERY

        byteReceived=serial_read(roombaResponse);
        
        #ifdef ROOMBA_DEBUG
            printf(" ********ENCODER #%d:  %d\n", encoder, encoderValue);
            printf("RECEIVED DATA FROM ROOMBA [%d] for encoder #%d: ", byteReceived, packetCode);
            int index = 0;
            for (index = 0; index < byteReceived; ++index){
                printf("|0x%.2x", roombaResponse[index]);
            }
            printf_s("\n");
        #endif
        encoderValue = (roombaResponse[0] << 8);
        encoderValue = (encoderValue & 0xFF00) | roombaResponse[1];

        return encoderValue;
    }
    
    return encoderValue;
}

/**
 * @brief The stasis caster sensor returns 1 when the robot is making forward progress and 0 when it is not. It
 * always returns 0 when the robot is turning, driving backward, or not driving. If the stasis wheel is too
 * dirty to be read, a value of 2 is returned. If this happens, remove the stasis wheel and clean it with a
 * damp cloth, then dry it thoroughly before reinstalling the wheel.
 * 
 * @return int 
 */
int Roomba_get_statsis_caster_state(void){
    int byteReceived;

        roombaCommand[0] = OPCODE_QUERY;
        roombaCommand[1] = 1;
        roombaCommand[2] = STASIS_CASTER;

        serial_write(roombaCommand, 3);                 // Send command QUERY

        byteReceived=serial_read(roombaResponse);
        
        #ifdef ROOMBA_DEBUG
            printf("RECEIVED DATA FROM ROOMBA [%d] for Statsis #%d: ", byteReceived, packetCode);
            int index = 0;
            for (index = 0; index < byteReceived; ++index){
                printf("|0x%.2x", roombaResponse[index]);
            }
            printf_s("\n");
        #endif
    
    return roombaResponse[0] & 0x03;
}


