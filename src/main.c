#include <Windows.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "serial.h"
#include "roomba.h"

int distance_mm;
char clean_toggle =0;

int main(void)
{
    int i=0;
    ROOMBA myRoomba;
    Roomba_init();
    Roomba_set_all_led_state(1);
    sleep(2);

    //sleep(2);
    //Roomba_drive(0, 0);
    //Roomba_set_all_led_state(0);

    Roomba_set_digits_text("SALUT");
    Roomba_set_led_schedule_state(LED_SCHEDULE_WEDNESDAY | LED_SCHEDULE_THURSDAY, 1);
    Roomba_set_led_schedule_state(LED_SCHEDULE_WEDNESDAY | LED_SCHEDULE_THURSDAY, 0);
    Roomba_set_led_state(LED_CLEAN, 1);
    sleep(2);
/*
    Roomba_set_motors_state(MOTOR_MAIN_BRUSH | MOTOR_SIDE_BRUSH, 1);
    sleep(2);
    Roomba_set_motors_state(MOTOR_MAIN_BRUSH, 0);
    sleep(2);
    Roomba_set_motors_state(MOTOR_MAIN_BRUSH_REVERSE, 1);
    sleep(2);
    Roomba_set_motors_state(MOTOR_MAIN_BRUSH_REVERSE, 0);
    sleep(2);
    Roomba_set_motors_state(MOTOR_SIDE_BRUSH, 0);
*/
    
    Roomba_set_led_state(LED_OK, 1);
    while(!Roomba_get_button_state(BUTTON_SPOT)){
    Roomba_update_gr0_status(&myRoomba);
    //printf ("    System: VOLTAGE[%d]mV CURRENT[%d]mA \n", myRoomba.measure.battery.voltage, myRoomba.measure.battery.current);

        unsigned char btnClean, btnDock, btnSchedule, btnClock, btnSpot, btnDay, btnHour;

        btnClean = Roomba_get_button_state(BUTTON_CLEAN);

        if(btnClean)
            if(clean_toggle){
                distance_mm = Roomba_get_encoder(LEFT);
                clean_toggle = 0;
                Roomba_drive(50, -100);
            }
        else{
            if(!clean_toggle){
               clean_toggle = 1;
               Roomba_drive(0, 0);
            }
        }

        btnDay = Roomba_get_button_state(BUTTON_DAY);
        btnHour = Roomba_get_button_state(BUTTON_HOUR);
        
        if(btnDay){
            if(btnHour){
                Roomba_drive_direct(300, -100);
            }
            else
            {
                Roomba_drive_direct(-100, -300);
            }
            
        }else{
            if(clean_toggle)
                Roomba_drive_direct(0, 0);
        }



        int test = (Roomba_get_encoder(LEFT) - distance_mm) * 0.444564;
        usleep(5000);

        char pulse[25];
        sprintf(pulse, "%4d", test);
        Roomba_set_digits_text(pulse);
        
        usleep(5000);
        char Bumps = Roomba_get_bumpsAndWheeldrops(BUMP_RIGHT | BUMP_LEFT | WHEELDROP_RIGHT | WHEELDROP_LEFT | WHEELDROP_CASTER);
        printf("Bumps & Wheeldrops:  0x%.2x\n", Bumps);

        int cliff = Roomba_get_CliffSignal(LEFT);
        printf("wall sensor #%d :  0x%.4x  [%d]\n", LEFT, cliff, cliff);

        /*
  
        printf("--------%s-------", pulse);
        

        printf("ENCODER LEFT: %d\n", Roomba_get_encoder(LEFT));
        printf("ENCODER RIGHT: %d\n", test);
        */
        /*
        Roomba_update_gr0_status(&myRoomba);
        printf ("Button state: CLEAN[%d] SPOT[%d] SCHEDULE[%d] CLOCK[%d] DOCK[%d]\n",myRoomba.button.clean, myRoomba.button.spot, myRoomba.button.schedule, myRoomba.button.clock, myRoomba.button.dock);
        printf ("Sensor state: BUMP LEFT[%d] BUMP RIGHT[%d] WHEELDROP_CASTER[%d] WHEELDROP_LEFT[%d] WHEELDROP_RIGHT[%d]\n",myRoomba.sensor.bump_left, myRoomba.sensor.bump_right, myRoomba.sensor.wheeldrop_caster, myRoomba.sensor.wheeldrop_left, myRoomba.sensor.wheeldrop_right);
        printf ("System: VOLTAGE[%d]mV CURRENT[%d]mA CHARGE_CURRENT[%d]mA CAPACITY[%d]  CHARGE STATUS[%d]\n", myRoomba.measure.battery.voltage, myRoomba.measure.battery.current, myRoomba.measure.battery.charging_current, myRoomba.measure.battery.capacity, myRoomba.measure.battery.charging_state);
        printf ("Distance: [%d]\n", myRoomba.measure.distance_mm);
        */

       //Roomba_query_packet(i);
       usleep(50000);
    }
    Roomba_set_led_state(LED_OK, 0);



    Roomba_drive(0, 0);

    Roomba_stop();

    Exit1();

    return 0;
}