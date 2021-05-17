/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
*/

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <motors.h>
#include <audio/microphone.h>
#include <camera/po8030.h>
#include <drive.h>
#include <image_processing.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <audio_processing.h>
#include <main.h>

//-------------------------------------------------------------------------------------------------------------

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

//-------------------------------------------------------------------------------------------------------------

static bool track_side = LEFT;

//-------------------------------------------------------------------------------------------------------------

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

void set_track_side_left(void){
	track_side = LEFT;
}

void set_track_side_right(void){
	track_side = RIGHT;
}

void toggle_track_side(void) {
	track_side = !track_side;
}

bool get_track_side(void){
	return track_side;
}

//-------------------------------------------------------------------------------------------------------------

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    //start the proximity sensors
    proximity_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the camera
    dcmi_start();
  	po8030_start();
  	po8030_set_awb(0);
  	po8030_set_ae(0);
  	po8030_set_rgb_gain(100, 100, 100);
  	po8030_set_exposure(500,0);

   	//starts the threads for the the processing of the image and the driving
   	process_image_start();

   	//inits the motors
   	motors_init();

   	//starts the distance detector
   	VL53L0X_start();

   	//the driver gets into the car
   	drive_start();

    //start microphones
    mic_start(&processAudioData);

   	track_side = LEFT;

   	clear_leds();

    /* Infinite loop. */
    while (1) {
    }
}

//-------------------------------------------------------------------------------------------------------------

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

