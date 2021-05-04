#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
<<<<<<< HEAD
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>
=======
#include <arm_math.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <drive.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>

//-------------------------------------------------------------------------------------------------------------

void SendUint8ToComputer(uint8_t* data, uint16_t size)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}
>>>>>>> origin/camera

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

<<<<<<< HEAD
    timer12_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
=======
    //starts the serial communication
    serial_start();

    //start the USB communication
    usb_start();

    //starts the camera
    dcmi_start();
  	po8030_start();
  	po8030_set_awb(0);
  	po8030_set_ae(0);
  	po8030_set_rgb_gain(94, 100, 100);
  	po8030_set_exposure(300,0);

  	//starts the distance detector
  	VL53L0X_start();

   	//inits the motors
   	motors_init();

   	//starts the threads for the the processing of the image and the driving
   	drive_start();
   	process_image_start();
>>>>>>> origin/camera

    while (1) {
<<<<<<< HEAD
=======

    	overtake();

        chThdSleepMilliseconds(2000);
>>>>>>> origin/camera
    }
}

//-------------------------------------------------------------------------------------------------------------

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

//-------------------------------------------------------------------------------------------------------------

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
