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
#include <audio_processing.h>
#include <sensors/proximity.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>
#include <arm_math.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <drive.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>


messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);
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

static void timer12_start(void){
    //General Purpose Timer configuration
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
}


int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    proximity_start();
    messagebus_init(&bus, &bus_lock, &bus_condvar);

    timer12_start();
    //starts the microphones processing thread.
    //it calls the callback given in parameter when samples are ready
    mic_start(&processAudioData);
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

    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;

    while (1) {
        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
        // Sensors info print: each line contains data related to a single sensor.
        /*for (uint8_t i = 0; i < sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++) {
            if(get_calibrated_prox(i) > 75) {
                set_body_led(1);
                switch (i) {
                    case 0:
                        set_led(LED1, 1);
                        break;

                    case 1:
                        break;

                    case 2:
                        set_led(LED3, 1);
                        break;

                    case 3:
                        set_led(LED5, 1);
                        break;

                    case 4:
                        set_led(LED5, 1);
                        break;

                    case 5:
                        set_led(LED7, 1);
                        break;

                    case 6:
                        break;

                    case 7:
                        set_led(LED1, 1);
                        break;
                }
            } else {
                switch (i) {
                    case 0:
                        set_led(LED1, 0);
                        break;

                    case 1:
                        break;

                    case 2:
                        set_led(LED3, 0);
                        break;

                    case 3:
                        set_led(LED5, 0);
                        break;

                    case 4:
                        set_led(LED5, 0);
                        break;

                    case 5:
                        set_led(LED7, 0);
                        break;

                    case 6:
                        break;

                    case 7:
                        set_led(LED1, 0);
                        break;
                }
            }
        }*/
    	//overtake();

        chThdSleepMilliseconds(1000);
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
