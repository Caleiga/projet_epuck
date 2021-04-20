#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>

#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <arm_math.h>

//uncomment to send the FFTs results from the real microphones
//#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

//-------------------------------------------------------------------------------------------------------------

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();


    //inits the motors
    motors_init();

    //starts pi controller thread
    pi_regulator_start();

    //starts camera threads
    process_image_start();

    /* Infinite loop. */
    while (1) {
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
