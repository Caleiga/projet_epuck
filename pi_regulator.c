#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

//-------------------------------------------------------------------------------------------------------------




//simple PI regulator implementation
int16_t pi_regulator(float error){


	float speed = 0;
	static float sum_error = 0;

	//disables the PI regulator if the error is too small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		sum_error = 0;
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error; //+ KI * sum_error;

    return (int16_t)speed;
}

//-------------------------------------------------------------------------------------------------------------


static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t speed_correction = 0;
    bool side= 0;

    while(1){
        time = chVTGetSystemTime();
        
        side = get_track_side();

        speed_correction = pi_regulator(get_error());


        if(side == LEFT){
        	right_motor_set_speed(NORMAL_SPEED);
			left_motor_set_speed(NORMAL_SPEED + speed_correction);
        } else {
        	right_motor_set_speed(NORMAL_SPEED + speed_correction);
			left_motor_set_speed(NORMAL_SPEED);
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


//-------------------------------------------------------------------------------------------------------------

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO + 1, PiRegulator, NULL);
}
