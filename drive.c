#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <drive.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>

//-------------------------------------------------------------------------------------------------------------

static bool track_side = LEFT; 	 	// which side of the track the robot should follow.
static bool pitstop = 0;	 		// whether the order for a pitstop has been received.
static bool car_in_front = 0; 		// whether there is a car to overtake in front.
static bool fast = 1;				// whether FAST or SLOW average speed.

//-------------------------------------------------------------------------------------------------------------

bool get_track_side(void) {
	return track_side;
}

//-------------------------------------------------------------------------------------------------------------

bool get_pitstop(void) {
	return pitstop;
}

//-------------------------------------------------------------------------------------------------------------

bool get_car_in_front(void) {
	return car_in_front;
}

//-------------------------------------------------------------------------------------------------------------

void overtake(void) {

	//checks if in a straight line and if there is a car in front. If yes, overtaking is done by switching the track side.
	if(fast && car_in_front)
		track_side = !track_side;
}

//-------------------------------------------------------------------------------------------------------------

void determine_track_side(void) {

	//checks if there is a coloured line was detected.
	if(get_coloured_line()) {

		//determines the track side to follow depending on what the line colour is.
		uint8_t colour = get_line_colour();

		if(colour == RED) {
			track_side = LEFT;

		} else if(colour == BLUE) {
			track_side = RIGHT;

		} else if(colour == GREEN) {
			if(pitstop) {
				track_side = LEFT;
			} else
				track_side = RIGHT;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------

//checks if there is a car to overtake in front
void determine_car_in_front(void) {

	if(VL53L0X_get_dist_mm() < OVERTAKE_DISTANCE) {
		car_in_front = 1;
		set_front_led(1);
	} else {
		car_in_front = 0;
		set_front_led(0);
	}
}

//-------------------------------------------------------------------------------------------------------------

// PI regulator to follow the side of the track.
int16_t pi_regulator(float error){


	float speed_correction = 0;
	static float sum_error = 0;

	//disables the PI regulator if the error is too small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy.
	if(abs(error) < ERROR_THRESHOLD){
		sum_error = 0;
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth.
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	if(fast)
		speed_correction = KP * error;
	else
		speed_correction = KP * error + KI * sum_error;

    return (int16_t)speed_correction;
}

//-------------------------------------------------------------------------------------------------------------


static THD_WORKING_AREA(waDrive, 256);
static THD_FUNCTION(Drive, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t average_speed = 0;
    int16_t speed_correction = 0;
    uint16_t error_pixels = 0;
    float error_cm = 0;


    while(1){

        time = chVTGetSystemTime();
        
        //fast = get_straight_line();

        determine_track_side();
        if(track_side == LEFT) {
        	set_led(LED7, 1);
        	set_led(LED3, 0);
        } else {
        	set_led(LED7, 0);
        	set_led(LED3, 1);
        }

        if(fast)
            average_speed = FAST;
        else
            average_speed = SLOW;

        if(!get_track_side_lost()) {
        	determine_car_in_front();
        	overtake();
        }


        if(get_track_side_lost())
        	error_pixels = CORRECTION_SIDE_LOST;
        else if(track_side == LEFT)
            error_pixels = get_track_side_position() - GOAL_LINE_POSITION_LEFT;
        else
            error_pixels = GOAL_LINE_POSITION_RIGHT - get_track_side_position();


        error_cm = PXTOCM * error_pixels;


        speed_correction = pi_regulator(error_cm);


        if(track_side == LEFT){
        	right_motor_set_speed(average_speed);
			left_motor_set_speed(average_speed + speed_correction);
        } else {
        	right_motor_set_speed(average_speed + speed_correction);
			left_motor_set_speed(average_speed);
		}

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


//-------------------------------------------------------------------------------------------------------------

void drive_start(void){
	chThdCreateStatic(waDrive, sizeof(waDrive), NORMALPRIO, Drive, NULL);
}
