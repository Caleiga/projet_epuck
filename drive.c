#include "ch.h"
#include "hal.h"
//#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <drive.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>
#include <audio_processing.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>

//-------------------------------------------------------------------------------------------------------------

static bool track_side = LEFT; 	 	// which side of the track the robot should follow.
static bool pitstop = 0;	 		// whether the order for a pitstop has been received.
static bool car_in_front = 0; 		// whether there is a car to overtake in front.
static bool fast = 1;				// whether FAST or SLOW average speed.
static bool stop = 0; 				// whether the robot has to be stopped (pitlane)
static bool pit_lane_next_turn = false;
static bool pitting = 0;

//-------------------------------------------------------------------------------------------------------------

void clear_stop(void) {
	stop = 0;
}

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

void determine_pit_lane_next_turn(void) {
	//checks if there are any signs announcing that the pit lane is after the next turn
	if(get_calibrated_prox(2) > 100 || get_calibrated_prox(5) > 100) {
    	pit_lane_next_turn = true;
    } else {
        pit_lane_next_turn = false;
   	}
}

//-------------------------------------------------------------------------------------------------------------

void overtake(void) {

	//checks if in a straight line and if there is a car in front. If yes, overtaking is done by switching the track side.
	if(fast && car_in_front)
		track_side = !track_side;
}

//-------------------------------------------------------------------------------------------------------------

void determine_track_side(void) {
	//check for the pitlane first
	if(get_boxbox() && pit_lane_next_turn) {
		track_side = LEFT;
		pitting = 1;
	
	} else if(get_coloured_line()) { 	//checks if there is a coloured line was detected.

		//determines the track side to follow depending on what the line colour is.
		bool colour = get_line_colour();

		if(colour == RED) {
			track_side = LEFT;

		} else if(colour == GREEN) {
			track_side = RIGHT;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------

//checks if there is a car to overtake in front
void determine_car_in_front(void) {

	if(VL53L0X_get_dist_mm() < OVERTAKE_DISTANCE) {
		car_in_front = 1;
		//set_front_led(1);
	} else {
		car_in_front = 0;
		//set_front_led(0);
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

	speed_correction = KP * error;

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
    messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    proximity_msg_t prox_values;


    while(1){

        time = chVTGetSystemTime();
        messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
        
        //check if the robot is allowed to drive
        if(!stop) {

        	determine_pit_lane_next_turn();
        	
        	fast = get_straight_line();

		    determine_track_side();

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

			//check the pitlane stop line
			if (pitting && get_line_colour() == GREEN) {
				
				clear_boxbox();
				stop = true;
				pitting = 0;
			}

		} else {

			right_motor_set_speed(STOP);
			left_motor_set_speed(STOP);

		}		

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


//-------------------------------------------------------------------------------------------------------------

void drive_start(void){
	chThdCreateStatic(waDrive, sizeof(waDrive), NORMALPRIO, Drive, NULL);
}

