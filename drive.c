/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
	Nom original : pi_regulator.c, renommé : drive.c
*/

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <motors.h>
#include <image_processing.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <leds.h>
#include <audio_processing.h>
#include <sensors/proximity.h>
#include <msgbus/messagebus.h>
#include <drive.h>

//-------------------------------------------------------------------------------------------------------------

static bool pitstop = 0;	 				// whether the order for a pitstop has been received.
static bool car_in_front = 0; 				// whether there is a car to overtake in front.
static bool man_in_front = 0;
static bool overtaking = 0;					// whether overtaking or not.
static bool stop = 1; 						// whether the robot has to be stopped (pitlane).
static bool pit_lane_next_turn = 0;			// whether the pit lane sign has been detected.
static bool pitting = 0;					// whether the robot is on its way to the pitstop.
static bool hairpin_turn = 0;
static bool wide_turn = 0;
static bool exiting_pit = 0;
static bool ir_sensors_pitting = 0;

//-------------------------------------------------------------------------------------------------------------

void clear_stop(void) {
	stop = 0;
}

bool get_pitstop(void) {
	return pitstop;
}

bool get_car_in_front(void) {
	return car_in_front;
}

void determine_pit_lane_next_turn(void) {

	static uint8_t noise_filter_counter = 0;
	static uint8_t noise_filter_counter_pitting = 0;
	//checks if there are any signs announcing that the pit lane is after the next turn
	if(get_track_side() == LEFT) {
		if(get_calibrated_prox(LEFT_PROXIMITY) > IR_INTENSITY) {
			++noise_filter_counter;
		} else {
			noise_filter_counter = 0;
			pit_lane_next_turn = 0;
		}

	} else {
		if (get_calibrated_prox(RIGHT_PROXIMITY) > IR_INTENSITY) {
			++noise_filter_counter;
		} else {
			noise_filter_counter = 0;
			pit_lane_next_turn = 0;
		}
	}

	if(noise_filter_counter > 5) {
		pit_lane_next_turn = 1;
	}

	if(get_calibrated_prox(LEFT_PROXIMITY) > 100 || get_calibrated_prox(RIGHT_PROXIMITY) > 100) {
		++noise_filter_counter_pitting;
	} else {
		noise_filter_counter_pitting = 0;
		ir_sensors_pitting = 0;
	}

	if(noise_filter_counter_pitting > 5) {
		ir_sensors_pitting = 1;
	}
}

void overtake(void) {

	//checks if in a straight line and if there is a car in front. If yes, overtaking is done by switching the track side.
	if(get_straight_line() && car_in_front) {
		toggle_track_side();
		overtaking = 1;
	}
}

void determine_track_side(void) {
	//check for the pit lane first
	if(get_boxbox() && pit_lane_next_turn) {
		set_track_side_left();
		pitting = 1;
	
	} else if (exiting_pit && get_straight_line() && !get_coloured_line()) {
		set_track_side_left();
		exiting_pit = 0;

	} else if(get_coloured_line() && !pitting && !exiting_pit) { 	//checks if a coloured line was detected.

		//determines the track side to follow depending on what the line colour is.
		if(get_line_colour() == RED) {
			if(get_track_side() == LEFT)
				hairpin_turn = 1;
			else {
				set_track_side_left();
				wide_turn = 1;
			}
		} else {
			if(get_track_side() == RIGHT)
				hairpin_turn = 1;
			else {
				set_track_side_right();
				wide_turn = 1;
			}
		}
	}
}

void determine_obstacle(void) {

	static uint8_t noise_filter_counter_1 = 0;
	static uint8_t noise_filter_counter_2 = 0;


	if((VL53L0X_get_dist_mm() < OVERTAKE_DISTANCE) && (VL53L0X_get_dist_mm() > 0)) {
		++noise_filter_counter_1;

	} else {
		noise_filter_counter_1 = 0;
		car_in_front = 0;
	}

	if(noise_filter_counter_1 > 5) {
		car_in_front = 1;
	}

	if((VL53L0X_get_dist_mm() < STOP_DISTANCE) && (VL53L0X_get_dist_mm() > 0)) {
		++noise_filter_counter_2;

	} else {
		noise_filter_counter_2 = 0;
		man_in_front = 0;
	}

	if(noise_filter_counter_2 == 5) {
		man_in_front = 1;
	}
}

int16_t pi_regulator(float error){

	float kp = 0;
	float ki = 0;
	static float sum_error = 0;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth.
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	if(overtaking) {
		if(car_in_front) {
			kp = KP_OVERTAKE;
		} else if(get_track_side() == LEFT) {
			return 2*KP_ADJUSTING * error - 2*get_calibrated_prox(IR_1) - get_calibrated_prox(IR_2);
		} else {
			return 2*KP_ADJUSTING * error - 2*get_calibrated_prox(IR_8) - get_calibrated_prox(IR_7);
		}

	} else if(pitting) {
		if(ir_sensors_pitting || !get_track_side_lost()) {
			kp = KP_PITTING;
		} else {
			kp = KP_ADJUSTING;
		}

	} else if(wide_turn) {
		sum_error += error;
		kp = KP_WIDE_TURN;
		if(get_track_side_lost()) {
			ki = KI_WIDE_TURN;
		} else {
			ki = 0;
		}
	} else if(hairpin_turn) {
		sum_error += error;
		kp = KP_HAIRPIN_TURN;
		ki = KI_HAIRPIN_TURN;

	} else {
		kp = KP_NORMAL;
		sum_error = 0;
	}


	return kp * error + ki * sum_error;
}

bool get_pit_lane_next_turn(void) {
	return pit_lane_next_turn;
}

bool get_stop(void) {
	return stop;
}
//-------------------------------------------------------------------------------------------------------------

static THD_WORKING_AREA(waDrive, 256);
static THD_FUNCTION(Drive, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t average_speed = 0;
    int16_t speed_correction = 0;
    int16_t error_pixels = 0;
    float error_cm = 0;

    while(1){

        time = chVTGetSystemTime();
        
        //check if the robot is allowed to drive
        if(!stop) {

        	determine_track_side();

		    if(get_straight_line()) {
		    	set_led(LED5,0);
		        average_speed = FAST;
		    	hairpin_turn = 0;
		    	wide_turn = 0;
		    } else {
		        average_speed = SLOW;
		        set_led(LED5,1);
		    }

		    determine_pit_lane_next_turn();

		    //check the pitlane stop line
		    if (pitting && man_in_front) {
		    	clear_boxbox();
		    	stop = true;
		    	pitting = 0;
		    	exiting_pit = 1;
		    }

		    determine_obstacle();

		    if(!get_track_side_lost() && !pitting) {
		    	overtaking = 0;
		    	overtake();
		    }

		    if(get_track_side_lost()) {
		    	error_pixels = ARTIFICIAL_ERROR;
		    } else if(get_track_side() == LEFT) {
		        error_pixels = get_track_side_position() - GOAL_LINE_POSITION_LEFT;
		    } else {
		        error_pixels = GOAL_LINE_POSITION_RIGHT - get_track_side_position();
		    }

		    error_cm = PXTOCM * error_pixels;

		    speed_correction = pi_regulator(error_cm);

		    if(get_track_side() == LEFT){
		    	right_motor_set_speed(average_speed);
				left_motor_set_speed(average_speed + speed_correction);
		    } else {
		    	right_motor_set_speed(average_speed + speed_correction);
				left_motor_set_speed(average_speed);
			}

		} else {

			right_motor_set_speed(STOP);
			left_motor_set_speed(STOP);

		}		

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void drive_start(void){
	chThdCreateStatic(waDrive, sizeof(waDrive), NORMALPRIO, Drive, NULL);
}
