#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>
#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>



static bool track_side = 1; 	// which side of the track the robot should follow. 0 for left, 1 for right.
static float error = 0;
static uint16_t right_side_position = GOAL_LINE_POSITION_RIGHT;
static uint16_t left_side_position = GOAL_LINE_POSITION_LEFT;


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//-------------------------------------------------------------------------------------------------------------

float get_error(void){
	return error;
}

//uint16_t get_line_position(void){
//	return line_position;
//}

//-------------------------------------------------------------------------------------------------------------

bool get_track_side(void){
	return track_side;
}

//-------------------------------------------------------------------------------------------------------------

/*bool determine_track_side(uint8_t *buffer){

	bool left_or_right = 0;
	// A COMPLETER

	return left_or_right;
} */

//-------------------------------------------------------------------------------------------------------------

/*uint16_t determine_line_position(uint8_t *buffer){
	
	uint16_t i, begin, end = 0;
	uint8_t stop, wrong_line = 0, line_not_found = 0;
	uint32_t mean = 0;

	//performs an average
	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; i++){
		mean += buffer[i];
	}
	mean /= IMAGE_BUFFER_SIZE;

	do{
		wrong_line = 0;
		//search for a begin
		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{ 
			//the slope must at least be WIDTH_SLOPE wide and is compared
		    //to the mean of the image
		    if(buffer[i] > mean && buffer[i+WIDTH_SLOPE] < mean)
		    {
		        begin = i;
		        stop = 1;
		    }
		    i++;
		}
		//if a begin was found, search for an end
		if (i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE) && begin)
		{
		    stop = 0;
		    
		    while(stop == 0 && i < IMAGE_BUFFER_SIZE)
		    {
		        if(buffer[i] > mean && buffer[i-WIDTH_SLOPE] < mean)
		        {
		            end = i;
		            stop = 1;
		        }
		        i++;
		    }
		    //if an end was not found
		    if (i > IMAGE_BUFFER_SIZE || !end)
		    {
		        line_not_found = 1;
		    }
		}
		else//if no begin was found
		{
		    line_not_found = 1;
		}

		//if a line too small has been detected, continues the search
		if(!line_not_found && (end-begin) < MIN_LINE_WIDTH){
			i = end;
			begin = 0;
			end = 0;
			stop = 0;
			wrong_line = 1;
		}
	}while(wrong_line);

	if(!line_not_found){
		line_position = (begin + end)/2; //gives the new line position
	}

	return line_position;
}

*/

uint16_t determine_right_side_position(uint8_t *buffer){

	uint16_t i = 0;
	uint16_t stop = 0;
	uint32_t falling_edge = 0;

	while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
	{
		//the slope must at least be WIDTH_SLOPE wide and is compared to the mean of the image
		if(buffer[i] > buffer[i+WIDTH_SLOPE] + JUMP)
		{
			falling_edge = i;
			stop = 1;
		}
		i++;
	}

	if(stop) {
		right_side_position = falling_edge;
		set_body_led(1);
	} else {
		set_body_led(0);
		right_side_position += CORRECTION_SIDE_LOST;
	}
	return right_side_position;
}


uint16_t determine_left_side_position(uint8_t *buffer){

	uint16_t i = 0;
		uint16_t stop = 0;
		uint32_t rising_edge = 0;

		while(stop == 0 && i < (IMAGE_BUFFER_SIZE - WIDTH_SLOPE))
		{
			//the slope must at least be WIDTH_SLOPE wide and is compared to the mean of the image
			if(buffer[i] < buffer[i+WIDTH_SLOPE] - JUMP )
			{
				rising_edge = i;
				stop = 1;
			}
			i++;
		}

		if(stop) {
			left_side_position = rising_edge;
			set_body_led(1);
		} else{
			set_body_led(0);
			left_side_position -= CORRECTION_SIDE_LOST;
		}
		return left_side_position;
}
//-------------------------------------------------------------------------------------------------------------

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 479 and 480 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, LINE_NUMBER, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//waits for the capture to be done
		wait_image_ready();
		//signals an image has been captured
		chBSemSignal(&image_ready_sem);

    }
}

//-------------------------------------------------------------------------------------------------------------

static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t error_pixels = 0;

	//bool left_or_right = 0;



	bool send_to_computer = true;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2){
			//extracts first 5bits of the first byte
			//takes nothing from the second byte
			image[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;
		}

		//search for a line in the image, gets its position and compares it to the desires value in pixels
		if(track_side == RIGHT)
			error_pixels = GOAL_LINE_POSITION_RIGHT - determine_right_side_position(image);
		else
			error_pixels = determine_left_side_position(image) - GOAL_LINE_POSITION_LEFT;

		//converts the error from pixels to cm
		error = PXTOCM*error_pixels;

		if(send_to_computer){
			//sends the image to the computer
			SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
		}
		send_to_computer = !send_to_computer;
    }
}

//-------------------------------------------------------------------------------------------------------------

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
