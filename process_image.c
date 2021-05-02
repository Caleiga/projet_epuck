#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>
#include <main.h>
#include <camera/po8030.h>
#include <drive.h>
#include <process_image.h>


static uint16_t track_side_position = MIDDLE;		// Position of the side of the track in pixels
static bool coloured_line = 0;						// Whether there is a coloured line
static uint8_t line_colour = NO_COLOURED_LINE;		// The colour of the coloured line (red, green or blue) indicating a turn or the pitstop entrance
static bool straight_line = 1;						// Whether currently in a turn or a straight line
static bool track_side_lost = 0;					// If the side of the track could not be found

static uint8_t mean_red = 0;
static uint8_t mean_green = 0;
static uint8_t mean_blue = 0;

//static uint8_t max_red = 0;
//static uint8_t max_green = 0;
//static uint8_t max_blue = 0;

//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

//-------------------------------------------------------------------------------------------------------------

uint16_t get_track_side_position(void) {
	return track_side_position;
}

//-------------------------------------------------------------------------------------------------------------

bool get_track_side_lost(void) {
	return track_side_lost;
}

//-------------------------------------------------------------------------------------------------------------

uint16_t get_line_colour(void) {
	return line_colour;
}

//-------------------------------------------------------------------------------------------------------------

bool get_straight_line(void) {
	return straight_line;
}

//-------------------------------------------------------------------------------------------------------------

void determine_line_colour(uint8_t *buffer_red, uint8_t *buffer_green, uint8_t *buffer_blue){

//	//reset the maxs and the coloured line boolean
//	max_red = 0;
//	max_green = 0;
//	max_blue = 0;
//	coloured_line = false;
//
//	for(uint16_t i = 0 ; i < IMAGE_BUFFER_SIZE ; ++i){
//		if(max_red < buffer_red[i])
//			max_red = buffer_red[i];
//
//		if(max_red < buffer_green[i])
//			max_green = buffer_green[i];
//
//		if(max_red < buffer_blue[i])
//			max_blue = buffer_blue[i];
//	}
//
//	if((max_red > max_green + MAX_DIFFERENCE_LIMIT) && (max_red > max_blue + MAX_DIFFERENCE_LIMIT)) {
//		if(line_colour == NO_COLOURED_LINE)
//			straight_line = !straight_line;
//		line_colour = RED;
//		set_led(LED7,1);
//		set_led(LED3, 0);
//		coloured_line = true;
//	}
//
//	if((max_blue > max_green + MAX_DIFFERENCE_LIMIT) && (max_blue > max_red + MAX_DIFFERENCE_LIMIT)) {
//		if(line_colour == NO_COLOURED_LINE)
//			straight_line = !straight_line;
//		line_colour = BLUE;
//		set_led(LED3,1);
//		set_led(LED7, 0);
//		coloured_line = true;
//	}
//
//
//	if((max_green > max_red + MAX_DIFFERENCE_LIMIT) && (max_green > max_blue + MAX_DIFFERENCE_LIMIT)) {
//		if(line_colour == NO_COLOURED_LINE)
//			straight_line = !straight_line;
//		line_colour = GREEN;
//		set_led(LED3,1);
//		set_led(LED7, 0);
//		coloured_line = true;
//	}
//
//	if(coloured_line == false)
//		clear_leds();

	// reset the means and the coloured line boolean
	mean_red = 0;
	mean_green = 0;
	mean_blue = 0;
	coloured_line = 0;

	if(get_track_side() == LEFT){
		for(uint16_t i = track_side_position ; i < IMAGE_BUFFER_SIZE - VIGNETTING_OFFSET ; ++i){
			mean_red += buffer_red[i];
			mean_green += buffer_green[i];
			mean_blue += buffer_blue[i];
		}
		mean_red /= (IMAGE_BUFFER_SIZE - VIGNETTING_OFFSET - track_side_position);
		mean_green /= (IMAGE_BUFFER_SIZE - VIGNETTING_OFFSET - track_side_position);
		mean_blue /= (IMAGE_BUFFER_SIZE - VIGNETTING_OFFSET - track_side_position);
	} else {
		for(uint16_t i = VIGNETTING_OFFSET ; i < track_side_position ; ++i){
			mean_red += buffer_red[i];
			mean_green += buffer_green[i];
			mean_blue += buffer_blue[i];
		}
		mean_red /= (track_side_position - VIGNETTING_OFFSET);
		mean_green /= (track_side_position - VIGNETTING_OFFSET);
		mean_blue /= (track_side_position - VIGNETTING_OFFSET);
	}

	if((mean_red > mean_green + MEAN_DIFFERENCE_LIMIT) && (mean_red > mean_blue + MEAN_DIFFERENCE_LIMIT)) {
		if(line_colour == NO_COLOURED_LINE)
			straight_line = !straight_line;
		line_colour = RED;
		set_led(LED7,1);
		set_led(LED3, 0);
		coloured_line = true;
	}

	if((mean_blue > mean_green + MEAN_DIFFERENCE_LIMIT) && (mean_blue > mean_red + MEAN_DIFFERENCE_LIMIT)) {
		if(line_colour == NO_COLOURED_LINE)
			straight_line = !straight_line;
		line_colour = BLUE;
		set_led(LED7,0);
		set_led(LED3, 1);
		coloured_line = true;
	}


	if((mean_green > mean_red + MEAN_DIFFERENCE_LIMIT) && (mean_green > mean_blue + MEAN_DIFFERENCE_LIMIT)) {
		if(line_colour == NO_COLOURED_LINE)
			straight_line = !straight_line;
		line_colour = GREEN;
		set_led(LED7,0);
		set_led(LED3, 1);
		coloured_line = true;
	}

	if(!coloured_line)
		clear_leds();
}

//-------------------------------------------------------------------------------------------------------------

void determine_right_side_position(uint8_t *buffer){

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
		track_side_position = falling_edge;
		track_side_lost = 0;
		set_body_led(1);
	} else {
		set_body_led(0);
		track_side_lost = 1;
	}
}

//-------------------------------------------------------------------------------------------------------------

void determine_left_side_position(uint8_t *buffer){

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
			track_side_position = rising_edge;
			track_side_lost = 0;
			set_body_led(1);
		} else{
			set_body_led(0);
			track_side_lost = 1;
		}
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

static THD_WORKING_AREA(waProcessImage, 2048);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image_red[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_green[IMAGE_BUFFER_SIZE] = {0};
	uint8_t image_blue[IMAGE_BUFFER_SIZE] = {0};

	bool send_to_computer = true;

    while(1){

    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565    
		img_buff_ptr = dcmi_get_last_image_ptr();

		//Extraction of the different colours. All the bits are aligned on the left.
		//Extracts only the red pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
			image_red[i/2] = (uint8_t)img_buff_ptr[i]&0xF8;

		//Extracts only the green pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
			image_green[i/2] =  (((uint8_t)img_buff_ptr[i]&0x07) << 5) | (((uint8_t)img_buff_ptr[i+1]&0xE0) >> 3);

		//Extracts only the blue pixels
		for(uint16_t i = 0 ; i < (2 * IMAGE_BUFFER_SIZE) ; i+=2)
			image_blue[i/2] = (((uint8_t)img_buff_ptr[i+1]&0x1F) << 3);

		//checks if there is a line indicating the beginning or the end of a turn and finds it's colour
		determine_line_colour(image_red, image_green, image_blue);

		//if there is no coloured line search for the track side it should follow
		if(!coloured_line){
			if(get_track_side() == LEFT)
				determine_left_side_position(image_green);
			else
				determine_right_side_position(image_green);
		}

		if(send_to_computer){
			//sends the image to the computer
			SendUint8ToComputer(image_blue, IMAGE_BUFFER_SIZE);
		}
		send_to_computer = !send_to_computer;
    }
}

//-------------------------------------------------------------------------------------------------------------

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
