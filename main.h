#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					0.01f 					//experimental value
#define GOAL_LINE_DISTANCE 		10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f					//[cm] because of the noise of the camera
#define KP						100.0f
#define KI 						1.0f					//must not be zero
#define MAX_SUM_ERROR 			50
#define LINE_NUMBER 			470 					// px
#define IMAGE_HEIGHT 			480 					// px
#define IMAGE_WIDTH 			640 					// px
#define FAST 					700						// tics per second, equivalent to half a rotation per second
#define SLOW					400
#define MIDDLE 					320
#define LEFT					0
#define RIGHT					1
#define JUMP					20
#define CORRECTION_SIDE_LOST	5
#define MEAN_DIFFERENCE_LIMIT	5
#define MAX_DIFFERENCE_LIMIT	30
#define MAX_SPEED				1000
#define VIGNETTING_OFFSET		100
#define NO_COLOURED_LINE		0
#define RED						1
#define GREEN					2
#define BLUE					3
#define OVERTAKE_DISTANCE		100


#define GOAL_LINE_POSITION_RIGHT 		500 			// where in the image the robot should aim to keep the line representing the side of the track
#define GOAL_LINE_POSITION_LEFT 		100

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
