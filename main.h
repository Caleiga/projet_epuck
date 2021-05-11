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
#define WIDTH_SLOPE				15
#define PXTOCM					0.01f 					//experimental value
#define ERROR_THRESHOLD			0.1f					//[cm] because of the noise of the camera
#define KP						100.0f
#define KI 						0.5f					//must not be zero
#define MAX_SUM_ERROR 			400
#define LINE_NUMBER 			479 					// px
#define IMAGE_HEIGHT 			480 					// px
#define IMAGE_WIDTH 			640 					// px
#define FAST 					700						// tics per second, equivalent to half a rotation per second
#define SLOW					500
#define LEFT					0
#define RIGHT					1
#define JUMP					70
#define CORRECTION_SIDE_LOST	-200
#define MEAN_DIFFERENCE_LIMIT	40
#define MAX_SPEED				1000
#define VIGNETTING_OFFSET		100
#define RED						0
#define GREEN					1
#define OVERTAKE_DISTANCE		200
#define STOP 					0						//vitesse nulle
#define WHITE 					80

#define GOAL_LINE_POSITION_RIGHT 		450 			// where in the image the robot should aim to keep the line representing the side of the track
#define GOAL_LINE_POSITION_LEFT 		150

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
