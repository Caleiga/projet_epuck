/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
*/

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				15
#define PXTOCM					0.01f 					//experimental value
#define ERROR_THRESHOLD			0.1f					//[cm] because of the noise of the camera
#define KP_NORMAL				100.0f
#define KP_ADJUSTING			-50.0f
#define KP_PITTING				250.0f
#define KP_OVERTAKE				150.0f
#define KP_WIDE_TURN			35.0f
#define KP_HAIRPIN_TURN			60.0f
#define KI_HAIRPIN_TURN			1.4f
#define KI_WIDE_TURN			0.85f
#define MAX_SUM_ERROR 			700
#define LINE_NUMBER 			479 					// px
#define IMAGE_HEIGHT 			480 					// px
#define IMAGE_WIDTH 			640 					// px
#define FAST 					800					// tics per second, equivalent to half a rotation per second
#define SLOW					600
#define LEFT					0
#define RIGHT					1
#define JUMP					60
#define ARTIFICIAL_ERROR		-200
#define MEAN_DIFFERENCE_LIMIT	50
#define MAX_SPEED				1000
#define VIGNETTING_OFFSET		100
#define RED						0
#define GREEN					1
#define OVERTAKE_DISTANCE		220
#define STOP_DISTANCE 			70
#define STOP 					0						//vitesse nulle
#define WHITE 					100
#define BLACK					50
#define IR_INTENSITY			150
#define LEFT_PROXIMITY			5
#define RIGHT_PROXIMITY			2
#define IR_1					0
#define IR_2					1
#define IR_7					6
#define IR_8					7
#define RTI_LEFT				6
#define GOAL_LINE_POSITION_RIGHT 		450 			// where in the image the robot should aim to keep the line representing the side of the track
#define GOAL_LINE_POSITION_LEFT 		150

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif


void set_track_side_left(void);
void set_track_side_right(void);
void toggle_track_side(void);
bool get_track_side(void);

#endif
