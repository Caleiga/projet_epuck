/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
*/

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

void process_image_start(void);
bool get_coloured_line(void);
uint16_t get_track_side_position(void);
bool get_track_side_lost(void);
bool get_line_colour(void);
bool get_straight_line(void);
void determine_line_colour(uint8_t *buffer_red, uint8_t *buffer_green);
void determine_right_side_position(uint8_t *buffer);
void determine_left_side_position(uint8_t *buffer);

#endif /* PROCESS_IMAGE_H */
