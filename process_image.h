#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_error(void);
bool get_track_side(void);
//bool determine_track_side(uint8_t *buffer);
uint16_t determine_line_position(uint8_t *buffer);
void process_image_start(void);
//uint16_t get_line_position(void);

uint16_t determine_right_side_position(uint8_t *buffer);
uint16_t determine_left_side_position(uint8_t *buffer);

#endif /* PROCESS_IMAGE_H */
