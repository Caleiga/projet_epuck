#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

void process_image_start(void);
void determine_line_colour(uint8_t *buffer_red, uint8_t *buffer_green, uint8_t *buffer_blue);
void determine_right_side_position(uint8_t *buffer);
void determine_left_side_position(uint8_t *buffer);

uint16_t get_track_side_position(void);
bool get_track_side_lost(void);
uint16_t get_line_colour(void);
bool get_straight_line(void);

#endif /* PROCESS_IMAGE_H */
