#ifndef PI_REGULATOR_H
#define PI_REGULATOR_H

int16_t pi_regulator(float error);
void drive_start(void);
void determine_overtake_status(void);
void overtake(void);
void determine_track_side(void);
void determine_pit_lane_next_turn(void);


bool get_overtake_status(void);
bool get_track_side(void);
bool get_pitstop(void);
bool get_pit_lane_next_turn(void);
void clear_stop(void);

#endif /* PI_REGULATOR_H */
