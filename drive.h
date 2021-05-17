/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
	Nom original : pi_regulator.h, renommé : drive.h
*/

#ifndef DRIVE_H
#define DRIVE_H

void drive_start(void);
void clear_stop(void);
bool get_pitstop(void);
bool get_car_in_front(void);
void determine_pit_lane_next_turn(void);
void overtake(void);
void determine_track_side(void);
void determine_obstacle(void);
int16_t pi_regulator(float error);
bool get_pit_lane_next_turn(void);
bool get_stop(void);

#endif /* DRIVE_H */
