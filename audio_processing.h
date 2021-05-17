/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
*/
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

void sound_remote(float* data);
bool get_boxbox(void);
void clear_boxbox(void);
void processAudioData(int16_t *data, uint16_t num_samples);
