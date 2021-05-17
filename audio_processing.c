/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
 */

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <audio/microphone.h>
#include <fft.h>
#include <arm_math.h>
#include <leds.h>
#include <drive.h>
#include <audio_processing.h>

//-------------------------------------------------------------------------------------------------------------

#define MIN_VALUE_THRESHOLD	100000

//Box box a un pic de fréquence entre 34 et 35 (531,25Hz et 546,875Hz)
#define MIN_FREQ_BOX	31
#define FREQ_LOW_BOX	34	//531,25HZ
#define FREQ_HIGH_BOX	35	//546,875Hz
#define MAX_FREQ_BOX	38
#define MIN_FREQ_GO		104
#define FREQ_LOW_GO 	106
#define FREQ_HIGH_GO 	107
#define MAX_FREQ_GO		108

//-------------------------------------------------------------------------------------------------------------

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

static bool boxbox = 0;
static int box_nb_of_times = 0;

//-------------------------------------------------------------------------------------------------------------

void sound_remote(float* data) {
	float max_norm = MIN_VALUE_THRESHOLD;
	int8_t max_norm_index = -1;

	//Search for pitstop order "box box"
	for(uint8_t i = MIN_FREQ_BOX ; i <= MAX_FREQ_BOX ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	if(max_norm_index >= FREQ_LOW_BOX && max_norm_index <= FREQ_HIGH_BOX){
		box_nb_of_times += 1;
	}

	//Search for start sound
	for(uint8_t i = MIN_FREQ_GO ; i <= MAX_FREQ_GO ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}
	
	if(max_norm_index >= FREQ_LOW_GO && max_norm_index <= FREQ_HIGH_GO){
		clear_stop();
	}

	if(box_nb_of_times >= 2) {
		boxbox = true;
		box_nb_of_times = 0;
	}
}

bool get_boxbox(void) {
	return boxbox;
}

void clear_boxbox(void) {
	boxbox = false;
}

void processAudioData(int16_t *data, uint16_t num_samples){

	static uint16_t nb_samples = 0;
	
	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){

		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		nb_samples = 0;
		sound_remote(micLeft_output);
	}
}
