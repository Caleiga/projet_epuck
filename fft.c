/* 	Fichier repris du cours de l'EPFL "systèmes embarqués et robotique" (MICRO-335)
	Modifié par Julian Bär et Félix Laurent
	Dernière modification: 16.05.2021
 */

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <fft.h>

//-------------------------------------------------------------------------------------------------------------

void doFFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
	
}
