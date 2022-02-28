/*
 * amt_inc_enc.c
 *
 *  Created on: Feb 28, 2022
 *      Author: ray
 */

#include "amt_inc_enc.h"
#include "math.h"

#define TIM_COUNT_MAX 32767
#define ENCODER_MAX_PPR 8192.0 //Max pulse per resolution
void AMT_Inc_Init(inc_enc_t* inc_enc, TIM_HandleTypeDef *htim){
    memset(inc_enc, 0, sizeof(*inc_enc));
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

int32_t AMT_ReadEncoder(inc_enc_t* inc_enc, int16_t tim_count){
    //If first time entering this function, store value and exit
    if (inc_enc->prev_t == 0 && inc_enc->prev_tim_count == 0){ //might  cause bug this line
	inc_enc->prev_tim_count  = tim_count;
	inc_enc->enc = tim_count;
	return inc_enc->enc;
    }
    int16_t diff_tim_count = tim_count - inc_enc->prev_tim_count;

    inc_enc->enc += diff_tim_count;
    inc_enc->prev_tim_count  = tim_count;

    return inc_enc->enc;
}

float AMT_CalVelocity(inc_enc_t* inc_enc){
    if (inc_enc->prev_t == 0 && inc_enc->prev_enc == 0){
	inc_enc->prev_enc = inc_enc->enc;
	inc_enc->prev_t = HAL_GetTick();
	return 0;
    }

    //Make sure systick at 1kHz
    float freq = (HAL_GetTickFreq() == HAL_TICK_FREQ_1KHZ) ? 1000.0 : 0;
    if (freq == 0)
	Error_Handler();

    float dt = (float)(HAL_GetTick() - inc_enc->prev_t) / freq;

    inc_enc->velocity = (((float)(inc_enc->enc - inc_enc->prev_enc)/ENCODER_MAX_PPR) * 2 * M_PI )/ dt;
    inc_enc->prev_t = HAL_GetTick();
    inc_enc->prev_enc = inc_enc->enc;
    return inc_enc->velocity;
}
