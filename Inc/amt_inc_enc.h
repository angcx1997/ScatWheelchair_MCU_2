/*
 * amt_inc_enc.h
 *
 *  Created on: Feb 28, 2022
 *      Author: ray
 */

#ifndef INC_AMT_INC_ENC_H_
#define INC_AMT_INC_ENC_H_

#include "main.h"

typedef struct{
    uint32_t prev_t;
    int16_t prev_tim_count; //get from timer input capture
    int32_t prev_enc; 	//use to calculate velocity
    int32_t enc;
    float velocity; //angular velocity
}inc_enc_t;

void AMT_Inc_Init(inc_enc_t* inc_enc, TIM_HandleTypeDef *htim);
int32_t AMT_ReadEncoder(inc_enc_t* inc_enc, int16_t tim_count);
float AMT_CalVelocity(inc_enc_t* inc_enc);
#endif /* INC_AMT_INC_ENC_H_ */
