/*!
    \file  main.c
    \brief USB CDC ACM device

    \version 2019-6-5, V1.0.0, demo for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include <math.h>
#include "gd32vf103.h"
#include "lcd.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"


#define GRAPH_HEIGHT    30
#define VIBRATOR_PIN GPIO_PIN_5
#define  MAX_AVVIKELSER 300

typedef struct {
    uint32_t minuter;
    uint32_t sekunder;
    float avvikelse;
} avvikelse_t;

avvikelse_t logg[MAX_AVVIKELSER];
int antal_avvikelser = 0;

volatile uint32_t millis = 0;
void interrupt_config(void){
    timer_oc_parameter_struct timer_ocinitpara;
    timer_parameter_struct timer_initpara;

    eclic_irq_enable(TIMER1_IRQn, 1, 0);
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    timer_struct_para_init(&timer_initpara);

    timer_initpara.prescaler        = 1;
    timer_initpara.alignedmode      = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period           = 54000;
    timer_initpara.clockdivision    = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    timer_channel_output_struct_para_init(&timer_ocinitpara);
    timer_ocinitpara.outputstate    = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity     = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate    = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 54000/2);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_TIMING);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_interrupt_enable(TIMER1, TIMER_INT_CH0);
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH0);
    timer_enable(TIMER1);
}


void TIMER1_IRQHandler(void)
{
    if (SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_CH0)) {
        millis++;
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH0);
    }
}


int main(void)
{
    /* The related data structure for the IMU, contains a vector of x, y, z floats*/
    mpu_vector_t vec, vec_temp;
    uint16_t line_color;        /* for lcd */
    float nollvinkel = 0.0;
    int har_kalibrerat = 0;
    int utanfor = 0;
        
    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_1);

    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, VIBRATOR_PIN);

    /* Initialize pins for I2C */
    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    /* Initialize the IMU (Notice that MPU6500 is referenced, this is due to the fact that ICM-20600
       ICM-20600 is mostly register compatible with MPU6500, if MPU6500 is used only thing that needs 
       to change is MPU6500_WHO_AM_I_ID from 0x11 to 0x70. */
    mpu6500_install(I2C0);
    
    /* Initialize LCD */
    Lcd_SetType(LCD_INVERTED);
    Lcd_Init();
    LCD_Clear(BLACK);
    
    mpu6500_getAccel(&vec_temp);

    eclic_global_interrupt_enable();
    timer_interupt_config();
    
    while(1){
        /* Get accelleration data (Note: Blocking read) puts a force vector with 1G = 4096 into x, y, z directions respectively */
        //gpio_bit_set(GPIOB, GPIO_PIN_0);
        
        mpu6500_getAccel(&vec);
        float nuvarande_vinkel = -atan2(vec.y, vec.z) * (180.0 / M_PI);

        if (gpio_input_bit_get(GPIOA, GPIO_PIN_1) == 0) {
            nollvinkel = nuvarande_vinkel;
            har_kalibrerat = 1;

            delay_1ms(200);
        }

        float avvikelse = nuvarande_vinkel - nollvinkel;
        
        if (har_kalibrerat==1) {
            /* If the angle is greater than 20 degrees, turn on the LED */
            if (fabs(avvikelse) >15){
                gpio_bit_reset(GPIOB, GPIO_PIN_0);
                gpio_bit_reset(GPIOB, VIBRATOR_PIN);

                if (utanfor==0) {
                    utanfor = 1;
                    if (antal_avvikelser < MAX_AVVIKELSER) {
                        logg[antal_avvikelser].minuter = millis / 60000;
                        logg[antal_avvikelser].sekunder = (millis % 60000) / 1000;
                        logg[antal_avvikelser].avvikelse = avvikelse;
                        antal_avvikelser++;
                    }
                }
            } else {
               /* Otherwise, turn off the LED */
                gpio_bit_set(GPIOB, GPIO_PIN_0);
                gpio_bit_set(GPIOB, VIBRATOR_PIN);
            }
        }
        
  
        //gpio_bit_set(GPIOB, VIBRATOR_PIN);


        /* Do some fancy math to make a nice display */

        /* Green if pointing up, red if down */
        line_color = (vec.z < 0) ? RED : GREEN;
        /* Draw a unit circle (1G) */
        Draw_Circle(160/2, 80/2, 28, BLUE);
        /* Erase last line */
        LCD_DrawLine(160/2, 80/2, (160/2)+(vec_temp.y)/(4096/28), (80/2)+(vec_temp.x/(4096/28)),BLACK);
        /* Draw new line, scaled to the unit circle */
        LCD_DrawLine(160/2, 80/2, (160/2)+(vec.y)/(4096/28), (80/2)+(vec.x/(4096/28)),line_color);
        /* Store the last vector so it can be erased */
        vec_temp = vec;
        /* Wait for LCD to finish drawing */
        LCD_Wait_On_Queue();
    }
}