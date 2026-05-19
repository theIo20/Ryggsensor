#include <math.h>
#include "gd32vf103.h"
#include "delay.h"
#include "gd32v_mpu6500_if.h"
#include "gd32v_tf_card_if.h"
#include "stdio.h"
#include "string.h"
#include "systick.h"
#include "usb_serial_if.h"
#include "usb_delay.h"

#define VIBRATOR_PIN        GPIO_PIN_5
#define MAX_AVVIKELSER      300

typedef struct {
    uint32_t minuter;
    uint32_t sekunder;
    float avvikelse;
} avvikelse_t;

avvikelse_t logg[MAX_AVVIKELSER];
int antal_avvikelser = 0;
volatile uint32_t millis = 0;

FATFS fs;
FIL fil;
UINT bw;
int sd_monterad = 0;
uint32_t senaste_sparning = 0;

void sd_init(void) {
    set_fattime(1980, 1, 1, 0, 0, 0);
    delay_1ms(100);
    FRESULT fr = f_mount(&fs, "", 1);
    if (fr == FR_OK) {
        sd_monterad = 1;
    }
}

void spara_logg(void) {
    if (!sd_monterad || antal_avvikelser == 0) return;

    FRESULT fr = f_open(&fil, "LOGG.CSV", FA_WRITE | FA_OPEN_APPEND);
    if (fr != FR_OK) return;

    if (f_size(&fil) == 0) {
        f_write(&fil, "Minuter,Sekunder,Avvikelse\n", 27, &bw);
    }

    char rad[64];
    for (int i = 0; i < antal_avvikelser; i++) {
        int len = snprintf(rad, sizeof(rad), "%lu,%lu,%d\n",
            logg[i].minuter,
            logg[i].sekunder,
            (int)logg[i].avvikelse);
        f_write(&fil, rad, len, &bw);
    }

    f_sync(&fil);
    f_close(&fil);
    antal_avvikelser = 0;
}

void interrupt_config(void) {
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
    timer_ocinitpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocinitpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocinitpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_channel_output_config(TIMER1, TIMER_CH_0, &timer_ocinitpara);

    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 54000/2);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_TIMING);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_interrupt_enable(TIMER1, TIMER_INT_CH0);
    timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH0);
    timer_enable(TIMER1);
}

void TIMER1_IRQHandler(void) {
    if (SET == timer_interrupt_flag_get(TIMER1, TIMER_INT_CH0)) {
        millis++;
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_CH0);
    }
}

int main(void) {
    mpu_vector_t vec, vec_temp;
    float nollvinkel = 0.0;
    int har_kalibrerat = 0;
    int utanfor = 0;
    uint32_t millis_kopia = 0; // ÄNDRAT: Variabel för att spara en säker kopia av tiden

    configure_usb_serial();
    while (!usb_serial_available()) usb_delay_1ms(100);
    usb_delay_1ms(1000);

    rcu_periph_clock_enable(RCU_GPIOA);
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_5);

    rcu_periph_clock_enable(RCU_GPIOB);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, VIBRATOR_PIN);

    rcu_periph_clock_enable(RCU_I2C0);
    gpio_init(GPIOB, GPIO_MODE_AF_OD, GPIO_OSPEED_50MHZ, GPIO_PIN_6 | GPIO_PIN_7);
    mpu6500_install(I2C0);

    sd_init();

    mpu6500_getAccel(&vec_temp);

    eclic_global_interrupt_enable();
    interrupt_config();

    while (1) {
        mpu6500_getAccel(&vec);
        
        // ÄNDRAT: Pythagoras sats kombinerar Z och X för att skapa en stabil bas när man vrider sig sidledes
        float stabil_bas = sqrt((vec.z * vec.z) + (vec.x * vec.x)); 
        
        // ÄNDRAT: Använder nu den stabila basen. Förhindrar flippande vinklar vid sidorörelser
        float nuvarande_vinkel = -atan2(vec.y, stabil_bas) * (180.0 / M_PI); 

        if (gpio_input_bit_get(GPIOA, GPIO_PIN_5) == 0) {
            nollvinkel = nuvarande_vinkel;
            har_kalibrerat = 1;
            gpio_bit_set(GPIOB, GPIO_PIN_0);
            delay_1ms(50);
        } else {
            gpio_bit_reset(GPIOB, GPIO_PIN_0);
        }

        float avvikelse = nuvarande_vinkel - nollvinkel;

        printf("X: %d  Y: %d  Z: %d\n", (int)vec.x, (int)vec.y, (int)vec.z);
        printf("Avvikelse: %d  Nollvinkel: %d\n", (int)avvikelse, (int)nollvinkel);
        fflush(0);

        usb_delay_1ms(100);

        if (har_kalibrerat == 1) {

            if (avvikelse > 25) {
                gpio_bit_set(GPIOB, VIBRATOR_PIN);
                if (utanfor == 0) {
                    utanfor = 1;
                    if (antal_avvikelser < MAX_AVVIKELSER) {
                        
                        // ÄNDRAT: Stänger av avbrott kort för att säkert läsa av millis utan krockar
                        eclic_global_interrupt_disable(); 
                        millis_kopia = millis;            
                        eclic_global_interrupt_enable();  // ÄNDRAT: Slår på avbrott direkt igen
                        
                        // ÄNDRAT: Använder nu millis_kopia istället för millis (fixar "197-problemet" i Excel)
                        logg[antal_avvikelser].minuter  = millis_kopia / 60000;
                        logg[antal_avvikelser].sekunder = (millis_kopia % 60000) / 1000;
                        logg[antal_avvikelser].avvikelse = avvikelse;
                        antal_avvikelser++;
                    }
                    if (antal_avvikelser >= MAX_AVVIKELSER) {
                        spara_logg();
                        
                        // ÄNDRAT: Säker uppdatering av senaste_sparning med skyddade tids-kopian
                        senaste_sparning = millis_kopia; 
                    }
                }
            } else if (avvikelse < 20) {
                gpio_bit_reset(GPIOB, VIBRATOR_PIN);
                utanfor = 0;
            }

            // ÄNDRAT: Gör en säker tidsavläsning även för den automatiska 20-sekunderssparandet
            eclic_global_interrupt_disable();
            uint32_t nuvarande_tid = millis;
            eclic_global_interrupt_enable();

            if (nuvarande_tid - senaste_sparning >= 20000) {
                spara_logg();
                senaste_sparning = nuvarande_tid; // ÄNDRAT: Uppdaterar spar-tiden med det säkra värdet
            }
        }
    }
}