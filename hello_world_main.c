#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/event_groups.h>
#include <esp_system.h>
#include <esp_log.h>
#include "driver/ledc.h"
#include "freertos/queue.h"
#include "driver/adc.h"


// ==== Config pinos ====
// Troquei o botão de emergência para um pino com PU interno
#define COR_GPIO     GPIO_NUM_33
#define EMERG_GPIO   GPIO_NUM_27     // << trocado de 34 p/ 27
#define ANALOG_GPIO ADC1_CHANNEL_7 
// ==== Estado global ====
#define MAX_RPM 6000

volatile int rpm_atual = 0;
volatile int rpm_alvo  = 0;
volatile int emergency = 0;
volatile int motor     = 0;          // valor “oficial” do motor (RPM suavizado)
volatile int count_cores=0;
// ==== Mutex para prints ====
static SemaphoreHandle_t mutual_exclusion_mutex = NULL;

// print seguro
static void print_msg(const char *msg){
    if (mutual_exclusion_mutex) {
        xSemaphoreTake(mutual_exclusion_mutex, portMAX_DELAY);
        printf("%s\n", msg);
        xSemaphoreGive(mutual_exclusion_mutex);
    } else {
        //printf("%s\n", msg);
    }
}

// INICIALIZAÇÕES DE BOTÕES
static void button_emergency_init(void){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << EMERG_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,      // OK no GPIO27
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static void button_cor_init(void){
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << COR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,      // OK no GPIO33
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

static void pot_init(void){
    adc1_config_width(ADC_WIDTH_BIT_12);              
    adc1_config_channel_atten(ANALOG_GPIO, ADC_ATTEN_DB_11);
}
// ====== TASKS ======

// motor_task agora só calcula “motor” (sem LEDC/PWM)
static void motor_task(void *arg){
    const int step = 100;   // rampa máx por tick (RPM)
    int filtro = 0;         // estado interno (motor suavizado)

    while(1){
    

        // AGORA segue o alvo (setpoint), não o "medido"
        int alvo = rpm_alvo;

        int diff = alvo - filtro;
        if (diff > step)       filtro += step;
        else if (diff < -step) filtro -= step;
        else                   filtro  = alvo;

        // Publica estado do motor
        motor     = filtro;
        rpm_atual = motor;   // "medido" = estado atual do motor simulado

        char buf[96];
        snprintf(buf, sizeof(buf),
                 "Motor: medido=%d | alvo=%d | motor=%d | pegos=%d | emergency=%d",
                 rpm_atual, rpm_alvo, motor, count_cores, emergency);
        print_msg(buf);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ENC_SENSE agora só mostra a variável "motor"
void ENC_SENSE(void *pvParameter){
    int tick = 0;
    while(1){
        if (++tick >= 20){ // ~100 ms
            //char buffer[48];
            //snprintf(buffer, sizeof(buffer), "Motor RPM: %4d", motor);
            //print_msg(buffer);
            tick = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


static void SPD_CTRL(void *pvParameter){
    // Filtro IIR simples no domínio do RAW (0..4095)
    int filt_raw = 0;
    const int alpha_num = 1;   // ganho do filtro (1/4)
    const int alpha_den = 4;

    while(1){
        // Se emergência estiver ativa, não sobrescreve o alvo
        if (!emergency) {
            int raw = adc1_get_raw(ANALOG_GPIO);  // 0..4095

            // Filtro IIR: filt += (raw - filt) * (1/4)
            filt_raw += (raw - filt_raw) * alpha_num / alpha_den;

            // Converte para RPM (0..MAX_RPM) com arredondamento
            int alvo = (filt_raw * MAX_RPM + 2047) / 4095;
            if (alvo < 0) alvo = 0;
            if (alvo > MAX_RPM) alvo = MAX_RPM;

            rpm_alvo = alvo;
        }

        // log ocasional para diagnóstico (comente se não quiser)
        // static int tick=0;
        // if ((tick++ % 10)==0) {  // ~500 ms se delay=50 ms
        //     char b[64];
        //     snprintf(b, sizeof(b), "ADC raw=%d  filt=%d  rpm_alvo=%d", raw, filt_raw, rpm_alvo);
        //     print_msg(b);
        // }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void SORT_ACT(void *pvParameter){
    int count=0, antigo=0;
    while(1){
        int bs = gpio_get_level(COR_GPIO);
        if (bs==0 && antigo==1 && emergency == 0){
            count++;
          //  char buf[64];
            count_cores++;
           //snprintf (buf, sizeof(buf),
           //          "[SORT_ACT] Peça detectada! Total=%d", count);
           // print_msg(buf);
        }
        antigo = bs;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void SAFETY_TASK(void *pvParameter){
    int  antigo=0;
    bool temp = true;
    while(1){
        int bs = gpio_get_level(EMERG_GPIO);
        
        if (bs==0 && antigo==1 && emergency == 1) {
                temp = false;
                emergency = 0;
        }
        if (((bs == 0 && antigo==1 ) || emergency == 1) && temp){
            rpm_alvo = 0;
            motor    = 0;
            emergency = 1;
        }
        temp = true;
        antigo = bs;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ====== app_main ======
void app_main(void){
    // 1) CRIE O MUTEX ANTES DE QUALQUER TASK
    mutual_exclusion_mutex = xSemaphoreCreateMutex();
    if (!mutual_exclusion_mutex){
        printf("ERRO: não foi possível criar mutex!\n");
    }

    // 2) Inits 
    button_cor_init();
    button_emergency_init();
    pot_init();
    // 3) Tasks

    xTaskCreate(SPD_CTRL,   "SPD_CTRL",   2048, NULL, 6, NULL);
    xTaskCreate(SAFETY_TASK,"SAFETY_TASK",2048, NULL, 7, NULL);
    xTaskCreate(SORT_ACT,   "SORT_ACT",   2048, NULL, 2, NULL);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 3, NULL);
    xTaskCreate(ENC_SENSE,  "ENC_SENSE",  2048, NULL, 5, NULL);
    
    
}