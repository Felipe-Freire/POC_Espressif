#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mirf.h"
#include "esp_simplefoc.h"
#include "comms_data.h"

#if CONFIG_SOC_MCPWM_SUPPORTED
#define USING_MCPWM
#endif

typedef struct {
    BLDCMotor *motor;
    BLDCDriver3PWM *driver;
} MotorTaskParams;

BLDCMotor motor1 = BLDCMotor(14);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(17, 16, 15);

BLDCMotor motor2 = BLDCMotor(19);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(18, 22, 23);

void initializeMotor(BLDCMotor *motor, BLDCDriver3PWM *driver);
void motorTask(void *pvParameters);

#if CONFIG_ADVANCED
void advancedSettings(NRF24_t * dev);
#endif
void initRadio(NRF24_t *dev, uint8_t channel, uint8_t payload, uint8_t *addr);

#if CONFIG_RECEIVER
void receiver(void *pvParameters);
#endif

#if CONFIG_SENDER
void sender(void *pvParameters);
#endif

extern "C" void app_main(void) {
/*	MotorTaskParams motor1Params = { &motor1, &driver1 };
    MotorTaskParams motor2Params = { &motor2, &driver2 };
    
    xTaskCreate(motorTask, "Motor 1 Control Task", 2048, (void *)&motor1Params, 5, NULL);
    xTaskCreate(motorTask, "Motor 2 Control Task", 2048, (void *)&motor2Params, 5, NULL);
*/	
#if CONFIG_RECEIVER
	xTaskCreate(&receiver, "RECEIVER", 1024*3, NULL, 2, NULL);
#endif

#if CONFIG_SENDER
	xTaskCreate(&sender, "SENDER", 1024*3, NULL, 2, NULL);
#endif

}

// Função para inicializar e configurar um motor
void initializeMotor(BLDCMotor &motor, BLDCDriver3PWM &driver) {
    driver.voltage_power_supply = 12;
    driver.voltage_limit = 11;

#ifdef USING_MCPWM
    driver.init(0);
#else
    driver.init({1, 2, 3});
#endif
    motor.linkDriver(&driver);

    motor.velocity_limit = 200.0;
    motor.voltage_limit = 12.0;
    motor.controller = MotionControlType::velocity_openloop;

    motor.init();
}

// Função da task que controla o movimento do motor
void motorTask(void *pvParameters) {
    BLDCMotor *motor = (BLDCMotor *)pvParameters;

    // Inicializar o motor associado a essa task
    if (motor == &motor1) {
        initializeMotor(*motor, driver1);
    } else if (motor == &motor2) {
        initializeMotor(*motor, driver2);
    }

    // Loop da task
    while (1) {
        motor->move(1.2f);  // Exemplo de movimento: 1.2 rad/s
        vTaskDelay(100 / portTICK_PERIOD_MS);  // Pequeno delay
    }
}

#if CONFIG_ADVANCED
void advancedSettings(NRF24_t * dev) {
    ESP_LOGW(pcTaskGetName(NULL), "Set RF Data Ratio");
    Nrf24_SetSpeedDataRates(dev, CONFIG_RF_DATA_RATE);

    ESP_LOGW(pcTaskGetName(NULL), "CONFIG_RETRANSMIT_DELAY=%d", CONFIG_RETRANSMIT_DELAY);
    Nrf24_setRetransmitDelay(dev, CONFIG_RETRANSMIT_DELAY);
}
#endif // CONFIG_ADVANCED

void initRadio(NRF24_t *dev, uint8_t channel, uint8_t payload, uint8_t *addr) {
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    Nrf24_init(dev);
    Nrf24_config(dev, channel, payload);

    // Set destination address using 5 characters
    esp_err_t ret = Nrf24_setTADDR(dev, (uint8_t *)"FGHIJ");
    if (ret != ESP_OK) {
        ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
        while(1) { vTaskDelay(1); }
    }
    
#if CONFIG_RECEIVER
    Nrf24_setRADDR(dev, addr);
#endif

#if CONFIG_SENDER
    Nrf24_setTADDR(dev, addr);
#endif

#if CONFIG_ADVANCED
    advancedSettings(dev);
#endif // CONFIG_ADVANCED

    // Print settings
    Nrf24_printDetails(dev);
}

#if CONFIG_RECEIVER
void receiver(void *pvParameters) {
    NRF24_t dev;
    uint8_t payload = 32;
    uint8_t channel = CONFIG_RADIO_CHANNEL;
    initRadio(&dev, channel, payload, (uint8_t *)"FGHIJ");
    
    struct comms_data_motion received_data;
    
    ESP_LOGI(pcTaskGetName(NULL), "Listening...");

    // Clear RX FiFo
    while(1) {
        if (Nrf24_dataReady(&dev) == false) break;
        Nrf24_getData(&dev, (uint8_t*)&received_data);
    }

    while(1) {
        // Wait for received data
        if (Nrf24_dataReady(&dev)) {
            Nrf24_getData(&dev, (uint8_t*)&received_data);

            // Log - data receive
            ESP_LOGI(pcTaskGetName(NULL), "Received data: ID=%d, FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f, Kick=%.2f",
                     received_data.id, 
                     received_data.front_left, 
                     received_data.front_right, 
                     received_data.rear_left, 
                     received_data.rear_right, 
                     received_data.kick);

            // Control motor
            motor.move(received_data.front_left);  // Exemplo: aplicar a velocidade do front_left ao motor
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Evitar WatchDog
    }
}
#endif // CONFIG_RECEIVER

#if CONFIG_SENDER
void sender(void *pvParameters) {
    NRF24_t dev;
    uint8_t channel = CONFIG_RADIO_CHANNEL;
    uint8_t payload = 32;
    initRadio(&dev, channel, payload, (uint8_t *)"FGHIJ");
    
    struct comms_data_motion data_to_send;
    
    // Preencher a estrutura com dados
        data_to_send.front_left = 1.0;
        data_to_send.front_right = 2.0;
        data_to_send.rear_left = 3.0;
        data_to_send.rear_right = 4.0;
        data_to_send.kick = 0.0;
        data_to_send.id = 12345;
        
    while(1) {
        Nrf24_send(&dev, (uint8_t*)&data_to_send);
        ESP_LOGI(pcTaskGetName(NULL), "Wait for sending.....");
        if (Nrf24_isSend(&dev, 1000)) {
            ESP_LOGI(pcTaskGetName(NULL),"Send success:%d [%s]", data_to_send.id);
            index++;
        } else {
            ESP_LOGW(pcTaskGetName(NULL),"Send fail:");
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}
#endif // CONFIG_SENDER
