#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mirf.h"
#include "comms_data.h"

#define ROBOT1 1
#define ROBOT2 2
#define ROBOT3 3
#define CHANNEL CONFIG_RADIO_CHANNEL
#define PAYLOAD 32

#if CONFIG_ROBOT
static uint8_t robot_id;
#endif // CONFIG_ROBOT

NRF24_t dev;


void initRadio();
void send_radio(uint8_t *data);
void task_receive(void *pvParameters);


void app_main(void) {
	initRadio();
#if CONFIG_ROBOT
	xTaskCreate(&task_receive, "TASK_RB_RECEIVE", 1024*3, NULL, 2, NULL);
#endif // CONFIG_ROBOT

#if CONFIG_ESP32_PC
	xTaskCreate(&task_receive, "TASK_PC_RECEIVE", 1024*3, NULL, 2, NULL);
#endif // CONFIG_ESP32_PC

	while (1) {
        // Delay para o intervalo de envio
        vTaskDelay(500 / portTICK_PERIOD_MS);  // 500 ms para periodicidade

#if CONFIG_ROBOT
        // Enviar dados simulados pelo robô
        struct comms_data_motion simulated_data_robot = {
            .id = robot_id,  // ID do robô atual
            .front_left = 1.0f, .front_right = 1.1f,
            .rear_left = 1.2f, .rear_right = 1.3f,
            .kick = 0.5f
        };
        send_radio((uint8_t *)&simulated_data_robot);

#elif CONFIG_ESP32_PC
        // Enviar dados simulados pelo PC
        struct comms_data_electronic simulated_data_pc = {
            .voltage = 3.7f, .current = 1.0f, .temperature = 25.0f
        };
        send_radio(&dev, (uint8_t *)&simulated_data_pc);
#endif // CONFIG_ROBOT / CONFIG_ESP32_PC
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


void initRadio() {
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    Nrf24_init(&dev);
    Nrf24_config(&dev, CHANNEL, PAYLOAD);
    
#if CONFIG_ROBOT
	// Set my own address using 5 characters
	esp_err_t ret = Nrf24_setRADDR(&dev, (uint8_t *)"BROAD");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

#if CONFIG_ROBOT1
	ret = Nrf24_setTADDR(&dev, (uint8_t *)"ROBO1");
	robot_id = ROBOT1;
#elif CONFIG_ROBOT2
	ret = Nrf24_setTADDR(&dev, (uint8_t *)"ROBO2");
	robot_id = ROBOT2;
#elif CONFIG_ROBOT3
	ret = Nrf24_setTADDR(&dev, (uint8_t *)"ROBO3");
	robot_id = ROBOT3;
#endif
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}
#endif // CONFIG_ROBOT

#if CONFIG_ESP32_PC
	esp_err_t ret = Nrf24_setRADDR(&dev, (uint8_t *)"ROBO1");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

	// Add more address using 1 characters
	Nrf24_addRADDR(&dev, ROBOT2, '2'); // ROBO2
	Nrf24_addRADDR(&dev, ROBOT3, '3'); // ROBO3
	
	// Set destination address using 5 characters
	ret = Nrf24_setTADDR(&dev, (uint8_t *)"BROAD");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}
#endif // CONFIG_ESP32_PC

#if CONFIG_ADVANCED
    advancedSettings(&dev);
#endif // CONFIG_ADVANCED

    // Print settings
    Nrf24_printDetails(&dev);
}


void log_received_data(void *data) {
#if CONFIG_ROBOT
    struct comms_data_motion *motion_data = (struct comms_data_motion *)data;
    ESP_LOGI(pcTaskGetName(NULL), "Received data: ID=%"PRIu32", FL=%.2f, FR=%.2f, RL=%.2f, RR=%.2f, Kick=%.2f",
             motion_data->id,
             motion_data->front_left, 
             motion_data->front_right, 
             motion_data->rear_left, 
             motion_data->rear_right, 
             motion_data->kick);
#elif CONFIG_ESP32_PC
    struct comms_data_electronic *electronic_data = (struct comms_data_electronic *)data;
    ESP_LOGI(pcTaskGetName(NULL), "Received data: Voltage=%.2f, Current=%.2f, Temp=%.2f",
             electronic_data->voltage,
             electronic_data->current,
             electronic_data->temperature);
#endif
}


void task_receive(void *pvParameters) {
#if CONFIG_ROBOT
    struct comms_data_motion received_data;
#else
    struct comms_data_electronic received_data;
#endif
	
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
            log_received_data(&received_data); // Print data based on CONFIG
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Evitar WatchDog
    }
}


void send_radio(uint8_t *data) {
	Nrf24_send(&dev, data);
    ESP_LOGI(pcTaskGetName(NULL), "Wait for sending.....");
    if (Nrf24_isSend(&dev, 1000)) {
        ESP_LOGI(pcTaskGetName(NULL),"Send success:%d", *data);
    } else {
        ESP_LOGW(pcTaskGetName(NULL),"Send fail:");
    }
}
