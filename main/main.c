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

#if CONFIG_ROBOT
static const uint8_t robot_id;
struct comms_data_motion received_data;
#endif // CONFIG_ROBOT

void initRadio(NRF24_t *dev, uint8_t channel, uint8_t payload);
void send_radio();
void task_receive(void *pvParameters);


void app_main(void) {
#if CONFIG_ROBOT
	xTaskCreate(&task_receive, "TASK_RB_RECEIVE", 1024*3, NULL, 2, NULL);
#endif // CONFIG_ROBOT

#if CONFIG_ESP32_PC
	xTaskCreate(&task_receive, "TASK_PC_RECEIVE", 1024*3, NULL, 2, NULL);
#endif // CONFIG_ESP32_PC
}


void send_radio(){}


#if CONFIG_ADVANCED
void advancedSettings(NRF24_t * dev) {
    ESP_LOGW(pcTaskGetName(NULL), "Set RF Data Ratio");
    Nrf24_SetSpeedDataRates(dev, CONFIG_RF_DATA_RATE);

    ESP_LOGW(pcTaskGetName(NULL), "CONFIG_RETRANSMIT_DELAY=%d", CONFIG_RETRANSMIT_DELAY);
    Nrf24_setRetransmitDelay(dev, CONFIG_RETRANSMIT_DELAY);
}
#endif // CONFIG_ADVANCED

//TODO: check best structure to start the radio
void initRadio(NRF24_t *dev, uint8_t channel, uint8_t payload) {
    ESP_LOGI(pcTaskGetName(NULL), "Start");

    Nrf24_init(dev);
    Nrf24_config(dev, channel, payload);
    
#if CONFIG_ROBOT
	// Set my own address using 5 characters
	esp_err_t ret = Nrf24_setRADDR(&dev, (uint8_t *)"BROAD");
	if (ret != ESP_OK) {
		ESP_LOGE(pcTaskGetName(NULL), "nrf24l01 not installed");
		while(1) { vTaskDelay(1); }
	}

#if CONFIG_ROBOT1
	esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *)"ROBO1");
	robot_id = ROBOT1;
#elif CONFIG_ROBOT2
	esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *)"ROBO2");
	robot_id = ROBOT2;
#elif CONFIG_ROBOT3
	esp_err_t ret = Nrf24_setTADDR(&dev, (uint8_t *)"ROBO3");
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

	// Add my own address using 1 characters
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
    advancedSettings(dev);
#endif // CONFIG_ADVANCED

    // Print settings
    Nrf24_printDetails(dev);
}

// TODO: receiving task for robot and pc
void task_receive(void *pvParameters) {
    NRF24_t dev;
    uint8_t payload = 32;
    uint8_t channel = CONFIG_RADIO_CHANNEL;
    
    initRadio(&dev, channel, payload);    
    
    ESP_LOGI(pcTaskGetName(NULL), "TASK RECEIVE...");

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
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);  // Evitar WatchDog
    }
}

// TODO: modify the way data is sent (maybe receive DEV and DATA type or UINT*)
void task_send(void *pvParameters) {
    NRF24_t dev;
    uint8_t channel = CONFIG_RADIO_CHANNEL;
    uint8_t payload = 32;
    initRadio(&dev, channel, payload);
    
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
