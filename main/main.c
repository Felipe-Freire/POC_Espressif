#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "mirf.h"

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
	AdvancedSettings(dev);
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
    
	ESP_LOGI(pcTaskGetName(NULL), "Listening...");

	uint8_t buf[32];

	// Clear RX FiFo
	while(1) {
		if (Nrf24_dataReady(&dev) == false) break;
		Nrf24_getData(&dev, buf);
	}

	while(1) {
		// Wait for received data
		if (Nrf24_dataReady(&dev)) {
			Nrf24_getData(&dev, buf);
			ESP_LOGI(pcTaskGetName(NULL), "Got data:%d [%s]", buf[0], &buf[1]);
			//ESP_LOG_BUFFER_HEXDUMP(pcTaskGetName(NULL), buf, payload, ESP_LOG_INFO);
		}
		vTaskDelay(1); // Avoid WatchDog alerts
	}
}
#endif // CONFIG_RECEIVER


#if CONFIG_SENDER
void sender(void *pvParameters) {
	NRF24_t dev;
    uint8_t channel = CONFIG_RADIO_CHANNEL;
    uint8_t payload = 32;
    initRadio(&dev, channel, payload, (uint8_t *)"FGHIJ");
    
	while(1) {
		TickType_t nowTick = xTaskGetTickCount();
		buf[0] = index;
		sprintf((char *)&buf[1], "Hello World %"PRIu32, nowTick);
		Nrf24_send(&dev, buf);
		//vTaskDelay(1);
		ESP_LOGI(pcTaskGetName(NULL), "Wait for sending.....");
		if (Nrf24_isSend(&dev, 1000)) {
			ESP_LOGI(pcTaskGetName(NULL),"Send success:%d [%s]", buf[0], &buf[1]);
			index++;
		} else {
			ESP_LOGW(pcTaskGetName(NULL),"Send fail:");
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
#endif // CONFIG_SENDER


void app_main(void) {
#if CONFIG_RECEIVER
	xTaskCreate(&receiver, "RECEIVER", 1024*3, NULL, 2, NULL);
#endif

#if CONFIG_SENDER
	xTaskCreate(&sender, "SENDER", 1024*3, NULL, 2, NULL);
#endif

}