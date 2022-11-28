## Blinky example provided by professor

```c
#include <Arduino.h>
#include <driver/adc.h>
#include <driver/gpio.h>


gpio_num_t LED_PIN = GPIO_NUM_13;
int led_value = 0;
int delayAmount = 1000;

void setup() {
	Serial.begin(115200);


	gpio_reset_pin(LED_PIN);
	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
	delay(1000);
}

void loop() {
	delayAmount = adc1_get_raw(ADC1_CHANNEL_6);
	Serial.println(delayAmount);
	gpio_set_level(LED_PIN, led_value);	
	led_value = !led_value;
  Serial.println(adc1_get_raw(ADC1_CHANNEL_6));
	delay(delayAmount);
}
```