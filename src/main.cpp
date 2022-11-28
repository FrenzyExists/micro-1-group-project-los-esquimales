#include <Arduino.h>
#include <driver/adc.h>
#include <driver/gpio.h>

#define _A GPIO_NUM_14 
#define _B GPIO_NUM_27
#define _C GPIO_NUM_33
#define _D GPIO_NUM_25
#define _E GPIO_NUM_26
#define _F GPIO_NUM_32
#define _G GPIO_NUM_35

#define SEGMENTGND1 GPIO_NUM_19
#define SEGMENTGND2 GPIO_NUM_15

gpio_num_t commons[] = {
  SEGMENTGND1, SEGMENTGND2
};
int commonLength = sizeof(commons)/sizeof(commons[0]);

#define TEMP_PIN

int taskDelay = 1000; // Set to 1 second


gpio_num_t segments[] = {
	_A, _B, _C, _D, _E, _F, _G
};
int segmentLength = sizeof(segments)/sizeof(segments[0]);

int displayDigits[] = {
  0b00111111, /* 0 */
  0b00000110, /* 1 */
  0b01011011, /* 2 */
  0b01001111, /* 3 */
  0b01100110, /* 4 */
  0b01101101, /* 5 */
  0b01111101, /* 6 */
  0b00000111, /* 7 */
  0b01111111, /* 8 */
  0b01100111  /* 9 */
};

#define A_BIT 1
#define B_BIT 2
#define C_BIT 4
#define D_BIT 8
#define E_BIT 16
#define F_BIT 32
#define G_BIT 64
int digitPosition[] = {A_BIT, B_BIT, C_BIT, D_BIT, E_BIT, F_BIT, G_BIT};


void resetDisplay() {
	for (int i = 0; i < segmentLength ; i++) {
		gpio_set_level(segments[i], 1);
	}
	for (int i = 0 ; i < commonLength ; i++) {
		gpio_set_level(commons[i], 0);
	}
}

void initDisplays() {
	// Turn on segment 
	gpio_reset_pin(SEGMENTGND1);
  gpio_set_direction(SEGMENTGND1, GPIO_MODE_OUTPUT);

	gpio_reset_pin(SEGMENTGND2);
  gpio_set_direction(SEGMENTGND2, GPIO_MODE_OUTPUT);

	for (int i = 0 ; i < segmentLength ; i++ ) {
		gpio_reset_pin(segments[i]);
    gpio_set_direction(segments[i], GPIO_MODE_OUTPUT);
	}
	resetDisplay();
}


void taskTemperature(void * parameters) {

}

void taskDisplayTemperature(void * parameters) {
  
}

void setup() {
  Serial.begin(115200);
  // initDisplays();

  // Displays
  xTaskCreate(
    taskDisplayTemperature,
    "Display",
    1000,
    NULL,
    1,
    NULL
  );
  
  // Temperature
  xTaskCreate(
    taskTemperature,
    "Display",
    1000,
    NULL,
    1,
    NULL
  );


}


void loop() {

}


// gpio_num_t LED_PIN = GPIO_NUM_13;
// int led_value = 0;
// int delayAmount = 1000;

// void setup() {
// 	Serial.begin(115200);


// 	gpio_reset_pin(LED_PIN);
// 	gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
// 	adc1_config_width(ADC_WIDTH_BIT_12);
// 	adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
// 	delay(1000);
// }

// void loop() {
// 	delayAmount = adc1_get_raw(ADC1_CHANNEL_6);
// 	Serial.println(delayAmount);
// 	gpio_set_level(LED_PIN, led_value);	
// 	led_value = !led_value;
//   Serial.println(adc1_get_raw(ADC1_CHANNEL_6));
// 	delay(delayAmount);
// }