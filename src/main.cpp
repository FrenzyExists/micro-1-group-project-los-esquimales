#include <Arduino.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include <driver/gpio.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <math.h>
#include <Preferences.h>

/**
 * Theory behind sensor
 * 
 * Vsense = Voltage across the termistor into the ADC input
 * LSB    = Least significant bit digital value that is
 *          converted from the ADC input (12-bit most common)
 * 2^ADC resolution -> 2^12bits in our case or 4096
 * VREF  = The input voltage that is put into the termistor
*/



/********************************************
**************** GPIO PINS ****************
*********************************************/
#define THERMISTOR_INPUT      ADC1_CHANNEL_7 // 25
#define BUTTON_1              GPIO_NUM_23
#define BUTTON_2              GPIO_NUM_1
#define _A                    GPIO_NUM_5 
#define _B                    GPIO_NUM_27
#define _C                    GPIO_NUM_32
#define _D                    GPIO_NUM_25
#define _E                    GPIO_NUM_26
#define _F                    GPIO_NUM_18
#define _G                    GPIO_NUM_19

#define DIGI                  GPIO_NUM_33

/********************************************
****** CONSTANTS N PREPROCESSOR STUFF *******
*********************************************/

//ADC Attenuation
#define ADC_ATTEN ADC_ATTEN_DB_11

// ADC Calibration
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_DEFAULT_VREF

/********************************************
************ DISPLAY CONST STUFF *************
*********************************************/

const gpio_num_t segments[] = {
	_A, _B, _C, _D, _E, _F, _G
};

const displayDigits[] = {
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


/********************************************
**************** MQTT CONFIG ****************
*********************************************/
#define mqtt_server           "3.23.192.83"
#define mqtt_port             1883
#define mqtt_user             "esquimalesAC"
#define mqtt_pass             "EsquimalesAC4206"

#define root_topic_subscribe  "Temp"
#define root_topic_publish    "Temp"
#define root_topic_recieve    "ESP"

/********************************************
**************** VARS N STUFF ***************
*********************************************/

WiFiClient espClient;
PubSubClient client(espClient);
char msg[25];
String Payload;
long count = 0;


int VsenseRaw = 0;
float ADCVoltage = 0.0;
float TempC_Sensor1 = 0.0;
float TempF_Sensor1 = 0.0;
float TempK_Sensor1 = 0.0;
int refTemp = 25; // Can be overriden by the MQTT
double Rt = 0.0;
int Beta =  3950; // Can be overriden by the MQTT

/********************************************
**************** WIFI CONFIG ****************
*********************************************/

char* ssid = "LIB-2734302";
char* password = "xfYbdx3N7dyt";


Preferences preferences;  // Init Preference to store data permanently

/********************************************
****************  FUNCTIONS  ****************
*********************************************/

void resetDisplay() {
	for (int i = 0; i < segmentLength ; i++) {
		// digitalWrite(segments[i], HIGH);
    GPIO.out_w1ts = ((uint32_t)1 << segments[i]); // Black Magic alternative
	}
	for (int i = 0 ; i < commonLength ; i++) {
		// digitalWrite(commons[i], LOW);
    GPIO.out_w1ts = ((uint32_t)0 << segments[i]);
	}
}

void displayOneDigit(int n,int pos) {
  for (int i = 0; i < segmentLength; i++) {
    if (displayDigits[n] & digitPosition[i]) 
      // digitalWrite(segments[i], LOW);
      GPIO.out_w1ts = ((uint32_t)0 << segments[i]);
    }
    // digitalWrite(commons[pos], HIGH);
    GPIO.out_w1ts = ((uint32_t)1 << common[pos]);
}


uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  esp_err_t DEFAULT_VREF = adc_vref_to_gpio(ADC_UNIT_1, GPIO_NUM_35);

  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);  

  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}


/*******************************************
**************** TASKS AND STUFF ***********
*******************************************/


void taskMeasureTemperature(void * parameters) {
    VsenseRaw = adc1_get_raw(THERMISTOR_INPUT);

    ADCVoltage = readADC_Cal(VsenseRaw);
    
    TempK_Sensor1 =  1 / (1/(273.15 + refTemp) + log(((ADCVoltage/10) / 0.02)/10000) / Beta);
  
    TempC_Sensor1 = TempK_Sensor1 - 273.15;
    TempF_Sensor1 = (TempK_Sensor1 - 273.15) * 9/5 + 32;    

    Serial.println("\n");
    Serial.print("ADC ");
    Serial.println(ADCVoltage);
    Serial.print("RAW ");
    Serial.println(VsenseRaw);

    Serial.print("Temperature in Kelvin: ");
    Serial.println(TempK_Sensor1);
    Serial.print("Temperature in Celsius: ");
    Serial.println(TempC_Sensor1);
    Serial.print("Temperature in Farenheit: ");
    Serial.println(TempF_Sensor1);
    vTaskDelay(1000);
}

void taskDisplay() {

  state = !state;
  if (state) {
    displayOneDigit(abs(floor(numberDisplay % 10)), 0);
  } else {
    displayOneDigit(abs(floor(numberDisplay / 10)), 1);
  }
  resetDisplay();
  vTaskDelay(100);

}

void setup() {
  Serial.begin(115200);
	adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(THERMISTOR_INPUT, ADC_ATTEN);

  Serial.println("WTF IS GOING ON");

  // Temp Task
  xTaskCreatePinnedToCore(
    taskMeasureTemperature,
    "Temperature",
    10000,
    NULL,
    1,
    NULL,
    0);                         

  // Display Task
  xTaskCreatePinnedToCore(
    taskDisplay,
    "Display",
    10000,
    NULL,
    1,
    NULL,
    0);

  // Wifi Task

  // Button Task


  delay(150);
}



// Keep this empty
void loop() {

}