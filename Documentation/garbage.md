#include <Arduino.h>
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
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
double temperature = 0.00;

#define TEMP_PIN ADC1_CHANNEL_6

int taskDelay = 1000; // Set to 1 second

static const char *TAG = "ADC SINGLE";

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

//ADC Attenuation
#define ADC_ATTEN ADC_ATTEN_DB_11

// ADC Calibration
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_DEFAULT_VREF


static esp_adc_cal_characteristics_t adc_chars;

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

bool adc_calibration_init() {
  esp_err_t ret = esp_adc_cal_check_efuse(ADC_CALI_SCHEME);
  bool cali_enable = false;

  if (ret = ESP_OK) {
    cali_enable = true;
  } else {
    ESP_LOGE(TAG, "ERROR, you fucked up");
  }

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 0, &adc_chars);
  Serial.println("INITIATING CALIBRATION DONE");
  return cali_enable;
}


void taskTemperature() {
  temperature = adc1_get_raw(ADC1_CHANNEL_6);
  ESP_LOGI(TAG, "raw  data: %d", temperature);
  Serial.printf("WOW\n");
  Serial.println(temperature);

  vTaskDelay(2500/portTICK_PERIOD_MS); // Basically the way set the delay in our tasks
}

void taskDisplayTemperature(void * parameters) {
 vTaskDelay(1000/portTICK_PERIOD_MS);   
}

void setup() {
  Serial.begin(9600);
  // initDisplays();

  // Displays
  // xTaskCreate(
  //   taskDisplayTemperature,
  //   "Display",
  //   1000,
  //   NULL,
  //   1,
  //   NULL
  // );
  
  // Temperature
  adc1_config_width(ADC_WIDTH_BIT_12);
  // The input voltage of ADC will be attenuated extending the range of measurement by about 6 dB (2 x) 
  adc1_config_channel_atten(TEMP_PIN, ADC_ATTEN);
  bool cali_enable = adc_calibration_init();

  


}

void loop() {
taskTemperature();
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












```Cpp

#include <Arduino.h>
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include <driver/gpio.h>
 
#define LM35_Sensor1    ADC1_CHANNEL_7
#define MAX_VREF_VOLTAGE 3129.00
#define INPUT_VOLTAGE 5
#define DEFAULT_ROOM_TEMP 25

int LM35_Raw_Sensor1 = 0;
float LM35_TempC_Sensor1 = 0.0;
float LM35_TempF_Sensor1 = 0.0;
float Voltage = 0.0;
float V = 0.0;
float Rt = 0.0;
int delayAmount = 1000;

//ADC Attenuation
#define ADC_ATTEN ADC_ATTEN_DB_11

// ADC Calibration
#define ADC_CALI_SCHEME ESP_ADC_CAL_VAL_DEFAULT_VREF

uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;
  
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  // esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_err_t DEFAULT_VREF = adc_vref_to_gpio(ADC_UNIT_1, GPIO_NUM_35);

  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);  

  return(esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}


void setup() {
  Serial.begin(9600);
	adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(LM35_Sensor1, ADC_ATTEN);


	delay(100);
}
 
void loop() {
  // Read LM35_Sensor1 ADC Pin
  LM35_Raw_Sensor1 = adc1_get_raw(LM35_Sensor1);
  // Calibrate ADC & Get Voltage (in mV)
  Voltage = readADC_Cal(LM35_Raw_Sensor1);
  V = (float)LM35_Raw_Sensor1 / 4095 * 5;
  Serial.print("Calculated V = ");
  Serial.println(V);
  Rt = 10 * V / (5 - V);
  float tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950);
  Serial.print("Calculated Rt = ");
  Serial.println(Rt);
  Serial.print("Calculated Temp Kelvin = ");
  Serial.println(tempK);
  Serial.println("\n\n");


  // TempC = Voltage(mV) / 10
  Serial.print("Raw input = ");
  Serial.println(LM35_Raw_Sensor1);
  Serial.print("Voltage Ref = ");
  Serial.println(Voltage);

  


  // Serial.print("Buffer = ");
  // Serial.println( Voltage / MAX_VREF_VOLTAGE * INPUT_VOLTAGE);
  // Serial.print("TEMP C = ");
  // Serial.println(Voltage/10);
  // LM35_TempC_Sensor1 = Voltage / 10;
  // LM35_TempF_Sensor1 = (LM35_TempC_Sensor1 * 1.8) + 32;
 
  // Print The Read]]]ings
  // Serial.print("Temperature = ");
  // Serial.print(LM35_TempC_Sensor1);
  // Serial.print(" °C , ");
  // Serial.print("Temperature = ");
  // Serial.print(LM35_TempF_Sensor1);
  // Serial.println(" °F");
  delay(1000);
}
 ```








void taskSetWifi(void * parameters) {
  if (!client.connected()) {
    reconnect();
  } else {
    String str = String(count);
    str.toCharArray(msg,25);
    client.publish(root_topic_publish, msg);
    client.subscribe(root_topic_recieve);
    client.subscribe(root_topic_recieve);


    // To change or something idk
    // if ( Payload == "5") {
    //   digitalWrite(led_gpio,HIGH);
    //   Serial.println("true");
    //   delay(1000);
    // }
    // if ( Payload == "6") {
    //   digitalWrite(led_gpio,LOW);
    //   Serial.println("false");
    //   delay(1000);
    // }
    count++;
    vTaskDelay(300);
  }

  client.loop();
}


void reconnect() {
	while (!client.connected()) {

		Serial.print("Intentando conexión Mqtt...");
		// Creamos un cliente ID
		String clientId = "Alex";
		clientId += String(random(0xffff), HEX);
		// Intentamos conectar
		if (client.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
			Serial.println("Conectado!");
			// Nos suscribimos
			if(client.subscribe("acatutopicoraiz/movement")){
        Serial.println("Suscripcion ok");
      }else{
        Serial.println("fallo Suscripción");
      }
		} else {
			Serial.print("falló :( con error -> ");
			Serial.print(client.state());
			Serial.println(" Intentamos de nuevo en 5 segundos");

			vTaskDelay(5000);
		}
	}
}


void setup_wifi() {
	vTaskDelay(10);
	// Nos conectamos a nuestra red Wifi
	Serial.println();
	Serial.print("Conectando a ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		vTaskDelay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Conectado a red WiFi!");
	Serial.println("Dirección IP: ");
	Serial.println(WiFi.localIP());
}


void callback(char* topic, byte* payload, unsigned int length) {

	for (int i = 0; i < length; i++) {
		Payload += (char)payload[i];
	}
	Payload.trim();
}