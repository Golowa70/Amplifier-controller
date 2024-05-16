#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>  
#include "GyverButton.h"

#include "defines.h"
#include "variables.h"
#include "init_functions.h"


void TaskOutputsUpdate(void* pvParameters);
void TaskInputsUpdate(void* pvParameters);
void TaskButtonsPolling(void* pvParameters);
void TaskSpeakersRelays(void* pvParameters);
void TaskDirectRelay(void* pvParameters);
void TaskDirectRelay(void* pvParameters);
void TaskMainData(void* pvParameters);

SemaphoreHandle_t LoudnessButtonSemaphore;
SemaphoreHandle_t DirectButtonSemaphore;
SemaphoreHandle_t Sp_A_ButtonSemaphore;
SemaphoreHandle_t Sp_B_ButtonSemaphore;

QueueHandle_t voltageQueue;
QueueHandle_t sensorsVoltageQueue;
QueueHandle_t sensor1TempQueue;
QueueHandle_t sensor2TempQueue;
QueueHandle_t funPwmQueue;
QueueHandle_t fun1RpmQueue;
QueueHandle_t fun2RpmQueue;
QueueHandle_t thermostat1StateQueue;
QueueHandle_t thermostat2StateQueue;
QueueHandle_t powerRelayStateQueue;
QueueHandle_t sp_A_stateQueue;
QueueHandle_t sp_B_stateQueue;
QueueHandle_t directRelayStateQueue;



GButton btn_loudness(LOUDNESS_REMOTE_BUTTON_IN);
GButton btn_direct(DIRECT_BUTTON_IN);
GButton btn_sp_A(SPEAKERS_A_BUTTON_IN);
GButton btn_sp_B(SPEAKERS_B_BUTTON_IN);

void setup() {
  Serial.begin(9600);

  btn_loudness.setTickMode(AUTO);
  btn_direct.setTickMode(AUTO);
  btn_sp_A.setTickMode(AUTO);
  btn_sp_B.setTickMode(AUTO);

  xTaskCreate(
    TaskInputsUpdate, "InputsUpdate" // A name just for humans
    ,
    192 // This stack size can be checked & adjusted by reading the Stack Highwater //128
    ,
    NULL, 3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL);

  xTaskCreate(TaskOutputsUpdate, "Inputs update task", 128, NULL, 3, NULL);
  xTaskCreate(TaskButtonsPolling, "Buttons polling task", 128, NULL, 3, NULL);
  xTaskCreate(TaskSpeakersRelays, "Speakers outputs task", 128, NULL, 3, NULL);
  xTaskCreate(TaskDirectRelay, "Direct relay task", 128, NULL, 3, NULL);

  LoudnessButtonSemaphore = xSemaphoreCreateBinary();
  DirectButtonSemaphore = xSemaphoreCreateBinary();
  Sp_A_ButtonSemaphore = xSemaphoreCreateBinary();
  Sp_B_ButtonSemaphore = xSemaphoreCreateBinary();

  voltageQueue = xQueueCreate(1, sizeof(float));
  sensorsVoltageQueue = xQueueCreate(1, sizeof(float));
  sensor1TempQueue = xQueueCreate(1, sizeof(float));
  sensor2TempQueue = xQueueCreate(1, sizeof(float));
  funPwmQueue = xQueueCreate(1, sizeof(uint8_t));
  fun1RpmQueue = xQueueCreate(1, sizeof(uint16_t));
  fun2RpmQueue = xQueueCreate(1, sizeof(uint16_t));
  thermostat1StateQueue = xQueueCreate(1, sizeof(bool));
  thermostat2StateQueue = xQueueCreate(1, sizeof(bool));
  powerRelayStateQueue = xQueueCreate(1, sizeof(bool));
  sp_A_stateQueue = xQueueCreate(1, sizeof(bool));
  sp_B_stateQueue = xQueueCreate(1, sizeof(bool));
  directRelayStateQueue = xQueueCreate(1, sizeof(bool));
}

void loop() {

}
//******************************************************************

void TaskInputsUpdate(void* pvParameters __attribute__((unused))) {
  bool th1 = false;
  bool  th2 = false;
  for (;;) {
    th1 = digitalRead(THERMOSTAT_1);
    th2 = digitalRead(THERMOSTAT_2);
    xQueueSend(thermostat1StateQueue, &th1, 0);
    xQueueSend(thermostat2StateQueue, &th2, 0);
  }
}


void TaskOutputsUpdate(void* pvParameters __attribute__((unused))) {
  for (;;) {
    digitalWrite(POWER_RELAY_OUT, main_data.power_relay_state);
    digitalWrite(SPEAKERS_A_OUT, main_data.sp_A_out_state);
    digitalWrite(SPEAKERS_B_OUT, main_data.sp_B_out_state);
    digitalWrite(DIRECT_INPUT_RELAY_OUT, main_data.direct_relay_state);
  }
}
void TaskButtonsPolling(void* pvParameters __attribute__((unused))) {
  for (;;) {
    if (btn_loudness.isClick()) {
      Serial.println("Button loudness");
      xSemaphoreGive(LoudnessButtonSemaphore);
    }
    if (btn_direct.isClick()) {
      Serial.println("Button direct");
      xSemaphoreGive(DirectButtonSemaphore);
    }
    if (btn_sp_A.isClick()) {
      Serial.println("Button sp A");
      xSemaphoreGive(Sp_A_ButtonSemaphore);
    }
    if (btn_sp_B.isClick()) {
      Serial.println("Button sp B");
      xSemaphoreGive(Sp_B_ButtonSemaphore);
    }
  }
}

void TaskSpeakersRelays(void* pvParameters __attribute__((unused))) {
  for (;;) {
    if (xSemaphoreTake(Sp_A_ButtonSemaphore, portMAX_DELAY) == pdPASS) {
      digitalWrite(SPEAKERS_A_OUT, HIGH);
      digitalWrite(SPEAKERS_B_OUT, LOW);
    }
    if (xSemaphoreTake(Sp_B_ButtonSemaphore, portMAX_DELAY) == pdPASS) {
      digitalWrite(SPEAKERS_B_OUT, HIGH);
      digitalWrite(SPEAKERS_A_OUT, LOW);
    }
  }
}

void TaskDirectRelay(void* pvParameters __attribute__((unused))) {
  for (;;) {
    if (xSemaphoreTake(Sp_A_ButtonSemaphore, portMAX_DELAY) == pdPASS) {
      digitalWrite(DIRECT_INPUT_RELAY_OUT, !digitalRead(DIRECT_INPUT_RELAY_OUT));
    }
  }
}

void TaskMainData(void* pvParameters __attribute__((unused))) {
  float voltage;
  float sensors_voltage;
  float sensor1_temp;
  float sensor2_temp;
  uint8_t fun_pwm_value;
  uint16_t fun1_rpm;
  uint16_t fun2_rpm;
  bool thermostat1_state;
  bool thermostat2_state;
  bool power_relay_state;
  bool sp_A_out_state;
  bool sp_B_out_state;
  bool direct_relay_state;

  for (;;) {
    if (xQueueReceive(voltageQueue, &voltage, 0) == pdPASS) main_data.voltage = voltage;
    if (xQueueReceive(sensorsVoltageQueue, &sensors_voltage, 0) == pdPASS) main_data.sensors_voltage = sensors_voltage;


  }
}
