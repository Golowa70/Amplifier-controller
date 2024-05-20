#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>  
#include "GyverButton.h"
#include <GyverDS18.h>

#include "defines.h"
#include "variables.h"
#include "init_functions.h"


void TaskOutputsUpdate(void* pvParameters);
void TaskInputsUpdate(void* pvParameters);
void TaskButtonsPolling(void* pvParameters);
void TaskSpeakersRelays(void* pvParameters);
void TaskDirectRelay(void* pvParameters);
void TaskPowerRelay(void* pvParameters);
void TaskMainDataHandler(void* pvParameters);
void TaskGetTemp(void* pvParameters);
void TaskGetRpm(void* pvParameters);

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
QueueHandle_t errorsQueue;

GButton btn_loudness(LOUDNESS_REMOTE_BUTTON_IN);
GButton btn_direct(DIRECT_BUTTON_IN);
GButton btn_sp_A(SPEAKERS_A_BUTTON_IN);
GButton btn_sp_B(SPEAKERS_B_BUTTON_IN);
GyverDS18Single ds1(ONE_WIRE_1_PIN);
GyverDS18Single ds2(ONE_WIRE_2_PIN);

bool fnCheckProtections();
uint16_t fnCalcRpm(uint16_t* sourceRpm);
void fnRpm1();
void fnRpm2();

void setup() {
  Serial.begin(115200);
  ds1.requestTemp();
  ds2.requestTemp();

  btn_loudness.setTickMode(AUTO);
  btn_direct.setTickMode(AUTO);
  btn_sp_A.setTickMode(AUTO);
  btn_sp_B.setTickMode(AUTO);

  xTaskCreate(
    TaskInputsUpdate, "InputsUpdate" // A name just for humans
    ,
    192 // This stack size can be checked & adjusted by reading the Stack Highwater //128
    ,
    NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL);

  xTaskCreate(TaskOutputsUpdate, "Inputs update task", 128, NULL, 2, NULL);
  xTaskCreate(TaskButtonsPolling, "Buttons polling task", 128, NULL, 2, NULL);
  xTaskCreate(TaskSpeakersRelays, "Speakers outputs task", 128, NULL, 1, NULL);
  xTaskCreate(TaskDirectRelay, "Direct relay task", 128, NULL, 1, NULL);
  xTaskCreate(TaskPowerRelay, "Power relay task", 128, NULL, 1, NULL);
  xTaskCreate(TaskMainDataHandler, "Main data task", 128, NULL, 3, NULL);
  xTaskCreate(TaskGetTemp, "Get temp task", 128, NULL, 2, NULL);
  xTaskCreate(TaskGetRpm, "Get rpm task", 128, NULL, 2, NULL);

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
  errorsQueue = xQueueCreate(6, sizeof(bool));

  attachInterrupt(3, fnRpm1, RISING);
  attachInterrupt(2, fnRpm2, RISING);
}

void loop() {

}
//******************** tasks **********************************************

void TaskInputsUpdate(void* pvParameters __attribute__((unused))) {
  bool th1 = false;
  bool  th2 = false;
  for (;;) {
    th1 = !digitalRead(THERMOSTAT_1);
    th2 = !digitalRead(THERMOSTAT_2);
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
  bool sp_A_state = false;
  bool sp_B_state = false;
  for (;;) {
    if (xSemaphoreTake(Sp_A_ButtonSemaphore, portMAX_DELAY) == pdPASS) {
      sp_A_state = true;
      sp_B_state = false;
      xQueueSend(sp_A_stateQueue, &sp_A_state, 0);
      xQueueSend(sp_B_stateQueue, &sp_B_state, 0);
    }
    if (xSemaphoreTake(Sp_B_ButtonSemaphore, portMAX_DELAY) == pdPASS) {
      sp_A_state = false;
      sp_B_state = true;
      xQueueSend(sp_A_stateQueue, &sp_A_state, 0);
      xQueueSend(sp_B_stateQueue, &sp_B_state, 0);
    }
  }
}

void TaskDirectRelay(void* pvParameters __attribute__((unused))) {
  bool dr_state = false;
  for (;;) {
    if (xSemaphoreTake(Sp_A_ButtonSemaphore, portMAX_DELAY) == pdPASS) {
      dr_state = !main_data.direct_relay_state;
      xQueueSend(directRelayStateQueue, &dr_state, 0);
    }
  }
}

void TaskPowerRelay(void* pvParameters __attribute__((unused))) {
  bool pwr_state = false;
  for (;;) {
    if (!fnCheckProtections) {
      pwr_state = true;
    }
    else {
      pwr_state = false;
    }
    xQueueSend(powerRelayStateQueue, &pwr_state, 0);
    vTaskDelay(1);
  }
}

void TaskMainDataHandler(void* pvParameters __attribute__((unused))) {
  Data data;
  for (;;) {
    if (xQueueReceive(voltageQueue, &data.voltage, 0) == pdPASS) main_data.voltage = data.voltage;
    if (xQueueReceive(sensorsVoltageQueue, &data.sensors_voltage, 0) == pdPASS) main_data.sensors_voltage = data.sensors_voltage;
    if (xQueueReceive(sensor1TempQueue, &data.sensor1_temp, 0) == pdPASS) main_data.sensor1_temp = data.sensor1_temp;
    if (xQueueReceive(sensor2TempQueue, &data.sensor2_temp, 0) == pdPASS) main_data.sensor2_temp = data.sensor2_temp;
    if (xQueueReceive(fun1RpmQueue, &data.fun1_rpm, 0) == pdPASS) main_data.fun1_rpm = data.fun1_rpm;
    if (xQueueReceive(fun2RpmQueue, &data.fun2_rpm, 0) == pdPASS) main_data.fun2_rpm = data.fun2_rpm;
    if (xQueueReceive(thermostat1StateQueue, &data.over_temp_1, 0) == pdPASS) main_data.over_temp_1 = data.over_temp_1;
    if (xQueueReceive(thermostat2StateQueue, &data.over_temp_2, 0) == pdPASS) main_data.over_temp_2 = data.over_temp_2;
    if (xQueueReceive(powerRelayStateQueue, &data.power_relay_state, 0) == pdPASS) main_data.power_relay_state = data.power_relay_state;
    if (xQueueReceive(sp_A_stateQueue, &data.sp_A_out_state, 0) == pdPASS) main_data.sp_A_out_state = data.sp_A_out_state;
    if (xQueueReceive(sp_B_stateQueue, &data.sp_B_out_state, 0) == pdPASS) main_data.sp_B_out_state = data.sp_B_out_state;
    if (xQueueReceive(directRelayStateQueue, &data.direct_relay_state, 0) == pdPASS) main_data.direct_relay_state = data.direct_relay_state;
    vTaskDelay(1);
  }
}

void TaskGetTemp(void* pvParameters __attribute__((unused))) {
  uint8_t err_index = 0;
  for (;;) {
    if (ds1.ready()) {
      float temp1 = FAULT_TEMP;
      if (ds1.readTemp()) {
        temp1 = ds1.getTemp();
        Serial.println((String)"temp1: " + temp1);
      }
      else {
        err_index = ERR_TEMP1;
        xQueueSend(errorsQueue, &err_index, 0);
        Serial.println("temp1 error");
      }
      xQueueSend(sensor1TempQueue, &temp1, 0);
      ds1.requestTemp();
    }
    if (ds2.ready()) {
      float temp2 = FAULT_TEMP;
      if (ds2.readTemp()) {
        temp2 = ds2.getTemp();
        Serial.println((String)"temp2: " + temp2);
      }
      else {
        err_index = ERR_TEMP1;
        xQueueSend(errorsQueue, &err_index, 0);
        Serial.println("temp2 error");
      }
      xQueueSend(sensor2TempQueue, &temp2, 0);
      ds2.requestTemp();
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void TaskGetRpm(void* pvParameters __attribute__((unused))) {
  for (;;) {
    taskENTER_CRITICAL();
    uint16_t rpm1, rpm2 = 0;
    rpm1 = fnCalcRpm(&nbTopsFan1);
    rpm2 = fnCalcRpm(&nbTopsFan2);
    taskEXIT_CRITICAL();
    xQueueSend(fun1RpmQueue, &rpm1, 0);
    xQueueSend(fun2RpmQueue, &rpm2, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

//***************************** functions *************************************************
bool fnCheckProtections() {
  if (main_data.fun1_rpm < MIN_RPM || main_data.fun2_rpm < MIN_RPM
    && main_data.sensor1_temp > MAX_TEMP && main_data.sensor2_temp > MAX_TEMP) {
    return true;
  }
  for (uint8_t i = 0; i < ERR_QUANTITY; i++) {
    if (errors[i])return;
  }
  return false;
}

uint16_t fnCalcRpm(uint16_t* sourceRpm) {
  if (*sourceRpm == 0) {
    return 0;
  }
  uint16_t res = (*sourceRpm * 60) / 2;
  *sourceRpm = 0;
  return res;
}

void fnRpm1() {
  nbTopsFan1++;
}

void fnRpm2() {
  nbTopsFan2++;
}