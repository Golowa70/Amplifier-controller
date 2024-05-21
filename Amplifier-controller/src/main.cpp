#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>  
#include "GyverButton.h"
#include <GyverDS18.h>

#include "AVR_PWM.h"
#define pinToUse      44            // Timer5C on Mega
AVR_PWM* PWM_Instance;
float pwmFreq = 20000;
float pwmDuty = 20;

#include "defines.h"
#include "variables.h"
#include "init_functions.h"

void TaskThermostate(void* pvParameters);
void TaskButtonsPolling(void* pvParameters);
void TaskSpeakersRelays(void* pvParameters);
void TaskDirectRelay(void* pvParameters);
void TaskLoudnes(void* pvParameters);
void TaskPowerRelay(void* pvParameters);
void TaskMainDataHandler(void* pvParameters);
void TaskGetTemp(void* pvParameters);
void TaskGetRpm(void* pvParameters);
void TaskCheckProtections(void* pvParameters);
void TaskStatusLed(void* pvParameters);
void TaskMain(void* pvParameters);

QueueHandle_t voltageQueue;
QueueHandle_t sensorsVoltageQueue;
QueueHandle_t funPwmQueue;
QueueHandle_t powerRelayStateQueue;
QueueHandle_t sp_A_stateQueue;
QueueHandle_t sp_B_stateQueue;
QueueHandle_t directRelayStateQueue;
QueueHandle_t loudnesQueue;
QueueHandle_t statusLedQueue;
QueueHandle_t systemModeQueue;
QueueHandle_t mainDataQueue; //TODO

GButton btn_loudness(LOUDNESS_REMOTE_BUTTON_IN);
GButton btn_direct(DIRECT_BUTTON_IN);
GButton btn_sp_A(SPEAKERS_A_BUTTON_IN);
GButton btn_sp_B(SPEAKERS_B_BUTTON_IN);
GyverDS18Single ds1(ONE_WIRE_1_PIN);
GyverDS18Single ds2(ONE_WIRE_2_PIN);

bool fnCheckErrors();
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
    TaskThermostate, "InputsUpdate" // A name just for humans
    ,
    192 // This stack size can be checked & adjusted by reading the Stack Highwater //128
    ,
    NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL);

  xTaskCreate(TaskButtonsPolling, "Buttons polling task", 128, NULL, 2, NULL);
  xTaskCreate(TaskSpeakersRelays, "Speakers outputs task", 128, NULL, 1, NULL);
  xTaskCreate(TaskDirectRelay, "Direct relay task", 128, NULL, 1, NULL);
  xTaskCreate(TaskLoudnes, "Loudnes task", 128, NULL, 2, NULL);
  xTaskCreate(TaskPowerRelay, "Power relay task", 128, NULL, 1, NULL);
  xTaskCreate(TaskMainDataHandler, "Main data task", 512, NULL, 3, NULL);
  xTaskCreate(TaskGetTemp, "Get temp task", 128, NULL, 2, NULL);
  xTaskCreate(TaskGetRpm, "Get rpm task", 128, NULL, 2, NULL);
  xTaskCreate(TaskCheckProtections, "Check protections task", 128, NULL, 3, NULL);
  xTaskCreate(TaskStatusLed, "Status led task", 128, NULL, 1, NULL);
  xTaskCreate(TaskMain, "Main task", 128, NULL, 2, NULL);

  voltageQueue = xQueueCreate(1, sizeof(float));
  sensorsVoltageQueue = xQueueCreate(1, sizeof(float));
  funPwmQueue = xQueueCreate(1, sizeof(uint8_t));//?
  powerRelayStateQueue = xQueueCreate(1, sizeof(bool));
  sp_A_stateQueue = xQueueCreate(1, sizeof(bool));
  sp_B_stateQueue = xQueueCreate(1, sizeof(bool));
  directRelayStateQueue = xQueueCreate(1, sizeof(bool));
  loudnesQueue = xQueueCreate(1, sizeof(bool));
  statusLedQueue = xQueueCreate(6, sizeof(uint8_t));
  systemModeQueue = xQueueCreate(3, sizeof(uint8_t));
  mainDataQueue = xQueueCreate(10, sizeof(Param));

  attachInterrupt(3, fnRpm1, RISING);
  attachInterrupt(2, fnRpm2, RISING);

  PWM_Instance = new AVR_PWM(pinToUse, 20000, 0);

  vTaskStartScheduler();
  uint8_t mode = START_MODE;
  xQueueSend(systemModeQueue, &mode, 0);
}

void loop() {

}
//******************** tasks **********************************************

void TaskThermostate(void* pvParameters __attribute__((unused))) {
  bool th1 = false;
  bool  th2 = false;
  for (;;) {
    th1 = !digitalRead(THERMOSTAT_1);
    th2 = !digitalRead(THERMOSTAT_2);
    if (th1)errors[THERMOSTAT_1] = true;
    if (th2)errors[THERMOSTAT_2] = true;
    //TODO send to main data
  }
}

void TaskButtonsPolling(void* pvParameters __attribute__((unused))) {
  bool loudnes_btn = false;
  bool direct_btn = false;
  bool sp_A_btn = false;
  bool sp_B_btn = false;
  for (;;) {
    if (btn_loudness.isClick()) {
      Serial.println("Button loudness");
      loudnes_btn = !loudnes_btn;
      xQueueSend(loudnesQueue, &loudnes_btn, 0);
      //TODO
    }
    if (btn_direct.isClick()) {
      Serial.println("Button direct");
      direct_btn = !direct_btn;
      xQueueSend(directRelayStateQueue, &direct_btn, 0);
    }
    if (btn_sp_A.isClick()) {
      Serial.println("Button sp A");
      sp_A_btn = !sp_A_btn;
      xQueueSend(sp_A_stateQueue, &direct_btn, 0);
    }
    if (btn_sp_B.isClick()) {
      Serial.println("Button sp B");
      sp_B_btn = !sp_B_btn;
      xQueueSend(sp_B_stateQueue, &direct_btn, 0);
    }
  }
}

void TaskSpeakersRelays(void* pvParameters __attribute__((unused))) {
  bool sp_A_state = false;
  bool sp_B_state = false;
  for (;;) {
    if (xQueueReceive(sp_A_stateQueue, &sp_A_state, 0) == pdPASS) {
      digitalWrite(SPEAKERS_A_OUT, sp_A_state);
      if (sp_A_state)digitalWrite(SPEAKERS_B_OUT, false);
      //TODO send to main data
    }
    if (xQueueReceive(sp_B_stateQueue, &sp_B_state, 0) == pdPASS) {
      digitalWrite(SPEAKERS_B_OUT, sp_B_state);
      if (sp_B_state)digitalWrite(SPEAKERS_A_OUT, false);
      //TODO send to main data
    }
    vTaskDelay(1);
  }
}

void TaskDirectRelay(void* pvParameters __attribute__((unused))) {
  bool dr_state = false;
  for (;;) {
    if (xQueueReceive(directRelayStateQueue, &dr_state, 0) == pdPASS) {
      digitalWrite(DIRECT_INPUT_RELAY_OUT, dr_state);
      //TODO send to main data
    }
    vTaskDelay(1);
  }
}

void TaskLoudnes(void* pvParameters __attribute__((unused))) {
  bool loudnes = false;
  for (;;) {
    if (xQueueReceive(loudnesQueue, &loudnes, 0) == pdPASS) {
      //TODO
      //TODO send to main data
    }
    vTaskDelay(1);
  }
}

void TaskPowerRelay(void* pvParameters __attribute__((unused))) {
  bool pwr_state = false;
  for (;;) {
    if (xQueueReceive(powerRelayStateQueue, &pwr_state, 0) == pdPASS) {
      digitalWrite(POWER_RELAY_OUT, pwr_state);
      //TODO send to main data
    }
    vTaskDelay(1);
  }
}

void TaskMainDataHandler(void* pvParameters __attribute__((unused))) { //TODO
  Param param;

  for (;;) {
    if (xQueueReceive(powerRelayStateQueue, &param, 0) == pdPASS) {
      switch (param.key)
      {
      case VOLTAGE:
        /* code */
        break;

      case SENSORS_VOLTAGE:
        /* code */
        break;

      case SENSOR1_TEMP:
        main_data.sensor1_temp = param.value;
        break;

      case SENSOR2_TEMP:
        main_data.sensor2_temp = param.value;
        break;

      case FUN_PWM:
        /* code */
        break;

      case SYS_MODE:
        main_data.mode = (uint8_t)param.value;
        break;

      case FUN1_RPM:
        /* code */
        break;

      case FUN2_RPM:
        /* code */
        break;

      case OVER_TEMP1:
        /* code */
        break;

      case OVER_TEMP2:
        /* code */
        break;

      case POWER_RELAY:
        /* code */
        break;

      case SP_A:
        /* code */
        break;

      case SP_B:
        /* code */
        break;

      case DIRECT_RELAY:
        /* code */
        break;

      default:
        /* code */
        break;
      }
    }

  }
}

void TaskGetTemp(void* pvParameters __attribute__((unused))) {
  for (;;) {
    if (ds1.ready()) {
      Param temp1 = { SENSOR1_TEMP, FAULT_TEMP };
      if (ds1.readTemp()) {
        temp1.value = (int)ds1.getTemp();
        Serial.println((String)"temp1: " + temp1.value);
      }
      else {
        errors[ERR_TEMP1] = true;
        Serial.println("temp1 error");
      }
      if (temp1.value > MAX_TEMP)errors[ERR_TEMP1] = true;
      xQueueSend(mainDataQueue, &temp1, 0);
      ds1.requestTemp();
    }
    if (ds2.ready()) {
      Param temp2 = { SENSOR2_TEMP, FAULT_TEMP };
      if (ds2.readTemp()) {
        temp2.value = (int)ds2.getTemp();
        Serial.println((String)"temp2: " + temp2.value);
      }
      else {
        errors[ERR_TEMP2] = true;
        Serial.println("temp2 error");
      }
      if (temp2.value > MAX_TEMP)errors[ERR_TEMP2] = true;
      xQueueSend(mainDataQueue, &temp2, 0);
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
    if (rpm1 < MIN_RPM)errors[ERR_FUN1] = true;
    // TODO send to main data
    rpm2 = fnCalcRpm(&nbTopsFan2);
    if (rpm2 < MIN_RPM)errors[ERR_FUN2] = true;
    // TODO send to main data
    taskEXIT_CRITICAL();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskCheckProtections(void* pvParameters __attribute__((unused))) {
  uint8_t mode = ERROR_MODE;
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  for (;;) {
    if (fnCheckErrors())xQueueSend(systemModeQueue, &mode, 0);
  }
}

void TaskStatusLed(void* pvParameters __attribute__((unused))) {
  uint8_t mode = LED_OFF;
  for (;;) {
    if (xQueueReceive(statusLedQueue, &mode, 0) == pdPASS) {
      switch (mode) {
      case LED_SLOW:
        digitalWrite(POWER_BUTTON_LED, !digitalRead(POWER_BUTTON_LED));
        vTaskDelay(500 / portTICK_PERIOD_MS);
        break;
      case LED_ON:
        digitalWrite(POWER_BUTTON_LED, true);
        break;
      case LED_FAST:
        digitalWrite(POWER_BUTTON_LED, !digitalRead(POWER_BUTTON_LED));
        vTaskDelay(100 / portTICK_PERIOD_MS);
        break;
      case LED_OFF:
        digitalWrite(POWER_BUTTON_LED, false);
        break;
      default:
        digitalWrite(POWER_BUTTON_LED, false);
        break;
      }
    }
  }
}

void TaskMain(void* pvParameters __attribute__((unused))) {
  uint8_t mode = OFF_MODE;
  uint8_t led_mode = LED_OFF;
  bool power_relay = false;
  for (;;) {
    if (xQueueReceive(systemModeQueue, &mode, 0) == pdPASS) {
      switch (mode) {

      case START_MODE:
        PWM_Instance->setPWM(pinToUse, pwmFreq, pwmDuty);
        led_mode = LED_SLOW;
        xQueueSend(statusLedQueue, &led_mode, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        mode = RUN_MODE;
        xQueueSend(systemModeQueue, &mode, 0);
        break;

      case RUN_MODE:
        led_mode = LED_ON;
        xQueueSend(statusLedQueue, &led_mode, 0);
        power_relay = true;
        xQueueSend(powerRelayStateQueue, &power_relay, 0);
        break;

      case ERROR_MODE:
        led_mode = LED_FAST;
        xQueueSend(statusLedQueue, &led_mode, 0);
        power_relay = false;
        xQueueSend(powerRelayStateQueue, &power_relay, 0);
        break;

      case OFF_MODE:
        led_mode = LED_OFF;
        xQueueSend(statusLedQueue, &led_mode, 0);
        power_relay = false;
        xQueueSend(powerRelayStateQueue, &power_relay, 0);
        break;

      default:
        led_mode = LED_OFF;
        xQueueSend(statusLedQueue, &led_mode, 0);
        power_relay = false;
        xQueueSend(powerRelayStateQueue, &power_relay, 0);
        break;
      }
      Param sys_mode = { SYS_MODE, (int)mode };
      xQueueSend(mainDataQueue, &sys_mode, 0);
    }
  }
}

//***************************** functions *************************************************
bool fnCheckErrors() {
  for (uint8_t i = 0; i < ERR_QUANTITY; i++) {
    if (errors[i])return true;
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