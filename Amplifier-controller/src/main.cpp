#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

#include "defines.h"
#include "variables.h"
#include "init_functions.h"


void TaskOutputsUpdate(void* pvParameters);
void TaskInputsUpdate(void* pvParameters);


void setup() {

  xTaskCreate(
    TaskInputsUpdate, "InputsUpdate" // A name just for humans
    ,
    192 // This stack size can be checked & adjusted by reading the Stack Highwater //128
    ,
    NULL, 3 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL);

  xTaskCreate(TaskOutputsUpdate, "InputsUpdate", 128, NULL, 3, NULL);


}

void loop() {

}
//******************************************************************

void TaskInputsUpdate(void* pvParameters __attribute__((unused))) {
  for (;;) {
    main_data.thermostat1_state = digitalRead(THERMOSTAT_1);
    main_data.thermostat2_state = digitalRead(THERMOSTAT_2);
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

