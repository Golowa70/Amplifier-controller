#include <Arduino.h>

#include "defines.h"
#include "variables.h"
#include "init_functions.h"

void fnInputsUpdate(void);
void fnOutputsUpdate(Data& data);

void setup() {

}

void loop() {

}
//******************************************************************

void fnInputsUpdate(void) {
  main_data.thermostat1_state = digitalRead(THERMOSTAT_1);
  main_data.thermostat2_state = digitalRead(THERMOSTAT_2);
}

void fnOutputsUpdate(Data& data) {
  digitalWrite(POWER_RELAY_OUT, main_data.power_relay_state);
  digitalWrite(SPEAKERS_A_OUT, main_data.sp_A_out_state);
  digitalWrite(SPEAKERS_B_OUT, main_data.sp_B_out_state);
  digitalWrite(DIRECT_INPUT_RELAY_OUT, main_data.direct_relay_state);
}

