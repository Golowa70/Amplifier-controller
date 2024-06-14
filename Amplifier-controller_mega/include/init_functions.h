#ifndef INIT_FUNCTIONS_H
#define INIT_FUNCTIONS_H

#include <Arduino.h>
#include "defines.h"
#include "variables.h"


void fnIOInit(void) {

    //inputs declaration
    pinMode(REMOTE_CONTROL_IN, INPUT);
    pinMode(POWER_OFF_DETECT_IN, INPUT);
    pinMode(THERMOSTAT_1, INPUT_PULLUP);
    pinMode(THERMOSTAT_2, INPUT_PULLUP);
    pinMode(FUN_1_TACHO_SIGNAL, INPUT_PULLUP);//
    pinMode(FUN_2_TACHO_SIGNAL, INPUT_PULLUP);//
    pinMode(LOUDNESS_REMOTE_BUTTON_IN, INPUT_PULLUP);//
    pinMode(SPEAKERS_B_BUTTON_IN, INPUT_PULLUP);//
    pinMode(DIRECT_BUTTON_IN, INPUT_PULLUP);//
    pinMode(SPEAKERS_A_BUTTON_IN, INPUT_PULLUP);//

    pinMode(SPARE_IN_J3, INPUT_PULLUP);//
    pinMode(SPARE_IN_J2_3, INPUT_PULLUP);//
    pinMode(BUTTON_ON_BOARD, INPUT_PULLUP);//
    pinMode(POWER_OK_FROM_ADM705, INPUT_PULLUP);//
    pinMode(ENCODER_DT, INPUT_PULLUP);
    pinMode(ENCODER_SW, INPUT_PULLUP);
    pinMode(ENCODER_CLK, INPUT_PULLUP);

    analogReference(INTERNAL2V56);      // внутренний исочник опорного напряжения 2.56в
    //analogReference(EXTERNAL);          // внешний исочник опорного напряжения 2.5в(TL431)

//outputs declaration
    pinMode(POWER_RELAY_OUT, OUTPUT);
    pinMode(SPEAKERS_A_OUT, OUTPUT);
    pinMode(SPEAKERS_B_OUT, OUTPUT);
    pinMode(FUN_PWM_OUT, OUTPUT);
    pinMode(DIRECT_INPUT_RELAY_OUT, OUTPUT);
    pinMode(LOUDNES_LED, OUTPUT);
    pinMode(POWER_BUTTON_LED, OUTPUT);

    pinMode(BUZZER, OUTPUT);
    pinMode(SENSORS_SUPPLY_5V_OUT, OUTPUT);
    pinMode(MAIN_SUPPLY_OUT, OUTPUT);
    pinMode(STATUS_LED_OUT, OUTPUT);
    pinMode(WDT_RESET_OUT, OUTPUT);

    //outputs start state
    digitalWrite(POWER_RELAY_OUT, LOW);
    digitalWrite(SPEAKERS_A_OUT, LOW);
    digitalWrite(SPEAKERS_B_OUT, LOW);
    analogWrite(FUN_PWM_OUT, 0);
    digitalWrite(DIRECT_INPUT_RELAY_OUT, LOW);
    digitalWrite(LOUDNES_LED, LOW);
    digitalWrite(POWER_BUTTON_LED, LOW);

    digitalWrite(BUZZER, LOW);
    digitalWrite(SENSORS_SUPPLY_5V_OUT, HIGH);
    digitalWrite(MAIN_SUPPLY_OUT, HIGH);
    digitalWrite(STATUS_LED_OUT, LOW);
    digitalWrite(WDT_RESET_OUT, OUTPUT);
}


void fnDefaultSetpointsInit(void) {
    // default_setpoints_data.debug_key = DEBUG_KEY_0;
    // default_setpoints_data.key = EEPROM_KEY;
}

#endif