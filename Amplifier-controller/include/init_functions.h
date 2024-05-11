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
    pinMode(WATER_PUMP_OUTPUT_1, OUTPUT);
    pinMode(FRIDGE_OUTPUT_2, OUTPUT);
    pinMode(CONVERTER_OUTPUT_3, OUTPUT);
    pinMode(SENSORS_SUPPLY_5v, OUTPUT);
    pinMode(MAIN_SUPPLY_OUT, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(BUILTIN_LED, OUTPUT);
    pinMode(WDT_RESET_OUT, OUTPUT);
    pinMode(DE_RS485_PIN, OUTPUT);
    pinMode(LCD_BLA_OUT, OUTPUT);

    //outputs start state
    digitalWrite(WATER_PUMP_OUTPUT_1, LOW);
    digitalWrite(FRIDGE_OUTPUT_2, LOW);
    digitalWrite(CONVERTER_OUTPUT_3, LOW);
    digitalWrite(SENSORS_SUPPLY_5v, LOW);
    digitalWrite(MAIN_SUPPLY_OUT, HIGH);
    digitalWrite(BUZZER, LOW);
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(WDT_RESET_OUT, LOW);
    analogWrite(LCD_BLA_OUT, 127);
}


void fnDefaultSetpointsInit(void) {
    default_setpoints_data.pump_off_delay = 10;
    default_setpoints_data.pump_out_mode = AUTO_MODE;  // Режим(авто, вкл, выкл).
    default_setpoints_data.converter_U_off = 115;    // дробное со смещённой вправо точкой 12.7в = 127,  13.2в =132 и т.д.
    default_setpoints_data.converter_T_U_off = 10;  // задержка отключения по низкому напряжению U_off
    default_setpoints_data.converter_U_on = 127;     // напряжение включения (порог)

    default_setpoints_data.converter_T_U_on = 10;   // задержка включения по U_on
    default_setpoints_data.converter_T_IGN_off = 10; // задержка отключения после выключения зажигания
    default_setpoints_data.convertet_out_mode = AUTO_MODE;  // Режим(авто, вкл, выкл). 
    default_setpoints_data.fridge_U_off = 115;     // дробное со смещённой вправо точкой 12.7в = 127,  13.2в =132 и т.д.
    default_setpoints_data.fridge_T_U_off = 10;   // задержка отключения по низкому напряжению U_off

    default_setpoints_data.fridge_U_on = 127;      // напряжение включения (порог)
    default_setpoints_data.fridge_T_U_on = 10;    // задержка включения по U_on
    default_setpoints_data.fridge_T_IGN_off = 10; // задержка отключения после выключения зажигания
    default_setpoints_data.fridge_Temp_on = 10;   // Температура включения Temp_on.
    default_setpoints_data.fridge_Temp_off = 7;  // Температура выключения Temp_off.

    default_setpoints_data.fridge_out_mode = AUTO_MODE;  // Режим(авто, авто с термостатом, вкл, выкл).
    default_setpoints_data.resistive_sensor_correction = 127; // 127 - коррекция по умолчанию
    default_setpoints_data.resistive_sensor_max = 190;    // 
    default_setpoints_data.resistive_sensor_min = 0;
    default_setpoints_data.water_sensor_type_selected = PJON_SENSOR_TYPE;
    default_setpoints_data.water_tank_capacity = WATER_TANK_CAPACITY;

    default_setpoints_data.mb_slave_ID = 2;
    default_setpoints_data.mb_baud_rate = MB_RATE_9600;
    default_setpoints_data.buzzer_out_mode = 0; // off
    default_setpoints_data.scrreen_off_delay = 5;  //
    default_setpoints_data.shutdown_delay = 2;

    default_setpoints_data.voltage_correction = 104; // 104 - коррекция по умолчанию
    default_setpoints_data.lcd_brightness = 70;
    default_setpoints_data.logo_selection = 0;
    default_setpoints_data.inside_sensor_id = 1;
    default_setpoints_data.outside_sensor_id = 2;

    default_setpoints_data.fridge_sensor_id = 3;
    default_setpoints_data.debug_key = DEBUG_KEY_0;
    default_setpoints_data.screensaver_timeout = 5;
    default_setpoints_data.spare_2 = 0;

    default_setpoints_data.key = EEPROM_KEY;

}

#endif