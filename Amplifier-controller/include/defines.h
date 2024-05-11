#ifndef DEFINES_H
#define DEFINES_H

#include <Arduino.h>

//1 wire pins
#define ONE_WIRE_1_PIN       3     
#define ONE_WIRE_2_PIN       4

//inputs
#define REMOTE_CONTROL_IN                   54       // remote control input          J2-1
#define POWER_OFF_DETECT_IN                 56       // AC off detection input        J2-2
#define THERMOSTAT_1                         5       // KSD-01F thermostat 1 input    J2-4
#define THERMOSTAT_2                        42       // KSD-01F thermostat 2 input    J2-5
#define FUN_1_TACHO_SIGNAL                  20      // FUN 1 TACHO SIGNAL   J12-2
#define FUN_2_TACHO_SIGNAL                  21      // FUN 2 TACHO SIGNAL   J12-3
#define LOUDNESS_REMOTE_BUTTON_IN           43      // remot control on/off button input  J9-7
#define  SPEAKERS_B_BUTTON_IN                51      // button to on/off speakers B output   J9-5
#define  DIRECT_BUTTON_IN                    52      // direct button  J9-6
#define  SPEAKERS_A_BUTTON_IN                53      // button to on/off speakers A output   J9-4

#define SPARE_IN_J3                         55       //  spare  input      J3
#define SPARE_IN_J2_3                       57       // spare input    J2-3
#define SUPPLY_VOLTAGE_INPUT                A4       // Power supply voltage measurement input
#define SENSORS_VOLTAGE_INPUT               A5       // Sensors supplier voltage measurement input(5V)
#define RESISTIVE_SENSOR                    A6       // resestive sensor input (not used)   J6
#define BUTTON_ON_BOARD                     10       // on board button input (not used)
#define POWER_OK_FROM_ADM705                41       // power ok signal input from supervisor AD705 (not used)
#define ENCODER_DT                          36       //  ENCODER DT  input (not used)       J10-4
#define ENCODER_SW                          35       //  ENCODER SW  input (not used)       J10-3
#define ENCODER_CLK                         37       //  ENCODER_CLK  input (not used)      J10-5


//outputs
#define POWER_RELAY_OUT                     22  //  J13
#define SPEAKERS_A_OUT                      23  //  J14
#define SPEAKERS_B_OUT                      24  //  J15
#define FUN_PWM_OUT                         44 //   pwm out    J9-8
#define DIRECT_INPUT_RELAY_OUT              31  //  J8-1
#define LOUDNES_LED                         32  //  J8-2
#define POWER_BUTTON_LED                    33  //  J8-3

#define BUZZER                              2  // 
#define SENSORS_SUPPLY_5V_OUT                7  //  
#define MAIN_SUPPLY_OUT			             9  // 
#define BUILTIN_LED                         13  // 
#define WDT_RESET_OUT                        8  // WDT reset signal to AD705



//other
#define DIVISION_RATIO_VOLTAGE_INPUT      0.0104     // 0.0208 разрешение 0.0025 уможить на коэфициент деления предусилителя 4.85(или делителя)
#define DIVISION_RATIO_SENS_SUPPLY_INPUT  0.0132     // разрешение 0.0025(для TL431) или 0.00256 (для внутреннего опорного 2.56в) уможить на коэфициент деления предусилителя 4.16
#define DIVISION_RATIO_RESIST_SENSOR      0.140      //0.278     

#define MS_100        100           //  100ms
#define SECOND       1000           //ms  секунда
#define MINUTE       60000          //ms  минута
#define HOUR         3600000        //ms  час

//timers
#define START_DELAY                     1000  //ms задержка выполнения некоторых функций после подачи питания (пока датчики запустятся)
#define WDT_RESET_PERIOD                1000  // us период сброса ADM705 < 1.6 sec
#define TEMP_SENSORS_UPDATE_PERIOD      1000   //ms


#endif