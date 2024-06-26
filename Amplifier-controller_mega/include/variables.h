#ifndef VARIABLES_H
#define VARIABLES_H

#include <inttypes.h>


//*********** Setpoints variables *********************************************************************

struct InitValuesStruct { // init setpoints
  uint8_t key;
  uint8_t crc8;  // not implemented yet

}default_setpoints_data;

// union {
//   InitValuesStruct setpoints_data;
//   uint8_t SetpointsArray[2];
// }SetpointsUnion; // 2 bytes

//*********** Main data *******************************************************************************
struct Data
{
  uint16_t voltage;                   //power supply voltage 
  uint16_t sensors_voltage;
  int sensor1_temp;
  int sensor2_temp;
  uint8_t fun_pwm_value;
  uint8_t mode;
  uint16_t fun1_rpm;
  uint16_t fun2_rpm;
  bool over_temp_1;
  bool over_temp_2;
  bool power_relay_state;
  bool sp_A_out_state;
  bool sp_B_out_state;
  bool direct_relay_state;
} main_data;

//param struct for send to main_data
struct Param {
  uint8_t key;
  int value;
};

//pwm 
float pwmDuty = 20;

//errors
bool errors[ERR_QUANTITY] = { 0, };

//fun rpm
uint16_t nbTopsFan1;
uint16_t nbTopsFan2;

#endif
