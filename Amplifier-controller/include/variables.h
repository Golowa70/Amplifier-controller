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
  float voltage;                   //power supply voltage 
  float sensors_voltage;
  float sensor1_temp;
  float sensor2_temp;
  uint8_t fun_pwm_value;
  uint16_t fun1_rpm;
  uint16_t fun2_rpm;
  bool over_temp_1;
  bool over_temp_2;
  bool power_relay_state;
  bool sp_A_out_state;
  bool sp_B_out_state;
  bool direct_relay_state;
} main_data;

bool errors[ERR_QUANTITY] = { 0, };

#endif
