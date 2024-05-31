#include <GyverPWM.h>
#include <GyverDS18.h>

#define PWM_PIN 9
#define PWM_FREQ  25000
#define PWM_MIN_DUTY  240
#define PWM_NORMAL_DUTY  150
#define PWM_MAX_DUTY  20
#define ERROR         200
#define TEMP_SETPOINT   55
#define TEMP_HYSTERESYS  5

GyverDS18 ds(5);  // пин
uint64_t leftSensor = 0x7580000008684828;
uint64_t rightSensor = 0xF580000027DF8D28;

float leftTemp;
float rightTemp;


void setup() {
  Serial.begin(115200);
  //scanner
  // uint64_t addr = ds.readAddress();
  //   if (addr) {
  //       Serial.print("address: ");
  //       gds::printAddress(addr, Serial);
  //   } else {
  //       Serial.println("error");
  //   }

  pinMode(PWM_PIN, OUTPUT);
  PWM_frequency(PWM_PIN, PWM_FREQ, FAST_PWM);
  PWM_set(PWM_PIN, PWM_MAX_DUTY); //inverted
  delay(2000);
  PWM_set(PWM_PIN, PWM_MIN_DUTY); //inverted
  ds.requestTemp();
}

  void loop() {
    if (ds.ready()) {  // измерения готовы по таймеру
      // читаем КОНКРЕТНЫЙ датчик по адресу
      if (ds.readTemp(rightSensor)) {  // если чтение успешно
        rightTemp = ds.getTemp();
        Serial.print("right sensor temp: ");
        Serial.println(rightTemp);
      } else {
        rightTemp = ERROR;
        Serial.println("right sensor error");
      }
       if (ds.readTemp(leftSensor)) {  // если чтение успешно
       leftTemp = ds.getTemp();
        Serial.print("left sensor temp: ");
        Serial.println(leftTemp);
      } else {
        leftTemp = ERROR;
        Serial.println("left sensor error");
      }

      ds.requestTemp();  // запрос следующего измерения ДЛЯ ВСЕХ
    }
    
    if(rightTemp > TEMP_SETPOINT || leftSensor > TEMP_SETPOINT){
      PWM_set(PWM_PIN, PWM_NORMAL_DUTY);
    }

    if(rightTemp < (TEMP_SETPOINT-TEMP_HYSTERESYS) && leftSensor < (TEMP_SETPOINT-TEMP_HYSTERESYS)){
      PWM_set(PWM_PIN, PWM_MIN_DUTY);
    }
    

  }