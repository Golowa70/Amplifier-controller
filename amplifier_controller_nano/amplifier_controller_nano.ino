#include <GyverPWM.h>
#include <GyverDS18.h>
#include <Adafruit_MCP23X17.h>

#define ONE_WIRE_PIN 5
#define PWM_PIN 9
#define PWM_FREQ 25000
#define PWM_OFF_DUTY 255
#define PWM_MIN_DUTY 180
#define PWM_NORMAL_DUTY 120
#define PWM_MAX_DUTY 20
#define ERROR 200
#define TEMP_SETPOINT 55
#define TEMP_HYSTERESYS 5
#define THERMOSTAT_IN 17
#define START_DELAY 5000
#define ON true
#define OFF false
#define LED_ON false
#define LED_OFF true
#define SP_A_LED_PIN 0  // MCP23XXX pin LED
#define SP_B_LED_PIN 1
#define DIRECT_LED_PIN 2
#define LOUDNESS_LED_PIN 3
#define POWER_LED_PIN 4
#define SP_A_BUTTON_PIN 8  // MCP23XXX pin button
#define SP_B_BUTTON_PIN 9
#define DIRECT_BUTTON_PIN 10
#define LOUDNESS_BUTTON_PIN 11
#define AMP_POWER_RELAY_PIN 6
#define SP_A_RELAY_PIN 10
#define SP_B_RELAY_PIN 11

GyverDS18 ds(ONE_WIRE_PIN);  // pin
uint64_t leftSensor = 0x7580000008684828;
uint64_t rightSensor = 0xF580000027DF8D28;

float leftTemp;
float rightTemp;
bool isTempHi = false;
bool isThermostatOn = false;
bool speakerA = false;
bool speakerB = false;

Adafruit_MCP23X17 mcp;

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
  pinMode(AMP_POWER_RELAY_PIN, OUTPUT);
  pinMode(SP_A_RELAY_PIN, OUTPUT);
  pinMode(SP_B_RELAY_PIN, OUTPUT);
  pinMode(THERMOSTAT_IN, INPUT);

  mcp.begin_I2C();
  mcp.pinMode(SP_A_LED_PIN, OUTPUT);
  mcp.pinMode(SP_B_LED_PIN, OUTPUT);
  mcp.pinMode(DIRECT_LED_PIN, OUTPUT);
  mcp.pinMode(LOUDNESS_LED_PIN, OUTPUT);
  mcp.pinMode(POWER_LED_PIN, OUTPUT);
  mcp.pinMode(SP_A_BUTTON_PIN, INPUT_PULLUP);
  mcp.pinMode(SP_B_BUTTON_PIN, INPUT_PULLUP);
  mcp.pinMode(DIRECT_BUTTON_PIN, INPUT_PULLUP);
  mcp.pinMode(LOUDNESS_BUTTON_PIN, INPUT_PULLUP);

  mcp.digitalWrite(SP_A_LED_PIN, LED_OFF);
  mcp.digitalWrite(SP_B_LED_PIN, LED_OFF);

  PWM_frequency(PWM_PIN, PWM_FREQ, FAST_PWM);
  PWM_set(PWM_PIN, PWM_MAX_DUTY);  //inverted
  delay(START_DELAY);
  PWM_set(PWM_PIN, PWM_MIN_DUTY);  //inverted
  ds.requestTemp();

  mcp.digitalWrite(POWER_LED_PIN, LED_ON);
  digitalWrite(AMP_POWER_RELAY_PIN, ON);
  digitalWrite(SP_A_RELAY_PIN, ON);
  mcp.digitalWrite(SP_A_LED_PIN, LED_ON);
}

void loop() {

  //********************** temp + fun control*******************************
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

  if (leftTemp > TEMP_SETPOINT || rightTemp > TEMP_SETPOINT) {
    isTempHi = true;
  }
  if (rightTemp < (TEMP_SETPOINT - TEMP_HYSTERESYS) && leftTemp < (TEMP_SETPOINT - TEMP_HYSTERESYS)) {
    isTempHi = false;
  }

  isThermostatOn = digitalRead(THERMOSTAT_IN);

  if (isThermostatOn) {
    PWM_set(PWM_PIN, PWM_MAX_DUTY);
  } else {
    isTempHi ? PWM_set(PWM_PIN, PWM_NORMAL_DUTY) : PWM_set(PWM_PIN, PWM_MIN_DUTY);
  }
  // digitalWrite(FUN_POWER_OUT, isTempHi || isThermostatOn ? ON : OFF);

  //************************** buttons + leds + relay *************************************

  if (!mcp.digitalRead(SP_A_BUTTON_PIN)) {
    mcp.digitalWrite(SP_A_LED_PIN, LED_ON);
    digitalWrite(SP_A_RELAY_PIN, ON);
    mcp.digitalWrite(SP_B_LED_PIN, LED_OFF);
    digitalWrite(SP_B_RELAY_PIN, OFF);
  }

  if (!mcp.digitalRead(SP_B_BUTTON_PIN)) {
    mcp.digitalWrite(SP_B_LED_PIN, LED_ON);
    digitalWrite(SP_B_RELAY_PIN, ON);
    mcp.digitalWrite(SP_A_LED_PIN, LED_OFF);
    digitalWrite(SP_A_RELAY_PIN, OFF);
  }
}