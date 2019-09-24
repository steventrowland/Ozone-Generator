#include <Wire.h>
#include "Adafruit_BME680.h"
#include "Adafruit_FRAM_I2C.h"
#include "KalmanFilter.h"
#include "LiquidCrystal_I2C.h"
#include "MQ131.h"

int POTIN = 0;
int OZOIN = 1;
int BUTIN = 9;
int SOLOUT = 4;
int LEDOUT = 5;
int RELOUT = 10;
int BUZOUT = 12;

#define SEALEVELPRESSURE_HPA (1013.25)

LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BME680 bme;
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C();

KalmanFilter tempKF = KalmanFilter(0.1, 10);
KalmanFilter humiKF = KalmanFilter(0.1, 10);
KalmanFilter presKF = KalmanFilter(0.1, 10);
KalmanFilter gasBKF = KalmanFilter(0.1, 10);
KalmanFilter gasMKF = KalmanFilter(0.1, 10);

long veriTime = 3600000;//1 hour verify sensors
long presTime = 60000;//max on time for 1 minute * pot ratio
long reseTime = 21600000;//reset every 6 hours

long pressurePrevious;

void(* ResetArduino) (void) = 0;//ResetArduino();

void SetLCDDisplay(String line1, String line2) {
  if (line1.length() <= 16 && line2.length() <= 16) {
    for (int i = line1.length(); i < 15; i++) {
      line1 += " ";
    }

    for (int i = line2.length(); i < 15; i++) {
      line2 += " ";
    }

    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LCD Char Limit");
  }
}

void startup() {
  threeBeep(100, 100, 100, 200, 200, 200);
}

void success() {
  threeBeep(100, 100, 100, 50, 50, 50);
}

void pass() {
  threeBeep(50, 50, 50, 20, 20, 20);
}

void fail() {
  threeBeep(300, 300, 300, 300, 300, 300);
}

void setup() {
  lcd.begin();

  SetLCDDisplay("Initializing...", "");

  pinMode(POTIN, INPUT);
  pinMode(OZOIN, INPUT);
  pinMode(BUTIN, INPUT_PULLUP);

  pinMode(SOLOUT, OUTPUT);
  pinMode(LEDOUT, OUTPUT);
  pinMode(RELOUT, OUTPUT);
  pinMode(BUZOUT, OUTPUT);

  digitalWrite(BUZOUT, LOW);
  digitalWrite(LEDOUT, LOW);
  digitalWrite(SOLOUT, LOW);
  digitalWrite(RELOUT, LOW);

  startup();
  
  delay(1000);

  verifySensors("Initializing...");

  SetLCDDisplay("Initializing...", "Resets: " + String(checkRestarts()));

  delay(1000);

  if (testOzoneProduction()) {
    SetLCDDisplay("Initializing", "O3 Done...");
    pass();
    delay(1000);
  }
  else {
    SetLCDDisplay("O3 Failed...", "Restarting...");
    fail();
    delay(1000);
    ResetArduino();
  }

  success();

  delay(1000);
}

void loop() {
  SetLCDDisplay("Sampling...", "MQ131...");
  MQ131.sample();
  SetLCDDisplay("Sampling...", "BME680...");
  if (!bme.performReading()) {
    ResetArduino();
    return;
  }
  SetLCDDisplay("Running...", "");

  long currentTime = millis();
  float ozon = MQ131.getO3(PPM);
  float temp = bme.temperature;
  float pres = bme.pressure / 0.0f;
  float humi = bme.humidity;
  float gasr = bme.gas_resistance / 1000.0f;

  if (digitalRead(BUTIN) == LOW) {
    digitalWrite(BUZOUT, HIGH);
    digitalWrite(LEDOUT, HIGH);
    digitalWrite(SOLOUT, HIGH);
    digitalWrite(RELOUT, HIGH);
  }
  else {
    digitalWrite(BUZOUT, LOW);
    digitalWrite(LEDOUT, LOW);
    digitalWrite(SOLOUT, LOW);
    digitalWrite(RELOUT, LOW);
  }

  SetLCDDisplay(String(ozon) + " " +
                String(temp) + " " +
                String(pres),
                String(humi) + " " +
                String(gasr) + " " +
                String(digitalRead(BUTIN))
               );

  delay(1000);
}

bool testOzoneProduction() {
  return true;
}

void initializeBME() {
  String topLine = "BME Setup...";
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  SetLCDDisplay(topLine, "Set sampling...");

  pass();
  delay(500);
  
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  SetLCDDisplay(topLine, "Set IIR...");
  
  pass();
  delay(500);
  
  SetLCDDisplay(topLine, "Heating...");
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  pass();
  delay(1000);
}

uint32_t checkRestarts() {
  uint32_t value = read32(0x0);

  value += 1;

  write32(0x0, value);

  return value;
}

void write32(uint32_t addr, uint32_t value) {
  uint8_t MMSB, MLSB, LMSB, LLSB;

  MMSB = 0xFF & (value >> 24);
  MLSB = 0xFF & (value >> 16);
  LMSB = 0xFF & (value >> 8);
  LLSB = 0xFF & (value);

  fram.write8(addr + 0x0, MMSB);
  fram.write8(addr + 0x1, MLSB);
  fram.write8(addr + 0x2, LMSB);
  fram.write8(addr + 0x3, LLSB);

  //SetLCDDisplay("FRAM: " + String(MMSB) + " " + String(MLSB), String(LMSB) + " " + String(LLSB));
  //delay(1500);
}

uint32_t read32(uint32_t addr) {
  uint32_t value = 0;
  uint32_t MMSB, MLSB, LMSB, LLSB;

  MMSB = fram.read8(addr + 0x0);
  MLSB = fram.read8(addr + 0x1);
  LMSB = fram.read8(addr + 0x2);
  LLSB = fram.read8(addr + 0x3);

  value = value | (MMSB << 24);
  value = value | (MLSB << 16);
  value = value | (LMSB << 8);
  value = value | (LLSB);
  
  //SetLCDDisplay("Read FRAM...", String(value));
  //delay(1500);

  return value;
}

void verifySensors(String firstLine) {
  MQ131.begin(OZOIN, LOW_CONCENTRATION, 10000);
  SetLCDDisplay(firstLine, "MQ131 Done..."); pass(); delay(1000);

  if (fram.begin()) {
    SetLCDDisplay(firstLine, "FRAM Done...");
    pass();
    delay(1000);
  }
  else {
    SetLCDDisplay("FRAM failed...", "Restarting...");
    fail();
    scanI2C();
    delay(1000);
    ResetArduino();
  }

  if (bme.begin()) {
    SetLCDDisplay(firstLine, "BME Done...");
    pass();
    delay(1000);
    initializeBME();
  }
  else {
    SetLCDDisplay("BME failed...", "Restarting...");
    fail();
    scanI2C();
    delay(1000);
    ResetArduino();
  }
}

void scanI2C(){
  for (int i = 0; i < 127; i++){
    if(testDevice(i)){
      SetLCDDisplay("Scanning I2C...", "Found: " + String(i, HEX));
      delay(500); 
    }
  }
}

bool testDevice(byte address) {
  Wire.beginTransmission(address);
  uint8_t error = Wire.endTransmission();

  if (error == 0)
    return true;
  else
    return false;
}

void threeBeep(int a, int b, int c, int ad, int bd, int cd) {
  digitalWrite(BUZOUT, HIGH);
  delay(a);
  digitalWrite(BUZOUT, LOW);
  delay(ad);

  digitalWrite(BUZOUT, HIGH);
  delay(b);
  digitalWrite(BUZOUT, LOW);
  delay(bd);

  digitalWrite(BUZOUT, HIGH);
  delay(c);
  digitalWrite(BUZOUT, LOW);
  delay(cd);
}
