//Blynk
#define BLYNK_TEMPLATE_ID "TMPL21RkzJMaP"
#define BLYNK_TEMPLATE_NAME "TCC Vinhal e Derek"
#define BLYNK_AUTH_TOKEN "qpIMuyJwo49WDOrkPGWlX7zzhfmTxyiR"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>

#define VPIN_TEMPERATURE V0
#define VPIN_HUMIDITY V1
#define VPIN_AIRTEMP V2
#define VPIN_PMV V3
#define VPIN_PPD V4
#define VPIN_PRESENCE V5

//ISO 7730
const float CLOTHING_INSULATION = 0.5;
const float METABOLIC_RATE = 1.2;
const float AIR_VELOCITY = 0.1;

struct PMVResult {
  float pmv;
  float ppd;
};

//Wifi
char ssid[] = "CLAROALLAN";
char pass[] = "Dedelu147.";

BlynkTimer timer;

//DHT22
#define DHTPIN 16
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

//LCD I2C
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
#define SDA_PIN 21
#define SCL_PIN 22
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

//Botões
#define BUTTON_ON 18
#define BUTTON_OFF 19

//Sensor PIR
#define PIR_PIN 13

//Infravermelho
#define IR_PIN 4

//LED embutido
#define LED_EM_PIN 2

const uint32_t AC_POWER_ON_CODE = 0xB2BF70;
const uint32_t AC_CODE_17 = 0xB2BF00;
const uint32_t AC_CODE_18 = 0xB2BF10;
const uint32_t AC_CODE_19 = 0xB2BF30;
const uint32_t AC_CODE_20 = 0xB2BF20;
const uint32_t AC_CODE_21 = 0xB2BF60;
const uint32_t AC_CODE_22 = 0xB2BF70;
const uint32_t AC_CODE_23 = 0xB2BF50;
const uint32_t AC_CODE_24 = 0xB2BF40;
const uint32_t AC_CODE_25 = 0xB2BFC0;
const uint32_t AC_CODE_26 = 0xB2BFD0;
const uint32_t AC_CODE_27 = 0xB2BF90;
const uint32_t AC_CODE_28 = 0xB2BF80;
const uint32_t AC_CODE_29 = 0xB2BFA0;
const uint32_t AC_CODE_30 = 0xB2BFB0;
const uint32_t AC_POWER_OFF_CODE = 0xB27BE0;

IRsend irsend(IR_PIN);

int ligado = 0;
float umidade = 0;
float temperatura = 0;
int tempArAtual = 0;
int presenca = 0;

unsigned long ultimaPresenca = 0;
bool acDesligadoPorAusencia = false;

float calculateSaturationPressure(float tempC) {
  return 610.7 * exp((17.27 * tempC) / (237.3 + tempC));
}

PMVResult calcularIndicadores(float airTemp, float relativeHumidity){
  float tr = airTemp;
  float pa = (relativeHumidity / 100.0) * calculateSaturationPressure(airTemp);
  float M = METABOLIC_RATE * 58.15;
  float W = 0.0;
  float I_cl = CLOTHING_INSULATION * 0.155;
  float f_cl = (I_cl < 0.078) ? 1.0 + 1.29 * I_cl : 1.05 + 0.645 * I_cl;
  float M_n = M - W;
  float h_c;
  
  if (AIR_VELOCITY < 0.1) {
    h_c = 2.38 * pow(fabs(airTemp - 34.0), 0.25);
  } else {
    h_c = 12.1 * sqrt(AIR_VELOCITY);
  }
  
  float t_cl = airTemp + (35.5 - airTemp) / (3.5 * (6.45 * I_cl + 0.1));
  float h_r = 4.0 * 5.67e-8 * 0.95 * pow((t_cl + tr) / 2.0 + 273.0, 3);
  
  for (int i = 0; i < 50; i++) {
    float t_cl_new = (35.7 - 0.028 * M_n - I_cl * f_cl * (h_r * (t_cl - tr) + h_c * (t_cl - airTemp)));
    if (fabs(t_cl_new - t_cl) < 0.01) {
      t_cl = t_cl_new;
      break;
    }
    t_cl = t_cl_new;
  }
  
  float E_sk = 3.05 * 0.001 * (5733.0 - 6.99 * M_n - pa);
  float E_sw = (M_n > 58.15) ? 0.42 * (M_n - 58.15) : 0.0;
  float E_re = 1.7 * 0.00001 * M * (5867.0 - pa);
  float C_re = 0.0014 * M * (34.0 - airTemp);
  float R = f_cl * h_r * (t_cl - tr);
  float C = f_cl * h_c * (t_cl - airTemp);
  float L = M_n - E_sk - E_sw - E_re - C_re - R - C;
  float F_s = 0.303 * exp(-0.036 * M) + 0.028;
  float pmv = F_s * L;
  
  pmv = fmax(fmin(pmv, 3.0), -3.0);
  float ppd = 100.0 - 95.0 * exp(-0.03353 * pow(pmv, 4) - 0.2179 * pow(pmv, 2));
  
  return {pmv, ppd};
}

float calcularTempConf(float relativeHumidity){
  float bestTemp = 22.0;
  float bestPmv = 10.0;
  
  for (float temp = 18.0; temp <= 30.0; temp += 0.1) {
    PMVResult result = calcularIndicadores(temp, relativeHumidity);
    if (fabs(result.pmv) < fabs(bestPmv)) {
      bestPmv = result.pmv;
      bestTemp = round(temp);
    }
    if (fabs(result.pmv) < 0.05) break;
  }
  
  return bestTemp;
}

void EnviarTempConf(int ConfTemp){
  switch (ConfTemp){
    case 17: irsend.sendCOOLIX(AC_CODE_17, 24); break;
    case 18: irsend.sendCOOLIX(AC_CODE_18, 24); break;
    case 19: irsend.sendCOOLIX(AC_CODE_19, 24); break;
    case 20: irsend.sendCOOLIX(AC_CODE_20, 24); break;
    case 21: irsend.sendCOOLIX(AC_CODE_21, 24); break;
    case 22: irsend.sendCOOLIX(AC_CODE_22, 24); break;
    case 23: irsend.sendCOOLIX(AC_CODE_23, 24); break;
    case 24: irsend.sendCOOLIX(AC_CODE_24, 24); break;
    case 25: irsend.sendCOOLIX(AC_CODE_25, 24); break;
    case 26: irsend.sendCOOLIX(AC_CODE_26, 24); break;
    case 27: irsend.sendCOOLIX(AC_CODE_27, 24); break;
    case 28: irsend.sendCOOLIX(AC_CODE_28, 24); break;
    case 29: irsend.sendCOOLIX(AC_CODE_29, 24); break;
    case 30: irsend.sendCOOLIX(AC_CODE_30, 24); break;
  }
}

void sendSensorData() {
  umidade = dht.readHumidity();
  temperatura = dht.readTemperature();
  
  if (isnan(umidade) || isnan(temperatura)) {
    Serial.println("Erro ao ler dados do sensor DHT!");
    lcd.clear();
    delay(10);
    lcd.setCursor(0, 0);
    lcd.print("Erro no Sensor");
    return;
  }
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Temp: ");
  lcd.print(temperatura, 1);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print(" Umid: ");
  lcd.print(umidade, 1);
  lcd.print("%");
  
  Blynk.virtualWrite(VPIN_TEMPERATURE, temperatura);
  Blynk.virtualWrite(VPIN_HUMIDITY, umidade);
  Serial.printf("Temperatura: %.1f °C | Umidade: %.1f %%\n", temperatura, umidade);
  
  digitalWrite(LED_EM_PIN, LOW);
  
  PMVResult current = calcularIndicadores(temperatura, umidade);
  Serial.print("PMV: ");
  Serial.println(current.pmv);
  Serial.print("PPD: ");
  Serial.println(current.ppd);
  
  Blynk.virtualWrite(VPIN_PMV, current.pmv);
  Blynk.virtualWrite(VPIN_PPD, current.ppd);
  
  int tempArNova = calcularTempConf(umidade);
  if (tempArAtual != tempArNova){
    EnviarTempConf(tempArNova);
    tempArAtual = tempArNova;
  }
}

void setup() {
  Serial.begin(9600);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  dht.begin();
  lcd.init();
  lcd.backlight();
  pinMode(BUTTON_ON, INPUT_PULLUP);
  pinMode(BUTTON_OFF, INPUT_PULLUP);
  pinMode(PIR_PIN, INPUT);
  irsend.begin();
  
  timer.setInterval(20000L, sendSensorData);
}

void loop() {
  Blynk.run();
  
  if (digitalRead(BUTTON_ON) == HIGH) {
    Serial.println("Botão ON pressionado");
    ligado = 1;
    acDesligadoPorAusencia = false;
    irsend.sendCOOLIX(AC_POWER_ON_CODE, 24);
    delay(1000);
    
    umidade = dht.readHumidity();
    temperatura = dht.readTemperature();
    int tempConf = calcularTempConf(umidade);
    EnviarTempConf(tempConf);
    tempArAtual = tempConf;
    Blynk.virtualWrite(VPIN_AIRTEMP, tempArAtual);
    delay(500);
  } else if (digitalRead(BUTTON_OFF) == HIGH){
    Serial.println("Botão OFF pressionado");
    ligado = 0;
    acDesligadoPorAusencia = false;
    irsend.sendCOOLIX(AC_POWER_OFF_CODE, 24);
    delay(500);
  }
  
  if(ligado == 1){
    timer.run();
    
    int val = digitalRead(PIR_PIN);
    
    if (val == HIGH) {
      Serial.println("Presença detectada");
      presenca = 1;
      ultimaPresenca = millis();
      
      if (acDesligadoPorAusencia) {
        Serial.println("Religando AC após ausência");
        irsend.sendCOOLIX(AC_POWER_ON_CODE, 24);
        delay(1500);
        umidade = dht.readHumidity();
        int tempConf = calcularTempConf(umidade);
        EnviarTempConf(tempConf);
        tempArAtual = tempConf;
        Blynk.virtualWrite(VPIN_AIRTEMP, tempArAtual);
        acDesligadoPorAusencia = false;
      }
      
      Blynk.virtualWrite(VPIN_PRESENCE, presenca);
    } else {
      if (presenca == 1 && (millis() - ultimaPresenca >= 1800000)) {
        Serial.println("Ausência prolongada - desligando AC");
        presenca = 0;
        Blynk.virtualWrite(VPIN_PRESENCE, presenca);
        irsend.sendCOOLIX(AC_POWER_OFF_CODE, 24);
        acDesligadoPorAusencia = true;
      }
    }
    
    delay(500);
  }
}