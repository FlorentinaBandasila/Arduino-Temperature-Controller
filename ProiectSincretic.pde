#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LM35_SENSOR_PIN A0
#define MOSFET_PIN 8
#define LED_PIN 7

LiquidCrystal_I2C lcd(0x27, 16, 2);

const double Kp = 10.0;        
const double Ki = 0.1;         
const double Kd = 1.0;        
double desiredTemperature = 30; 
const double hysteresis = 0.1;  

double integral = 0.0;
double lastError = 0.0;
double smoothedTemperature = 25.0;  
const double alpha = 0.1;  

void readDesiredTemperature() {
 
  Serial.println("Enter the desired temperature:");
  while (!Serial.available()) {
    
  }
  desiredTemperature = Serial.parseFloat();
  Serial.print("Desired Temp set to: ");
  Serial.println(desiredTemperature);
}

void setup() {
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setBacklight(150);

  
  int initialTempADC = analogRead(LM35_SENSOR_PIN);
  double initialTemp = (initialTempADC * 4.8) / 10;
  smoothedTemperature = initialTemp;

  
  readDesiredTemperature();
}

void loop() {
  lcd.clear();
  
  int temp_adc_val = analogRead(LM35_SENSOR_PIN);
  double temp_val = (temp_adc_val * 4.8) / 10;

  
  smoothedTemperature = alpha * temp_val + (1 - alpha) * smoothedTemperature;

  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(smoothedTemperature);

  
  if (isWithinHysteresis()) {
    
    digitalWrite(LED_PIN, LOW);

    
    if (smoothedTemperature < desiredTemperature - 1) {
      digitalWrite(MOSFET_PIN, HIGH);
    } else {
      digitalWrite(MOSFET_PIN, LOW);
      
      
      while (true) {
        delay(1000); 
      }
    }
  } else {
    
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(MOSFET_PIN, HIGH);
  }

  
  delay(500);
}

bool isWithinHysteresis() {
  return (smoothedTemperature >= desiredTemperature - hysteresis) && (smoothedTemperature <= desiredTemperature + hysteresis);
}
