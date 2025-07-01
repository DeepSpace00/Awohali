#include <Arduino.h>
#include <Wire.h>
#include <sht4x.h>


SHT4X_Result env;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Wire.begin();
  if(!sht4x_init()){
    Serial.println("Init failed...");
    while(true){
      delay(10);
    }
  }
  Serial.println("SHT4x initialized successfully.");
}

void loop() {
  if(sht4x_read(&env) == SHT4X_OK) {
    Serial.println(sht4x_read(&env));
    char buffer[32];
    sprintf(buffer, "Temp: %0.2f C, RH: %0.2f %%\n", env.temperature_c, env.humidity_rh);
  }
  else{
    Serial.println("Failed to read sensor data.");
  }
  delay(1000);
}