#include "gyro_sensor.h"

gyro_sensor drone_gyro;

unsigned long lastPrintTime = 0;

void setup() {
  // put your setup code here, to run once:
drone_gyro.initialization();//initializing communication...
drone_gyro.calibrate();//determining offset by average measurements
lastPrintTime=micros();
}


void loop() {
  unsigned long currentTime = micros();
  

  drone_gyro.update();
  
 

  // 2. Telemetry Output (Runs only 10 times per second)
  if (currentTime - lastPrintTime > 50000) { //plot at 20 hz 
    lastPrintTime = currentTime; // Reset the print timer
    Serial.print("RawPitch:");
    Serial.print(drone_gyro.getRawPitch());
    Serial.print(","); // Comma separates the lines on the graph
    Serial.print("RawRoll:");
    Serial.println(drone_gyro.getRawRoll()); // println finishes the data frame
    
    Serial.print("FilteredPitch:");
    Serial.print(drone_gyro.getFilteredPitch());
    Serial.print(","); // Comma separates the lines on the graph
    Serial.print("FilteredRoll:");
    Serial.println(drone_gyro.getFilteredRoll());
    
    Serial.print("ComplPitch:");
    Serial.print(drone_gyro.getComplPitch());
    Serial.print(","); // Comma separates the lines on the graph
    Serial.print("ComplRoll:");
    Serial.println(drone_gyro.getComplRoll());
    

    Serial.print("KalmanPitch:");
    Serial.print(drone_gyro.getKalmanPitch());
    Serial.print(","); // Comma separates the lines on the graph
    Serial.print("KalmanRoll:");
    Serial.println(drone_gyro.getKalmanRoll());

    Serial.print(","); // Comma separates the lines on the graph
    Serial.print("Yaw:");
    Serial.println(drone_gyro.getYaw());
    // Only print 10 times a second to prevent USB bottlenecking
    //Serial.print("Sensor reading frequency [Hz]: "); 
    //Serial.println(loopFrequency);
   }




}