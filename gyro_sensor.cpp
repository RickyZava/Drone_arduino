#include "gyro_sensor.h"

gyro_sensor :: gyro_sensor(){

    lastTime=0;
    dt=0;
    loopFrequency=0;

    yaw=0;
    rawRoll=0;
    rawPitch=0;
    filteredPitch=0;
    filteredRoll=0;
    alpha=0.5;//low pass filer
    beta=0.9;//complementary filter
    complPitch=0;
    complRoll=0;


    gyroXoffset=0;
    gyroYoffset=0;
    gyroZoffset=0;

}


void gyro_sensor :: initialization(){

    Wire.begin();

  Wire.setClock(400000);
  Serial.begin(115200);
  // Init the module
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("MPU initializated");
  // Check module connection
  Serial.println("Testing device connections...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
  while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

lastTime = micros();



};

void gyro_sensor :: update(){
int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long currentTime = micros();
  dt = (currentTime - lastTime) / 1000000.0;
  lastTime = currentTime;
  // 1. Apply Gyro Offsets
  float true_gx = gx - gyroXoffset;
  float true_gy = gy - gyroYoffset;
  float true_gz = gz - gyroZoffset;

  // 2. Apply Accel Offsets
  float f_ax = (float)ax - accelXoffset;
  float f_ay = (float)ay - accelYoffset;
  float f_az = (float)az - accelZoffset;
  
  // 3. Process Accelerometer (Pitch and Roll)
  rawPitch = atan2(-f_ax, sqrt(f_ay * f_ay + f_az * f_az)) * 180.0 / PI;
  rawRoll  = atan2(f_ay, f_az) * 180.0 / PI;

  // 4. Process Gyroscope (Yaw integration)
  float gyroZRate = true_gz / 131.0; 
  yaw += gyroZRate * dt;

  // 5. Apply Low-Pass Filter
  filteredPitch = (alpha * rawPitch) + ((1.0 - alpha) * filteredPitch);
  filteredRoll  = (alpha * rawRoll)  + ((1.0 - alpha) * filteredRoll);


// 4. Process Gyroscope (Convert raw data to degrees per second)
  gyroXRate = true_gx / 131.0; 
  gyroYRate = true_gy / 131.0;
  //float gyroZRate = true_gz / 131.0; 

  //yaw += gyroZRate * dt;

  // 5. Apply Complementary Filter
  // Note: We use rawPitch/rawRoll here, you do not need to pre-filter them.
  complPitch = beta * (complPitch + (gyroYRate * dt)) + (1.0 - beta) * rawPitch;
  complRoll  = beta * (complRoll  + (gyroXRate * dt)) + (1.0 - beta) * rawRoll;


  //kalman filtering
  kalmanFilter(gx,gy);
}

void gyro_sensor::calibrate(){
Serial.println("Calibrating Gyro & Accel... DO NOT MOVE DRONE!");
  
  long gxSum = 0, gySum = 0, gzSum = 0;
  long axSum = 0, aySum = 0, azSum = 0;
  const int numSamples = 2000;
  
  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    gxSum += gx;
    gySum += gy;
    gzSum += gz;
    
    axSum += ax;
    aySum += ay;
    azSum += az;
    
    delay(3);
  }
  
  // Calculate average gyro offsets, that are the angular velocity offset
  gyroXoffset = (float)gxSum / numSamples;
  gyroYoffset = (float)gySum / numSamples;
  gyroZoffset = (float)gzSum / numSamples;
  
  // Calculate average accelerometer offsets
  accelXoffset = (float)axSum / numSamples;
  accelYoffset = (float)aySum / numSamples;
  // Subtract 16384 from Z because gravity (1G) is acting on it!
  accelZoffset = ((float)azSum / numSamples) - 16384.0; 
  
  Serial.println("Calibration Complete!");
  kalmanBiasPitch= gyroYoffset/131.0;
  kalmanBiasRoll= gyroXoffset/131.0;
  delay(3000);
}

void gyro_sensor::kalmanFilter(const int16_t gx, const int16_t gy) {
  // -------------------------
  // PITCH KALMAN FILTER
  // -------------------------
  // 1. Predict


  //Serial.print("dt:"); Serial.print(dt, 6); // Print with 6 decimal places
  //Serial.print(", gyroY:"); Serial.println(gyroYRate);
  //applying kalman filter to each angle separately

  float ratePitch = (gy/131.0) - kalmanBiasPitch;//wk-1 - b_k-1
  kalmanPitch += dt * ratePitch;//dt(wk-1 - b_k-1)  apriori prediction
  

  //P_k|k-1
  P_pitch[0][0] += dt * (dt * P_pitch[1][1] - P_pitch[0][1] - P_pitch[1][0] + Q_angle);
  P_pitch[0][1] -= dt * P_pitch[1][1];
  P_pitch[1][0] -= dt * P_pitch[1][1];
  P_pitch[1][1] += Q_bias * dt;

  // 2. Update
  //s_k
  float S_pitch = P_pitch[0][0] + R_measure;
  //K_k
  float K_pitch[2] = { P_pitch[0][0] / S_pitch, P_pitch[1][0] / S_pitch };
  //y_k|k=z_k(measurement)-x_k|k-1
  float y_pitch = rawPitch - kalmanPitch;//error for innovation, measuremet

  kalmanPitch += K_pitch[0] * y_pitch; //x_k|k
  kalmanBiasPitch += K_pitch[1] * y_pitch;//x_k|k
  
  P_pitch[0][0] -= K_pitch[0] * P_pitch[0][0];
  P_pitch[0][1] -= K_pitch[0] * P_pitch[0][1];
  P_pitch[1][0] -= K_pitch[1] * P_pitch[0][0];
  P_pitch[1][1] -= K_pitch[1] * P_pitch[0][1];

  // -------------------------
  // ROLL KALMAN FILTER
  // -------------------------
  // 1. Predict
  float rateRoll = (gx/131.0) - kalmanBiasRoll;
  kalmanRoll += dt * rateRoll;
  
  P_roll[0][0] += dt * (dt * P_roll[1][1] - P_roll[0][1] - P_roll[1][0] + Q_angle);
  P_roll[0][1] -= dt * P_roll[1][1];
  P_roll[1][0] -= dt * P_roll[1][1];
  P_roll[1][1] += Q_bias * dt;

  // 2. Update
  float S_roll = P_roll[0][0] + R_measure;
  float K_roll[2] = { P_roll[0][0] / S_roll, P_roll[1][0] / S_roll };
  float y_roll = rawRoll - kalmanRoll;
  
  kalmanRoll += K_roll[0] * y_roll;
  kalmanBiasRoll += K_roll[1] * y_roll;
  
  P_roll[0][0] -= K_roll[0] * P_roll[0][0];
  P_roll[0][1] -= K_roll[0] * P_roll[0][1];
  P_roll[1][0] -= K_roll[1] * P_roll[0][0];
  P_roll[1][1] -= K_roll[1] * P_roll[0][1];
}

float gyro_sensor::getFilteredPitch() { return filteredPitch; }
float gyro_sensor::getFilteredRoll() { return filteredRoll; }

float gyro_sensor::getRawPitch() { return rawPitch;}
float gyro_sensor::getRawRoll() { return rawRoll;}

float gyro_sensor::getComplPitch(){return complPitch;}
float gyro_sensor::getComplRoll(){return complRoll;}

float gyro_sensor::getKalmanPitch(){return kalmanPitch;}
float gyro_sensor::getKalmanRoll(){return kalmanRoll;}






float gyro_sensor::getYaw() { return yaw; }









