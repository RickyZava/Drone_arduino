#ifndef gyro_sensor_h
#define gyro_sensor_h


#include "I2Cdev.h"
#include "MPU6050.h"
#include <math.h>
#include "Wire.h"


class gyro_sensor
{
  public:
    gyro_sensor();
    void initialization(); //initialize the sensor
    void update(); // read data

    float getFilteredPitch();
    float getFilteredRoll();
    float getRawPitch();
    float getRawRoll();
    float getComplPitch();
    float getComplRoll();
    float getYaw();
    float getFrequency();
    void calibrate();
    void kalmanFilter(const int16_t gx,const int16_t gy);
    float getKalmanPitch();
    float getKalmanRoll();

    


  

  private:
    MPU6050 mpu;
    unsigned long lastTime;
    float dt;
    float loopFrequency;

    float yaw;
    float rawPitch;
    float rawRoll;
    float filteredPitch;
    float filteredRoll;
    float complPitch;
    float complRoll;


    float beta;
    float alpha;
    float gyroXoffset, gyroYoffset, gyroZoffset;
    float accelXoffset=0;
    float accelYoffset=0;
    float accelZoffset=0;
    float gyroXRate;
    float gyroYRate;

    float kalmanPitch=0;
    float kalmanRoll=0;
    float kalmanBiasPitch=0;
    float kalmanBiasRoll=0;
    float P_pitch[2][2]={{0,0},{0,0}};
    float P_roll[2][2]={{0,0},{0,0}};

    const float Q_angle=0.01;
    const float Q_bias=0.003;
    const float R_measure = 0.07;



};
#endif