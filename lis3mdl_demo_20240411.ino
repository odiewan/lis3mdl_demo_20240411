// Basic demo for magnetometer readings from Adafruit LIS3MDL

#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <String.h>

Adafruit_LIS3MDL lis3mdl;
#define LIS3MDL_CLK 13
#define LIS3MDL_MISO 12
#define LIS3MDL_MOSI 11
#define LIS3MDL_CS 10

#define MAG_MIN -200
#define MAG_MAX 200

enum calStates {
  CAL_ST_WAIT,
  CAL_ST_IN_CAL,
  CAL_ST_CAL_DONE,
  CAL_ST_CAL_RESTART,
  NUM_CAL_STATES,
};

String calStateStrs[] = {
  "WAIT",
  "IN_CAL",
  "CAL_DONE",
  "CAL_RESTART",
};

float magX;
float magY;
float magZ;

float magXa;
float magYa;
float magZa;

float minX = MAG_MAX;
float minY = MAG_MAX;
float minZ = MAG_MAX;

float maxX = MAG_MIN;
float maxY = MAG_MIN;
float maxZ = MAG_MIN;

float netX;
float netY;
float netZ;

float vecMag;
float vecMagXY;
float vecMagXYsqrt;

float theta;
float phi;

float theta_min = 360;
float theta_max = -360;

int icount = 0;

int calStat = CAL_ST_WAIT;

int mag_lim_update_x_timer = 0;
int mag_lim_update_y_timer = 0;
int mag_lim_update_z_timer = 0;



//-------------------------------------------------------------------------------------------------
void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LIS3MDL test!");
  
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
  //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
    Serial.println("Failed to find LIS3MDL chip");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found!");

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}

//-------------------------------------------------------------------------------------------------
void loop() {
  lis3mdl.read();      
  
  sensors_event_t event; 
  lis3mdl.getEvent(&event);
  magX = event.magnetic.x;
  magY = event.magnetic.y;
  magZ = event.magnetic.z;


  switch(calStat) {
    case CAL_ST_WAIT:
      if(icount > 30)
        calStat = CAL_ST_IN_CAL;

      break;

    case CAL_ST_IN_CAL:
      mag_lim_update_x_timer++;
      mag_lim_update_y_timer++;
      mag_lim_update_z_timer++;
      if(magX > maxX) {
        maxX = magX;
        mag_lim_update_x_timer = 0;
      }

      if(magX < minX) { 
        minX = magX;
        mag_lim_update_x_timer = 0;
      }
        
      if(magY > maxY) {
        maxY = magY;
        mag_lim_update_y_timer = 0;
      }

      if(magY < minY) {
        minY = magY;
        mag_lim_update_y_timer = 0;
      }
        
      if(magZ > maxZ) {
        maxZ = magZ;
        mag_lim_update_z_timer = 0;
      }

      if(magZ < minZ) {
        minZ = magZ;
        mag_lim_update_z_timer = 0;
        }

      magXa = (maxX - minX)/2;
      magYa = (maxY - minY)/2;
      magZa = (maxZ - minZ)/2;
      
      if(mag_lim_update_x_timer > 150 && mag_lim_update_y_timer > 150 && mag_lim_update_z_timer > 150)
        calStat = CAL_ST_CAL_DONE;

      break;

    case CAL_ST_CAL_DONE:

      break;
   
    case CAL_ST_CAL_RESTART:

      calStat = CAL_ST_IN_CAL;
      break;

    default:
      break;


  }



  netX =  magX - magXa;
  netY =  magY - magYa;
  netZ =  magZ - magZa;

  // vecMagXY = pow(magX, 2) + pow(magY, 2);
  // vecMagXYsqrt = sqrt(vecMagXY);
  // vecMag = vecMagXY + pow(magZ, 2);
  // vecMag = sqrt(vecMag);

  // magX = magX/vecMagXYsqrt;
  // magY = magY/vecMagXYsqrt;
  // magZ = magZ/vecMag;
  Serial.print("ic:");
  Serial.print(icount);
  Serial.print(" calStat:");
  Serial.println(calStateStrs[calStat]);

  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("\tX: ");      Serial.print(magX);
  Serial.print(" \tY: ");     Serial.print(magY);
  Serial.print(" \tZ: ");     Serial.println(magZ);

  Serial.print("\tXmin: ");   Serial.print(minX);
  Serial.print(" \tYmin: ");  Serial.print(minY);
  Serial.print(" \tZmin: ");  Serial.println(minZ);

  Serial.print("\tXmax: ");   Serial.print(maxX);
  Serial.print(" \tYmax: ");  Serial.print(maxY);
  Serial.print(" \tZmax: "); Serial.println(maxZ);

  Serial.print("\t magXa: ");   Serial.print(magXa);
  Serial.print(" \tmagYa: ");   Serial.print(magYa);
  Serial.print(" \tmagZa: "); Serial.println(magZa);

  Serial.print("\t netX: ");   Serial.print(netX);
  Serial.print(" \tnetY: ");   Serial.print(netY);
  Serial.print(" \tnetZ: "); Serial.println(netZ);

  Serial.print("\t mag_lim_update_x_timer: ");   Serial.print(mag_lim_update_x_timer);
  Serial.print(" \tmag_lim_update_y: ");   Serial.print(mag_lim_update_y_timer);
  Serial.print(" \tmag_lim_update_z: "); Serial.println(mag_lim_update_z_timer);

  // Serial.print("\tX: ");  Serial.print(magX); Serial.print(" dX "); Serial.print(DmagX); Serial.print(netDx);
  // Serial.print(" \tY: "); Serial.print(magY); Serial.print(" dY "); Serial.print(DmagY); Serial.print(netDy);
  // Serial.print(" \tZ: "); Serial.print(magZ); Serial.print(" dZ "); Serial.print(DmagZ); Serial.print(netDz);
  Serial.println(" uTesla ");

  theta = atan2(netX, netY)*(180/3.1415792) + 180;

  if(theta < 0)
     theta += 360;

  if(theta < theta_min)
    theta_min = theta;
  if(theta > theta_max)
    theta_max = theta;



  // Serial.print(" vecMagXYsqrt: "); Serial.print(vecMagXYsqrt); Serial.print(" vecMag: "); Serial.print(vecMag);
  // Serial.println(" uTesla ");

  Serial.print(" theta:"); Serial.print(theta);
  Serial.print(" theta_min:"); Serial.print(theta_min);
  Serial.print(" theta_max:"); Serial.print(theta_max);

  // Serial.print(" theta:"); Serial.print(theta * (180/3.1415792));

  delay(100); 
  Serial.println();


  icount++;
}