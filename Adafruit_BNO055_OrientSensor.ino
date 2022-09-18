#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//============================================================================================
// GLOBALS
//============================================================================================
//const float PI = 3.1415;
const float D2R = PI/180.0;
const float R2D = 180.0/PI;

const int ledPin = 13;
bool ledState = true;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28) id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool isCalibrated = false;
float hdg_mag = 0;
float hdg_true = 0;
float pitch = 0;
float roll = 0;
float declination = -10.8186;


//============================================================================================
// FUNCTIONS
//============================================================================================
float angleWrap_deg(float angle)
{
  float aw = angle;
  if( angle > 180.0 )
  {
    aw = angle - 360.0;
  }
  else if( angle < -180.0 )
  {
    aw = angle + 360;
  }
  return aw;
}

// Displays some basic information on this sensor from the unified
// sensor API sensor_t type (see Adafruit_Sensor for more information)
void displayImuDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.print  ("Sensor =      "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver =  "); Serial.println(sensor.version);
  Serial.print  ("Unique ID =   "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value = : "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value = : "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution =  "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("");
  delay(500);
}

// Display some basic info about the sensor status
void displayImuStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.print("System Status = 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test =     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error =  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

//
void displayImuMode(void)
{
  adafruit_bno055_opmode_t mode = bno.getMode();

  switch( mode )
  {
    case OPERATION_MODE_CONFIG:
      Serial.println("OPERATION_MODE_COMPASS");
      break;
    case OPERATION_MODE_COMPASS:
      Serial.println("OPERATION_MODE_COMPASS"); 
      break;    
    case OPERATION_MODE_NDOF_FMC_OFF:
      Serial.println("OPERATION_MODE_NDOF_FMC_OFF"); 
      break;
    case OPERATION_MODE_NDOF:
      Serial.println("OPERATION_MODE_NDOF"); 
      break;
    default:
      Serial.println("OPERATION MODE UNKNOWN"); 
  }
  Serial.println("");
  delay(500);
}


// Display sensor calibration status
// The four calibration registers (an overall system calibration status, 
// as well individual gyroscope, magnetometer and accelerometer values)
// will return a value between '0' (uncalibrated data) and '3' (fully calibrated). 
// The higher the number the better the data will be.
bool displayImuCalibStatus(void)
{  
  uint8_t sys, gyro, accel, mag;
  sys = gyro = accel = mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("Sys = ");
  Serial.print(sys, DEC);
  Serial.print(" G = ");
  Serial.print(gyro, DEC);
  Serial.print(" A = ");
  Serial.print(accel, DEC);
  Serial.print(" M = ");
  Serial.println(mag, DEC);

  if( ((gyro+accel+mag) == 9) && (sys == 3)) return true;
  else return false;
}

// Display the raw calibration offset and radius data
// Formated so you can just past directly into loadImuConfig()
void displayImuCalibOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.println("\nBNO055 Calibration Data");
    Serial.print("calibData.accel_offset_x = "); Serial.print(calibData.accel_offset_x); Serial.println(";");
    Serial.print("calibData.accel_offset_y = "); Serial.print(calibData.accel_offset_y); Serial.println(";");
    Serial.print("calibData.accel_offset_z = "); Serial.print(calibData.accel_offset_z); Serial.println(";");
    Serial.print("calibData.accel_radius = "); Serial.print(calibData.accel_radius); Serial.println(";");

    Serial.print("calibData.gyro_offset_x = "); Serial.print(calibData.gyro_offset_x); Serial.println(";");
    Serial.print("calibData.gyro_offset_y = "); Serial.print(calibData.gyro_offset_y); Serial.println(";");
    Serial.print("calibData.gyro_offset_z = "); Serial.print(calibData.gyro_offset_z); Serial.println(";");

    Serial.print("calibData.mag_offset_x = "); Serial.print(calibData.mag_offset_x); Serial.println(";");
    Serial.print("calibData.mag_offset_y = "); Serial.print(calibData.mag_offset_y); Serial.println(";");
    Serial.print("calibData.mag_offset_z = "); Serial.print(calibData.mag_offset_z); Serial.println(";");
    Serial.print("calibData.mag_radius = "); Serial.print(calibData.mag_radius); Serial.println(";");

    Serial.println("");
}

// This will eliminate the need to calibrate the accelerometers!!!
// Note that the magnetometers will still need to be calibrated
// The gyros still require the sensor to be motionless at start up
bool loadImuCalibration()
{
  adafruit_bno055_offsets_t calibData;
  bno.getSensorOffsets( calibData );
  displayImuCalibOffsets( calibData );
  
  Serial.println("Restoring calibration data to the BNO055");  
  calibData.accel_offset_x = 16;
  calibData.accel_offset_y = -23;
  calibData.accel_offset_z = -5;
  calibData.accel_radius = 1000;
  
  calibData.gyro_offset_x = 1;
  calibData.gyro_offset_y = -2;
  calibData.gyro_offset_z = 1;
  
  calibData.mag_offset_x = -122;
  calibData.mag_offset_y = 424;
  calibData.mag_offset_z = -163; 
  calibData.mag_radius = 778;  
  
  bno.setSensorOffsets(calibData);

  displayImuCalibOffsets( calibData );

  return true;
}


//============================================================================================
// SETUP
//============================================================================================
void setup() 
{
  pinMode(ledPin, OUTPUT);

  Serial.begin(38400);
  Serial.println("Adafruit BNO055 Orientation Sensor\n");

  // configure serial plotter heading
  //Serial.println("hdg_mag:,hdg_true:,pitch:,roll:");
  
  // Initialise the BNO055 sensor 
  // --> defaults to OPERATION_MODE_NDOF, which is Fast Magnetic Calibration (FMC) mode ON
  //if(!bno.begin(OPERATION_MODE_COMPASS))  
  if(!bno.begin())  
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1)
    {
      delay(10);
    }
  }  
  delay(500);    
  
  bno.setExtCrystalUse(true); 
  delay(500);

//  bno.setAxisRemap(Adafruit_BNO055::adafruit_bno055_axis_remap_config_t::REMAP_CONFIG_P0);
//  bno.setAxisSign(Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t::REMAP_SIGN_P0);
  
//  displayImuDetails();
//  displayImuStatus();
//  displayImuMode(); 


  //*****************************************************************************************
  // For a fast calibration, uncomment this line and use the most recently saved calibration.
  // You will still need to calibrate the magnetometers by mvoing it around.
  //loadImuCalibration();
  //*****************************************************************************************
}


//============================================================================================
// LOOP
//============================================================================================
void loop() 
{
  //----------------------------------------------------------------
  // Blink LED
  //----------------------------------------------------------------
  ledState = !ledState;
  digitalWrite(ledPin, ledState);   // set the LED on


  //----------------------------------------------------------------
  // IMU
  //----------------------------------------------------------------    
  if( !isCalibrated )  
  {
    if( displayImuCalibStatus() )
    {
      isCalibrated = true;

      adafruit_bno055_offsets_t calibData;
      bno.getSensorOffsets( calibData );
      displayImuCalibOffsets( calibData );
    }
  }
  else  
  {
    /* Get a new sensor event */ 
//    sensors_event_t event; 
//    bno.getEvent(&event);
//    hdg_mag = event.orientation.x;
//    pitch = event.orientation.y;
//    roll = event.orientation.z;
  
  
    // Possible vector values can be:
    // - VECTOR_ACCELEROMETER - m/s^2
    // - VECTOR_MAGNETOMETER  - uT
    // - VECTOR_GYROSCOPE     - rad/s
    // - VECTOR_EULER         - degrees
    // - VECTOR_LINEARACCEL   - m/s^2
    // - VECTOR_GRAVITY       - m/s^2
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    // P1 (default)
    hdg_mag = euler.x() + 90.0; // CW is positive; seems to be off 90 deg
    pitch = euler.y();  // up is positive
    roll = euler.z();   // CCW (left) is positive

    // angle wrap
    hdg_mag = angleWrap_deg(hdg_mag);
    pitch = angleWrap_deg(pitch);
    roll = angleWrap_deg(roll);

    // declination adjustment
    hdg_true = hdg_mag - declination;
    hdg_true = angleWrap_deg(hdg_true);
  
    Serial.print("hdg_mag:");
    Serial.print(hdg_mag, 4);
    Serial.print(" hdg_true:");
    Serial.print(hdg_true, 4);
    Serial.print(" pitch:");
    Serial.print(pitch, 4);
    Serial.print(" roll:");
    Serial.print(roll, 4);
    Serial.println("");
  } //if( !isCalibrated ) 
  
  delay(100);
}
