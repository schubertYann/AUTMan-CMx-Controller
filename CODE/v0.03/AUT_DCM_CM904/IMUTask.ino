//=====================================================================================================
// AUTMan Humanoid ROBOT 2014
// OpenCM9.04 + GY-80 
//=====================================================================================================
//
// Mojtaba's implementation of AUTMan IMU algorithm.
// See: http://autman.aut.ac.ir
//      http://mojtaba.info.se/
//
// Date			Author			        Notes
// 01/01/2014		Mojtaba.K	        	Initial release
//=====================================================================================================
// The Sensor IMU Task for sensor reading and fusion alghorithm
//=====================================================================================================

#define Gyro_Scale              (0.00126)  // define the gyro scale that represent it in deg/s from gyro data
#define HMC5883L_Address        (0x1E)     // Address of i2c Device Magnometer HMC5883
#define ADXL345_Address         (0x53)     // ADXL345 device address
#define L3G4200D_Address        (105)      // I2C address of the L3G4200D
 
#define ADXL345_DEVID           0x00
#define ADXL345_RESERVED1       0x01
#define ADXL345_THRESH_TAP      0x1d
#define ADXL345_OFSX            0x1e
#define ADXL345_OFSY            0x1f
#define ADXL345_OFSZ            0x20
#define ADXL345_DUR             0x21
#define ADXL345_LATENT          0x22
#define ADXL345_WINDOW          0x23
#define ADXL345_THRESH_ACT      0x24
#define ADXL345_THRESH_INACT    0x25
#define ADXL345_TIME_INACT      0x26
#define ADXL345_ACT_INACT_CTL   0x27
#define ADXL345_THRESH_FF       0x28
#define ADXL345_TIME_FF         0x29
#define ADXL345_TAP_AXES        0x2a
#define ADXL345_ACT_TAP_STATUS  0x2b
#define ADXL345_BW_RATE         0x2c
#define ADXL345_POWER_CTL       0x2d
#define ADXL345_INT_ENABLE      0x2e
#define ADXL345_INT_MAP         0x2f
#define ADXL345_INT_SOURCE      0x30
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_DATAX0          0x32
#define ADXL345_DATAX1          0x33
#define ADXL345_DATAY0          0x34
#define ADXL345_DATAY1          0x35
#define ADXL345_DATAZ0          0x36
#define ADXL345_DATAZ1          0x37
#define ADXL345_FIFO_CTL        0x38
#define ADXL345_FIFO_STATUS     0x39

#define ADXL345_BW_1600   0xF // 1111
#define ADXL345_BW_800    0xE // 1110
#define ADXL345_BW_400    0xD // 1101  
#define ADXL345_BW_200    0xC // 1100
#define ADXL345_BW_100    0xB // 1011  
#define ADXL345_BW_50     0xA // 1010 
#define ADXL345_BW_25     0x9 // 1001 
#define ADXL345_BW_12     0x8 // 1000 
#define ADXL345_BW_6      0x7 // 0111
#define ADXL345_BW_3      0x6 // 0110
 
// Interrupt PINs ,INT1: 0 ,INT2: 1
#define ADXL345_INT1_PIN 0x00
#define ADXL345_INT2_PIN 0x01

//Interrupt bit position
#define ADXL345_INT_DATA_READY_BIT 0x07
#define ADXL345_INT_SINGLE_TAP_BIT 0x06
#define ADXL345_INT_DOUBLE_TAP_BIT 0x05
#define ADXL345_INT_ACTIVITY_BIT   0x04
#define ADXL345_INT_INACTIVITY_BIT 0x03
#define ADXL345_INT_FREE_FALL_BIT  0x02
#define ADXL345_INT_WATERMARK_BIT  0x01
#define ADXL345_INT_OVERRUNY_BIT   0x00

#define ADXL345_DATA_READY         0x07
#define ADXL345_SINGLE_TAP         0x06
#define ADXL345_DOUBLE_TAP         0x05
#define ADXL345_ACTIVITY           0x04
#define ADXL345_INACTIVITY         0x03
#define ADXL345_FREE_FALL          0x02
#define ADXL345_WATERMARK          0x01
#define ADXL345_OVERRUNY           0x00

#define ADXL345_OK                 1 // no error
#define ADXL345_ERROR              0 // indicates error is predent

#define ADXL345_NO_ERROR           0 // initial state
#define ADXL345_READ_ERROR         1 // problem reading accel
#define ADXL345_BAD_ARG            2 // bad method argument

#define ADXL345_TO_READ            7    // num of bytes we are going to read each time (two bytes for each axis)

//Accelorometer varible
byte ADXL345_status;   // set when error occurs 
byte error_code;       // Initial state
double gains[3];       // counts to Gs		
double getRate();
byte _buff[6] ;        //6 bytes buffer for saving data read from the device

//Internal register of Gyro sensor L3G4200D
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

float P_roll=0 , P_pitch=0 , P_yaw=0;
float sampleFreq=300.0;
float betaDef=0.1;                      // 2 * proportional gain
float beta = betaDef;			// 2 * proportional gain (Kp)

//*****************************************************************************************************
void vSIMUTask( void *pvParameters )
{
  vTaskDelay(100);
  setupL3G4200D(2000);                           // Configure L3G4200  - 250, 500 or 2000 deg/sec  
  setup_ADXL345();                               // Configure ADXL345 for first time 
  setup_HMC5883L();                              // Configure HMC5883 magnometer sensor
  vTaskDelay(100);
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount ();
  ///----------------------------------
  for( ;; )
  { 
    SIMUTaskCNT++;
    
    //vTaskSuspendAll();   //Suspend all other tasks  
    //ADXL3450_Read();     //Read Accelerometer
    //L3G4200D_Read();     //Read Gyroscope 
    //HMC5883L_Read();     //Read Magnometer  
    //xTaskResumeAll();   //Resume all other tasks
    
    //float Hz=(float)IMUTaskHz; //Get current task frequency
    //IMUUpdate((float)Gyr_x*Gyro_Scale , (float)Gyr_y*Gyro_Scale , (float)Gyr_z*Gyro_Scale , Acc_x , Acc_y , Acc_z, 0 , 0, 0, Hz);                 //No Compass Used
    //IMUUpdate((float)Gyr_x*Gyro_Scale , (float)Gyr_y*Gyro_Scale , (float)Gyr_z*Gyro_Scale , Acc_x , Acc_y , Acc_z, Cmp_x , Cmp_y , Cmp_z , Hz); //Compass data are used
    //getRollPitchYaw();   //To calculate angle from quaternian. 
    
    //xTaskResumeAll();   //Resume all other tasks
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

/*
* get Euler angles
* aerospace sequence, to obtain sensor attitude:
* 1. rotate around sensor Z plane by yaw
* 2. rotate around sensor Y plane by pitch
* 3. rotate around sensor X plane by roll
*/
void getRollPitchYaw() 
{
  roll=(0.9 * P_roll) + (0.1*(atan2(2 * (q0 * q1 + q2 * q3),1 - 2 * (q1 * q1 + q2 * q2))*RAD2DEG));
  pitch =(0.9 * P_pitch) + (0.1*(asin(2 * (q0 * q2 - q3 * q1))*RAD2DEG));
  yaw =(0.9 * P_yaw) + (0.1*  (atan2(2 * (q0 * q3 + q1 * q2) , 1 - 2* (q2 * q2 + q3 * q3))*RAD2DEG));
  
  P_roll=roll;  P_pitch=pitch;  P_yaw=yaw;
}

/*
* Fast inverse square-root
* See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*/
float invSqrt(float x) 
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/*
* Optimum Fast inverse square-root
*/	
float InvSqrt_Opt(float x)
{
  long i = 0x5F1F1412 - (*(long*)&x >> 1);
  float tmp = *((float*)&i);
  return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}

/*
* AHRS algorithm update
*/
void IMUUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float _Hz) 
{	        
        if(_Hz==0) 
          sampleFreq=300.0;
        else
          sampleFreq=_Hz;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
        {
	  IMUUpdate_GA(gx, gy, gz, ax, ay, az);
	  return;
	}
        
        float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

/*
* IMU algorithm update
*/
void IMUUpdate_GA(float gx, float gy, float gz, float ax, float ay, float az) 
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
        {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (1.0f / sampleFreq);
	q1 += qDot2 * (1.0f / sampleFreq);
	q2 += qDot3 * (1.0f / sampleFreq);
	q3 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

/*
* L
*/


//Setup Magnometer sensor for first time
void setup_HMC5883L()
{
  writeRegister(0x1E, 0x02, 0x00); 
}

//Read data from magnometer sensor
void HMC5883L_Read()
{
  byte xMSB =(byte) readRegister(HMC5883L_Address, 0x03);
  byte xLSB =(byte) readRegister(HMC5883L_Address, 0x04);
  Cmp_x = ((xMSB << 8) | xLSB);

  byte yMSB =(byte) readRegister(HMC5883L_Address, 0x05);
  byte yLSB =(byte) readRegister(HMC5883L_Address, 0x06);
  Cmp_y = ((yMSB << 8) | yLSB);

  byte zMSB =(byte) readRegister(HMC5883L_Address, 0x07);
  byte zLSB =(byte) readRegister(HMC5883L_Address, 0x08);
  Cmp_z = ((zMSB << 8) | zLSB);
}

/*
* Setup accelorometer for first time
*/
void setup_ADXL345()
{
  ADXL345_status = ADXL345_OK;
  error_code = ADXL345_NO_ERROR;
	
  gains[0] = 0.00376390;
  gains[1] = 0.00376009;
  gains[2] = 0.00349265;
  
  ADXL345_powerOn();
  
  //set activity/ inactivity thresholds (0-255)
  ADXL345_setActivityThreshold(75);    //62.5mg per increment
  ADXL345_setInactivityThreshold(75);  //62.5mg per increment
  ADXL345_setTimeInactivity(10);       //How many seconds of no activity is inactive?
 
  //look of activity movement on this axes - 1 == on; 0 == off 
  ADXL345_setActivityX(1);
  ADXL345_setActivityY(1);
  ADXL345_setActivityZ(1);
 
  //look of inactivity movement on this axes - 1 == on; 0 == off
  ADXL345_setInactivityX(1);
  ADXL345_setInactivityY(1);
  ADXL345_setInactivityZ(1);
 
  //look of tap movement on this axes - 1 == on; 0 == off
  ADXL345_setTapDetectionOnX(0);
  ADXL345_setTapDetectionOnY(0);
  ADXL345_setTapDetectionOnZ(1);
 
  //set values for what is a tap, and what is a double tap (0-255)
  ADXL345_setTapThreshold(50);      //62.5mg per increment
  ADXL345_setTapDuration(15);       //625μs per increment
  ADXL345_setDoubleTapLatency(80);  //1.25ms per increment
  ADXL345_setDoubleTapWindow(200);  //1.25ms per increment
 
  //set values for what is considered freefall (0-255)
  ADXL345_setFreeFallThreshold(7);  //(5 - 9) recommended - 62.5mg per increment
  ADXL345_setFreeFallDuration(45);  //(20 - 70) recommended - 5ms per increment
 
  //setting all interupts to take place on int pin 1
  //I had issues with int pin 2, was unable to reset it
  ADXL345_setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
  ADXL345_setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
  ADXL345_setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
  ADXL345_setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
  ADXL345_setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
 
  //register interupt actions - 1 == on; 0 == off  
  ADXL345_setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1);
  ADXL345_setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
  ADXL345_setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
  ADXL345_setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
  ADXL345_setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
  
}

/*
* set power on configuration on accelorometer
*/
void ADXL345_powerOn() 
{
  //Join i2c bus (address optional for master) ,Turning on the ADXL345
  ADXL345_writeTo(ADXL345_POWER_CTL, 0);      
  ADXL345_writeTo(ADXL345_POWER_CTL, 16);
  ADXL345_writeTo(ADXL345_POWER_CTL, 8); 
}

/*
* Reads the acceleration into three variable x, y and z
*/
void ADXL3450_Read()
{
  byte xMSB =(byte) readRegister(ADXL345_Address, ADXL345_DATAX1);
  byte xLSB =(byte) readRegister(ADXL345_Address, ADXL345_DATAX0);
  Acc_x =(signed short)((xMSB << 8) | xLSB);

  byte yMSB =(byte) readRegister(ADXL345_Address, ADXL345_DATAY1);
  byte yLSB =(byte) readRegister(ADXL345_Address, ADXL345_DATAY0);
  Acc_y =(signed short)((yMSB << 8) | yLSB);

  byte zMSB =(byte) readRegister(ADXL345_Address, ADXL345_DATAZ1);
  byte zLSB =(byte) readRegister(ADXL345_Address, ADXL345_DATAZ0);
  Acc_z =(signed short)((zMSB << 8) | zLSB);
}

// Writes val to address register on device
void ADXL345_writeTo(byte address, byte val) 
{
	Wire.beginTransmission(ADXL345_Address); // start transmission to device 
	Wire.write(address);             // send register address
	Wire.write(val);                 // send value to write
	Wire.endTransmission();         // end transmission
}

// Reads num bytes starting from address register on device in to _buff array
void ADXL345_readFrom(byte address, byte num, byte _buff[]) 
{
	Wire.beginTransmission(ADXL345_Address); // start transmission to device 
	Wire.write(address);             // sends address to read from
	Wire.endTransmission();         // end transmission
	
	Wire.beginTransmission(ADXL345_Address);   // start transmission to device
	Wire.requestFrom(ADXL345_Address, num);    // request 6 bytes from device
	
	byte _i = 0;
	while(Wire.available())              // device may send less than requested (abnormal)
	{ 
		_buff[_i] = Wire.read();    // receive a byte
		_i++;
	}
	if(_i != num)
        {
		ADXL345_status = ADXL345_ERROR;
		error_code = ADXL345_READ_ERROR;
	}
	Wire.endTransmission();         // end transmission
}

// Sets the range setting, possible values are: 2, 4, 8, 16
void ADXL345_setRangeSetting(int val) 
{
	byte _s;
	byte _b;
	
	switch (val) 
        {
		case 2:  
			_s = B00000000; 
			break;
		case 4:  
			_s = B00000001; 
			break;
		case 8:  
			_s = B00000010; 
			break;
		case 16: 
			_s = B00000011; 
			break;
		default: 
			_s = B00000000;
	}
	ADXL345_readFrom(ADXL345_DATA_FORMAT, 1, &_b);
	_s |= (_b & B11101100);
	ADXL345_writeTo(ADXL345_DATA_FORMAT, _s);
}

// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void ADXL345_setInterruptLevelBit(bool interruptLevelBit) 
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}

// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void ADXL345_setFullResBit(bool fullResBit) 
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}


// Sets the JUSTIFY bit
// if sets to 1 selects the left justified mode
// if sets to 0 selects right justified mode with sign extension
void ADXL345_setJustifyBit(bool justifyBit) 
{
	ADXL345_setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}

// Sets the THRESH_TAP byte value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior
void ADXL345_setTapThreshold(int tapThreshold) 
{
	tapThreshold = constrain(tapThreshold,0,255);
	byte _b = byte (tapThreshold);
	ADXL345_writeTo(ADXL345_THRESH_TAP, _b);  
}

// set/get the gain for each axis in Gs / count
void ADXL345_setAxisGains(double *_gains)
{
	int i;
	for(i = 0; i < 3; i++)
        {
		gains[i] = _gains[i];
	}
}

// Sets the OFSX, OFSY and OFSZ bytes
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between 
void ADXL345_setAxisOffset(int x, int y, int z) 
{
	ADXL345_writeTo(ADXL345_OFSX, byte (x));  
	ADXL345_writeTo(ADXL345_OFSY, byte (y));  
	ADXL345_writeTo(ADXL345_OFSZ, byte (z));  
}

// Gets the OFSX, OFSY and OFSZ bytes
void ADXL345_getAxisOffset(int* x, int* y, int*z) 
{
	byte _b;
	ADXL345_readFrom(ADXL345_OFSX, 1, &_b);  
	*x = int (_b);
	ADXL345_readFrom(ADXL345_OFSY, 1, &_b);  
	*y = int (_b);
	ADXL345_readFrom(ADXL345_OFSZ, 1, &_b);  
	*z = int (_b);
}

// Sets the DUR byte
// The DUR byte contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625µs/LSB
// A value of 0 disables the tap/double tap funcitons. Max value is 255.
void ADXL345_setTapDuration(int tapDuration) 
{
	tapDuration = constrain(tapDuration,0,255);
	byte _b = byte (tapDuration);
	ADXL345_writeTo(ADXL345_DUR, _b);  
}

// Gets the DUR byte
int ADXL345_getTapDuration() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_DUR, 1, &_b);  
	return int (_b);
}

// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the double tap function.
// It accepts a maximum value of 255.
void ADXL345_setDoubleTapLatency(int doubleTapLatency) 
{
	byte _b = byte (doubleTapLatency);
	ADXL345_writeTo(ADXL345_LATENT, _b);  
}

// Gets the Latent value
int ADXL345_getDoubleTapLatency() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_LATENT, 1, &_b);  
	return int (_b);
}

// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the double tap function. The maximum value is 255.
void ADXL345_setDoubleTapWindow(int doubleTapWindow) 
{
	doubleTapWindow = constrain(doubleTapWindow,0,255);
	byte _b = byte (doubleTapWindow);
	ADXL345_writeTo(ADXL345_WINDOW, _b);  
}

// Gets the Window register
int ADXL345_getDoubleTapWindow() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_WINDOW, 1, &_b);  
	return int (_b);
}

// Sets the THRESH_ACT byte which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared 
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the 
// activity interrupt is enabled. The maximum value is 255.
void ADXL345_setActivityThreshold(int activityThreshold) 
{
	activityThreshold = constrain(activityThreshold,0,255);
	byte _b = byte (activityThreshold);
	ADXL345_writeTo(ADXL345_THRESH_ACT, _b);  
}

// Gets the THRESH_ACT byte
int ADXL345_getActivityThreshold() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_THRESH_ACT, 1, &_b);  
	return int (_b);
}

// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared 
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the 
// inactivity interrupt is enabled. The maximum value is 255.
void ADXL345_setInactivityThreshold(int inactivityThreshold) 
{
	inactivityThreshold = constrain(inactivityThreshold,0,255);
	byte _b = byte (inactivityThreshold);
	ADXL345_writeTo(ADXL345_THRESH_INACT, _b);  
}

// Gets the THRESH_INACT byte
int ADXL345_getInactivityThreshold() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_THRESH_INACT, 1, &_b);  
	return int (_b);
}

// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void ADXL345_setTimeInactivity(int timeInactivity) 
{
	timeInactivity = constrain(timeInactivity,0,255);
	byte _b = byte (timeInactivity);
	ADXL345_writeTo(ADXL345_TIME_INACT, _b);  
}

// Gets the TIME_INACT register
int ADXL345_getTimeInactivity() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_TIME_INACT, 1, &_b);  
	return int (_b);
}

// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The 
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void ADXL345_setFreeFallThreshold(int freeFallThreshold) 
{
	freeFallThreshold = constrain(freeFallThreshold,0,255);
	byte _b = byte (freeFallThreshold);
	ADXL345_writeTo(ADXL345_THRESH_FF, _b);  
}

// Gets the THRESH_FF register.
int ADXL345_getFreeFallThreshold() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_THRESH_FF, 1, &_b);  
	return int (_b);
}

// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall 
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void ADXL345_setFreeFallDuration(int freeFallDuration) 
{
	freeFallDuration = constrain(freeFallDuration,0,255);  
	byte _b = byte (freeFallDuration);
	ADXL345_writeTo(ADXL345_TIME_FF, _b);  
}

// Gets the TIME_FF register.
int ADXL345_getFreeFallDuration() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_TIME_FF, 1, &_b);  
	return int (_b);
}
bool ADXL345_isActivityXEnabled() 
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 6); 
}
bool ADXL345_isActivityYEnabled() 
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 5); 
}
bool ADXL345_isActivityZEnabled() 
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 4); 
}
bool ADXL345_isInactivityXEnabled() 
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 2); 
}
bool ADXL345_isInactivityYEnabled() 
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 1); 
}
bool ADXL345_isInactivityZEnabled() 
{  
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 0); 
}
void ADXL345_setActivityX(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state); 
}
void ADXL345_setActivityY(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state); 
}
void ADXL345_setActivityZ(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state); 
}
void ADXL345_setInactivityX(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state); 
}
void ADXL345_setInactivityY(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state); 
}
void ADXL345_setInactivityZ(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state); 
}
bool ADXL345_isActivityAc() 
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 7); 
}
bool ADXL345_isInactivityAc()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_INACT_CTL, 3); 
}
void ADXL345_setActivityAc(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state); 
}
void ADXL345_setInactivityAc(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state); 
}
bool ADXL345_getSuppressBit()
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 3); 
}
void ADXL345_setSuppressBit(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 3, state); 
}
bool ADXL345_isTapDetectionOnX()
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 2); 
}
void ADXL345_setTapDetectionOnX(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 2, state); 
}
bool ADXL345_isTapDetectionOnY()
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 1); 
}
void ADXL345_setTapDetectionOnY(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 1, state); 
}
bool ADXL345_isTapDetectionOnZ()
{ 
	return ADXL345_getRegisterBit(ADXL345_TAP_AXES, 0); 
}
void ADXL345_setTapDetectionOnZ(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_TAP_AXES, 0, state); 
}

bool ADXL345_isActivitySourceOnX()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 6); 
}
bool ADXL345_isActivitySourceOnY()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 5); 
}
bool ADXL345_isActivitySourceOnZ()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 4); 
}

bool ADXL345_isTapSourceOnX()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 2); 
}
bool ADXL345_isTapSourceOnY()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 1); 
}
bool ADXL345_isTapSourceOnZ()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 0); 
}

bool ADXL345_isAsleep()
{ 
	return ADXL345_getRegisterBit(ADXL345_ACT_TAP_STATUS, 3); 
}

bool ADXL345_isLowPower()
{ 
	return ADXL345_getRegisterBit(ADXL345_BW_RATE, 4); 
}
void ADXL345_setLowPower(bool state) 
{  
	ADXL345_setRegisterBit(ADXL345_BW_RATE, 4, state); 
}

double ADXL345_getRate()
{
	byte _b;
	ADXL345_readFrom(ADXL345_BW_RATE, 1, &_b);
	_b &= B00001111;
	return (pow(2,((int) _b)-6)) * 6.25;
}

void ADXL345_setRate(double rate)
{
	byte _b,_s;
	int v = (int) (rate / 6.25);
	int r = 0;
	while (v >>= 1)
	{
		r++;
	}
	if (r <= 9) 
        { 
		ADXL345_readFrom(ADXL345_BW_RATE, 1, &_b);
		_s = (byte) (r + 6) | (_b & B11110000);
		ADXL345_writeTo(ADXL345_BW_RATE, _s);
	}
}

void ADXL345_set_bw(byte bw_code)
{
	if((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600))
        {
		ADXL345_status = false;
		error_code = ADXL345_BAD_ARG;
	}
	else
        {
		ADXL345_writeTo(ADXL345_BW_RATE, bw_code);
	}
}

byte ADXL345_get_bw_code()
{
	byte bw_code;
	ADXL345_readFrom(ADXL345_BW_RATE, 1, &bw_code);
	return bw_code;
}

//Used to check if action was triggered in interrupts
//Example triggered(interrupts, ADXL345_SINGLE_TAP);
bool ADXL345_triggered(byte interrupts, int mask)
{
	return ((interrupts >> mask) & 1);
}

/*
 ADXL345_DATA_READY
 ADXL345_SINGLE_TAP
 ADXL345_DOUBLE_TAP
 ADXL345_ACTIVITY
 ADXL345_INACTIVITY
 ADXL345_FREE_FALL
 ADXL345_WATERMARK
 ADXL345_OVERRUNY
 */
byte ADXL345_getInterruptSource() 
{
	byte _b;
	ADXL345_readFrom(ADXL345_INT_SOURCE, 1, &_b);
	return _b;
}

bool ADXL345_getInterruptSource(byte interruptBit) 
{
	return ADXL345_getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}

bool ADXL345_getInterruptMapping(byte interruptBit) 
{
	return ADXL345_getRegisterBit(ADXL345_INT_MAP,interruptBit);
}

// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void ADXL345_setInterruptMapping(byte interruptBit, bool interruptPin) 
{
	ADXL345_setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}

bool ADXL345_isInterruptEnabled(byte interruptBit) 
{
	return ADXL345_getRegisterBit(ADXL345_INT_ENABLE,interruptBit);
}

void ADXL345_setInterrupt(byte interruptBit, bool state) 
{
	ADXL345_setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}

void ADXL345_setRegisterBit(byte regAdress, int bitPos, bool state) 
{
	byte _b;
	ADXL345_readFrom(regAdress, 1, &_b);
	if (state) {
		_b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
	} 
	else 
        {
		_b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
	}
	ADXL345_writeTo(regAdress, _b);  
}

bool ADXL345_getRegisterBit(byte regAdress, int bitPos) 
{
	byte _b;
	ADXL345_readFrom(regAdress, 1, &_b);
	return ((_b >> bitPos) & 1);
}

//Read gyroscope values
void L3G4200D_Read()
{  
  byte xMSB =(byte) readRegister(L3G4200D_Address, 0x29);
  byte xLSB =(byte) readRegister(L3G4200D_Address, 0x28);
  Gyr_x =(signed short)((xMSB << 8) | xLSB);

  byte yMSB =(byte) readRegister(L3G4200D_Address, 0x2B);
  byte yLSB =(byte) readRegister(L3G4200D_Address, 0x2A);
  Gyr_y =(signed short)((yMSB << 8) | yLSB);

  byte zMSB =(byte) readRegister(L3G4200D_Address, 0x2D);
  byte zLSB =(byte) readRegister(L3G4200D_Address, 0x2C);
  Gyr_z =(signed short)((zMSB << 8) | zLSB);
}

//setup for firs time he gyroscope sensor
void setupL3G4200D(int scale)
{
  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:
  if(scale == 250)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }
  else if(scale == 500)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }
  else //2000 dps
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

//Write register to i2c device with specific i2c
void writeRegister(byte deviceAddress, byte address, byte val) 
{
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);           // send value to write
    Wire.endTransmission();    // end transmission
}

//Read a byte from i2c device with specific address
byte readRegister(byte deviceAddress, byte address)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);                 // register to read
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, 2);  // read a byte
    while(!Wire.available()) { } //wait until ready
    return Wire.read();
}
