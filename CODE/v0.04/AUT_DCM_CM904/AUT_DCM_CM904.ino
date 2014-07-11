//=====================================================================================================
// AUTMan ROBOT
// OpenCM9.04 + GY-80 
//=====================================================================================================
//
// Mojtaba's implementation of HEXAPOD CONTROLLER system.
// See: http://autman.aut.ac.ir
//      http://mojtaba.info.se/
// Date			Author			        Notes
// 01/01/2014		Mojtaba.K	        	Initial release
//=====================================================================================================
// The Main Code
//=====================================================================================================
#include <Wire.h>            // I2C communication lib
#include <MapleFreeRTOS.h>   // MapleFree Real Time Operating System lib 
 
//Global Deffenition 
#define DEG2RAD (0.0174532925199432957)
#define RAD2DEG (57.295779513082320876)
#define DEG2DXL (11.377777776666666668)

#define _LOBYTE(w)   ((unsigned char)(((unsigned long)(w)) & 0xff))
#define _HIBYTE(w)   ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

#define BLED (12)  //blue led on board
#define WLED (10)  //white led on board

#define NUM_OF_DXL (20)  //white led on board

Dynamixel DXL(1);            // Dynamixel on Serial1(USART1)
HardwareTimer Timer(1);      // Hardware timer for RTC

//Global Varibles
signed short Gyr_x=0 , Gyr_y=0 , Gyr_z=0;                //Gyroscope sensor data
signed short Acc_x=0 , Acc_y=0 , Acc_z=0;                //Accelorometer sensor dada
signed short Cmp_x=0 , Cmp_y=0 , Cmp_z=0;                //Compasss sensor data

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	 //Quaternion of sensor frame relative to auxiliary frame
float roll=0.0f, pitch=0.0f, yaw=0.0f;                   //Estimated Orientation

signed short SIMUTaskCNT=0 , SIMUTaskHz=0;                 //Sensor IMU task internal frequency value
signed short SUSBTaskCNT=0 , SUSBTaskHz=0;                 //Serial usb task internal frequency value
signed short DXLWTaskCNT=0 , DXLWTaskHz=0;                 //Dxl write task internal frequency value
signed short DXLRTaskCNT=0 , DXLRTaskHz=0;                 //Dxl read task internal frequency value
signed short MCNTTaskCNT=0 , MCNTTaskHz=0;                 //System debug task internal frequency value

long RTC=0;                                              //Real Time Clock value

double Angle[NUM_OF_DXL];

byte DXL_Exist[NUM_OF_DXL];   //the array that shows existing dynamixel with specific id on the bus
//Writing walue to motors
byte D_TORQUE_ENABLE[NUM_OF_DXL];
byte D_STATUS_LED[NUM_OF_DXL];
byte D_KD_GAIN[NUM_OF_DXL];   
byte D_KI_GAIN[NUM_OF_DXL];
byte D_KP_GAIN[NUM_OF_DXL];
int  D_GOAL_POSITION[NUM_OF_DXL];
int  D_MOVING_SPEED[NUM_OF_DXL];

//Reading value from dynamixels are saved in global arrays
int PRESENT_MODEL_NUMBER[NUM_OF_DXL];
int PRESENT_POSITION[NUM_OF_DXL];   
int PRESENT_SPEED[NUM_OF_DXL];
int PRESENT_LOAD[NUM_OF_DXL];
int PRESENT_VOLTAGE[NUM_OF_DXL];
int PRESENT_TEMPERATURE[NUM_OF_DXL];
int PRESENT_CURRENT[NUM_OF_DXL];

//Setup the main confiquration
void setup()
{
  DXL.begin(1);          delay(10);               // Config Daynamixel Bus @1000000 bps 
  SerialUSB.begin();     delay(10);               // Config serialUSB port
  SerialUSB.attachInterrupt(usbInterrupt);        // Atach interrupt to serial usb (on recive byte)
  Wire.begin(1,0);       delay(10);               // Initialize i2c comunication port (sda and scl)
  Serial2.begin(230400); delay(10);
  
  pinMode(BOARD_LED_PIN, OUTPUT);                //Config onboard LED
  //pinMode(BLED, OUTPUT);                         //Config onboard Blue  LED
  //pinMode(WLED, OUTPUT);                         //Config onboard White LED
  
  Timer.pause();                                 // Pause the timer while we're configuring it
  Timer.setPeriod(1000000);                      // Set up period in microseconds
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);// Set up an interrupt on channel 1
  Timer.setCompare(TIMER_CH1, 1);                // Interrupt 1 count after each update
  Timer.attachInterrupt(TIMER_CH1, RTC_INT);     // atach interrupt function to timer
  Timer.refresh();                               // Refresh the timer's count, prescale, and overflow
  Timer.resume();                                // Start the timer counting
  
  //initilize cuncurent tasks, the periority of tasks are equal in this configuration and the running Hz are configure using TaskDelay function
  xTaskCreate( vSIMUTask       ,( signed char * ) "SIMU" ,configMINIMAL_STACK_SIZE, NULL, 1, NULL );      //Init IMU task for read and run fusion alghorithm
  xTaskCreate( vSUSBTask       ,( signed char * ) "SUSB" ,configMINIMAL_STACK_SIZE, NULL, 1, NULL );      //Init serial usb task for send information (String format) to PC 
  xTaskCreate( vDXLWTask       ,( signed char * ) "DXLW" ,configMINIMAL_STACK_SIZE, NULL, 2, NULL );      //Init Real Time Clock task for calculate internal frequency of tasks
  xTaskCreate( vDXLRTask       ,( signed char * ) "DXLR" ,configMINIMAL_STACK_SIZE, NULL, 1, NULL );      //Init Real Time Clock task for calculate internal frequency of tasks  
  
  xTaskCreate( vMCNTTask       ,( signed char * ) "SDBG" ,configMINIMAL_STACK_SIZE, NULL, 4, NULL );      //Init Real Time Clock task for calculate internal frequency of tasks
  
  vTaskStartScheduler();      //Start schaduler of task, this function from RTOS lib and defult system if pipe line system
}



//void interrupt_name (byte* buffer, byte nCount){}
//USB max packet data is maximum 64byte, so nCount can not exceeds 64 bytes
void usbInterrupt(byte* buffer, byte nCount)
{ 
      
}


// Hardware interrupt for real time clock
void RTC_INT(void) 
{
    togglePin(BLED);  
    RTC++;
    
    SUSBTaskHz = SUSBTaskCNT;
    SUSBTaskCNT=0;
    
    SIMUTaskHz = SIMUTaskCNT;
    SIMUTaskCNT=0;
    
    DXLWTaskHz = DXLWTaskCNT;
    DXLWTaskCNT=0;
    
    DXLRTaskHz = DXLRTaskCNT;
    DXLRTaskCNT=0;
    
    MCNTTaskHz = MCNTTaskCNT;
    MCNTTaskCNT=0;
}

// Main Loop
void loop()
{
  //RTOS task is running, there is no code for main loop execute
}





















