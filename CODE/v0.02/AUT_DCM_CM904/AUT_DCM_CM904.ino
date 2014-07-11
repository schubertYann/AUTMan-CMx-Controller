//=====================================================================================================
// HEXAPOD ROBOT
// OpenCM9.04 + GY-80 
//=====================================================================================================
//
// Mojtaba's implementation of HEXAPOD CONTROLLER system.
// See: http://autman.aut.ac.ir
//      http://mojtaba.info.se/
//
// Date			Author			        Notes
// 01/01/2014		Mojtaba.K	        	Initial release
// 01/03/2014		Mojtaba.K, Alireza.A		Optimised for reduced CPU load
//
//=====================================================================================================
// The Main Code
//=====================================================================================================

//Include Lib 
#include <Wire.h>            // I2C communication lib
#include <MapleFreeRTOS.h>   // MapleFree Real Time Operating System lib 
Dynamixel Dxl(1);            // Dynamixel on Serial1(USART1)
HardwareTimer Timer(1);      // Hardware timer for RTC

//Global Deffenition 
#define DEG2RAD (0.0174532925199432957)
#define RAD2DEG (57.295779513082320876)

#define _LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define _HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

#define BLED (12)  //blue led on board
#define WLED (10)  //white led on board

//Global Varibles
signed short Gyr_x=0 , Gyr_y=0 , Gyr_z=0;                //Gyroscope sensor data
signed short Acc_x=0 , Acc_y=0 , Acc_z=0;                //Accelorometer sensor dada
signed short Cmp_x=0 , Cmp_y=0 , Cmp_z=0;                //Compasss sensor data

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	 //Quaternion of sensor frame relative to auxiliary frame
float roll=0.0f, pitch=0.0f, yaw=0.0f;                   //Estimated Orientation

signed short IMUTaskCNT=0 , IMUTaskHz=0;                 //IMU task internal frequency value
signed short SerialUSBTaskCNT=0 , SerialUSBTaskHz=0;     //Serial task internal frequency value
signed short DXLWTaskCNT=0 , DXLWTaskHz=0;               //IMU task internal frequency value
signed short DXLRTaskCNT=0 , DXLRTaskHz=0;               //IMU task internal frequency value

signed short CNTTaskCNT=0 , CNTTaskHz=0;               //IMU task internal frequency value

long RTC=0;                                              //Real Time Clock value

//Writing walue to motors
int  Speed[20];
int  Position[20];

//Reading value from dynamixels are saved in global arrays
int PRESENT_POSITION[20];   
int PRESENT_SPEED[20];
int PRESENT_LOAD[20];
int PRESENT_VOLTAGE[20];
int PRESENT_TEMPERATURE[20];
int PRESENT_CURRENT[20];

byte nn=0;

byte DXL_R_Flag=0;

//Setup the main confiquration
void setup()
{
  Dxl.begin(0);        delay(500);               // Config onboard LED 
  SerialUSB.begin();   delay(500);               // Config serialUSB port
  SerialUSB.attachInterrupt(usbInterrupt);       // Atach interrupt to serial usb (on recive byte)
  Wire.begin(1,0);     delay(500);               // Initialize i2c comunication port (sda and scl)
  //Serial2.begin(4800); delay(500);
  
  pinMode(BOARD_LED_PIN, OUTPUT);                //Config onboard LED
  pinMode(BLED, OUTPUT);                         //Config onboard LED
  pinMode(WLED, OUTPUT);                         //Config onboard LED
  
  setupL3G4200D(2000);                           // Configure L3G4200  - 250, 500 or 2000 deg/sec  
  setup_ADXL345();                               // Configure ADXL345 for first time 
  setup_HMC5883L();                              // Configure HMC5883 magnometer sensor
  
  Timer.pause();                                 // Pause the timer while we're configuring it
  Timer.setPeriod(1000000);                      // Set up period in microseconds
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);// Set up an interrupt on channel 1
  Timer.setCompare(TIMER_CH1, 1);                // Interrupt 1 count after each update
  Timer.attachInterrupt(TIMER_CH1, RTC_INT);     // atach interrupt function to timer
  Timer.refresh();                               // Refresh the timer's count, prescale, and overflow
  Timer.resume();                                // Start the timer counting
  
  //initilize cuncurent tasks, the periority of tasks are equal in this configuration and the running Hz are configure using TaskDelay function
  xTaskCreate( vIMUTask       ,( signed char * ) "IMU"  ,configMINIMAL_STACK_SIZE, NULL, 1, NULL );      //Init IMU task for read and run fusion alghorithm
  //xTaskCreate( vSerialUSBTask ,( signed char * ) "SUSB" ,configMINIMAL_STACK_SIZE, NULL, 2, NULL );      //Init serial usb task for send information (String format) to PC 
  //xTaskCreate( vDXLWTask      ,( signed char * ) "DXLW"  ,configMINIMAL_STACK_SIZE, NULL, 3, NULL );      //Init Real Time Clock task for calculate internal frequency of tasks
  xTaskCreate( vDXLRTask      ,( signed char * ) "DXLR"  ,configMINIMAL_STACK_SIZE, NULL, 2, NULL );      //Init Real Time Clock task for calculate internal frequency of tasks
  
  xTaskCreate( vDXLRRAWTask      ,( signed char * ) "rrr"  ,configMINIMAL_STACK_SIZE, NULL, 2, NULL );      //Init Real Time Clock task for calculate internal frequency of tasks
  
  //xTaskCreate( vCNTROLTask      ,( signed char * ) "CONTROL"  ,configMINIMAL_STACK_SIZE, NULL, 2, NULL );      //Init Real Time Clock task for calculate internal frequency of tasks
  
  vTaskStartScheduler();      //Start schaduler of task, this function from RTOS lib and defult system if pipe line system
}

byte Chk_Sum=0;
byte TX_Buff[64];
//SerialUSB Interrupt type must have the below proto-type
//void interrupt_name (byte* buffer, byte nCount){}
//USB max packet data is maximum 64byte, so nCount can not exceeds 64 bytes
void usbInterrupt(byte* buffer, byte nCount)
{
  int i=0;
  int index=0;
  
  if(buffer[index]==255)
  {
    if(buffer[index+1]==255) 
    { 
      switch (buffer[index+2])   //check for id
      {
        case 100 : 
                
                break;
        case 254 : 
                switch (buffer[index+4]) //check for instraction
                {
                  case INST_PING:
                                break;
                  case INST_READ:                  
                                TX_Buff[0]=255; 
                                TX_Buff[1]=255;
                                
                                TX_Buff[2]=254;
                                TX_Buff[3]=(6*2*12)+2;
                                TX_Buff[3]=0;
                                
                                for(i=0;i<=11;i++)
                                {
                                  TX_Buff[(i*2)+((0*24)+4)]=_LOBYTE(PRESENT_POSITION[i]);
                                  TX_Buff[(i*2)+((0*24)+4)]=_HIBYTE(PRESENT_POSITION[i]); 
             
                                  TX_Buff[(i*2)+((1*24)+4)]=_LOBYTE(PRESENT_SPEED[i]);
                                  TX_Buff[(i*2)+((1*24)+5)]=_HIBYTE(PRESENT_SPEED[i]);
                                
                                  TX_Buff[(i*2)+((2*24)+4)]=_LOBYTE(PRESENT_LOAD[i]);
                                  TX_Buff[(i*2)+((2*24)+5)]=_HIBYTE(PRESENT_LOAD[i]);
                                  
                                  TX_Buff[(i*2)+((3*24)+4)]=_LOBYTE(PRESENT_VOLTAGE[i]);
                                  TX_Buff[(i*2)+((3*24)+5)]=_HIBYTE(PRESENT_VOLTAGE[i]);
                                  
                                  TX_Buff[(i*2)+((4*24)+4)]=_LOBYTE(PRESENT_TEMPERATURE[i]);
                                  TX_Buff[(i*2)+((4*24)+5)]=_HIBYTE(PRESENT_TEMPERATURE[i]);
                                  
                                  TX_Buff[(i*2)+((5*24)+4)]=_LOBYTE(PRESENT_CURRENT[i]);
                                  TX_Buff[(i*2)+((5*24)+5)]=_HIBYTE(PRESENT_CURRENT[i]);                                 
                                }                                
                                for(i=2;i<=(6*2*12);i++) TX_Buff[(6*2*12)+1]+=TX_Buff[i];
                                TX_Buff[(6*2*12)+1]!=TX_Buff[(6*2*12)+1];                              
                                
                                for(i=0;i<=(6*2*12)+2+3;i++)
                                  SerialUSB.write(TX_Buff[i]);
                                 
                                break;                               
                  case INST_WRITE:
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                  
                                break;   
                  case INST_SYNC_WRITE:
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                  
                                break; 
                  default:      
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                  
                                break;          
                } 
                break;
        default: 
                switch (buffer[index+4]) //check for instraction
                {
                  case INST_PING:
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                DXL_R_Flag=1;
                                  
                                break;
                  case INST_READ:
                                
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                 DXL_R_Flag=1;
                                  /*
                                  
                                while(!Dxl.available()) togglePin(BLED);
                                 
                                while(Dxl.available())
                                {
                                  SerialUSB.write(Dxl.readRaw());
                                  togglePin(WLED);
                                }
                                 */   
                                break;
                  case INST_WRITE:
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                  
                                break;   
                  case INST_SYNC_WRITE:
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                  
                                break; 
                  default:      
                                for(unsigned int i=index; i < nCount;i++)
                                  Dxl.writeRaw(buffer[i]);
                                  
                                break;          
                }              
                break;
      }
    }
  }
  index++;
  
      //Dxl.writeRaw(buffer[i]);
      //SerialUSB.print((char)buffer[i]);
  //Send_Info(); 
  //nn=buffer[0];
  //ik(1,100,50,0,50,0);
}


// Hardware interrupt for real time clock
void RTC_INT(void) 
{
    togglePin(12);  
    RTC++;
    
    SerialUSBTaskHz=SerialUSBTaskCNT;
    SerialUSBTaskCNT=0;
    
    IMUTaskHz = IMUTaskCNT;
    IMUTaskCNT=0;
    
    DXLWTaskHz = DXLWTaskCNT;
    DXLWTaskCNT=0;
    
    DXLRTaskHz = DXLRTaskCNT;
    DXLRTaskCNT=0;
    
    CNTTaskHz = CNTTaskCNT;
    CNTTaskCNT=0;
}

// Main Loop
void loop()
{
  //RTOS task is running, there is no code for main loop execute
}





















