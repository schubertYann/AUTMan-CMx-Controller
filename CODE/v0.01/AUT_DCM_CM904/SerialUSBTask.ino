//=====================================================================================================
// HEXAPOD ROBOT
// OpenCM9.04 + GY-80 
//=====================================================================================================
//
// Mojtaba's implementation of AUTMan IMU algorithm.
// See: http://autman.aut.ac.ir
//      http://mojtaba.info.se/
//
// Date			Author			        Notes
// 01/01/2014		Mojtaba.K	        	Initial release
// 01/03/2014		Mojtaba.K, Alireza.A		Optimised for reduced CPU load
//
//=====================================================================================================
// The SerialUSB task for send information to PC
//=====================================================================================================

#define _LOBYTE(w)           ((unsigned char)(((unsigned long)(w)) & 0xff))
#define _HIBYTE(w)           ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff))

signed short Check_Sum=0;
byte TX_Buff[32];


//Serial Task runs under RTOS
void vSerialUSBTask( void *pvParameters )
{ 
  portTickType xLastWakeTime;
  const portTickType xFrequency = 100;
  
  xLastWakeTime = xTaskGetTickCount ();
  for( ;; )
  {      
    //vTaskSuspendAll();   //Suspend all other tasks
    Send_Info();  
    //Send_Info_packet(); 
    
    //if(Dxl.available())  
    //{    
    //  SerialUSB.print((char)Dxl.readRaw());
    //  toggleLED();  //toggle digital value based on current value.
    //}
    
    SerialUSBTaskCNT++;
    //xTaskResumeAll();   //Resume all other tasks
    //vTaskDelay(0.1);    //change it for send information frequency
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

//Send information under string format
void Send_Info()
{
    toggleLED();
    SerialUSB.println("Root:\_>");
    
    SerialUSB.print("Gx=");SerialUSB.print(Gyr_x);SerialUSB.print("\t");
    SerialUSB.print("Gy=");SerialUSB.print(Gyr_y);SerialUSB.print("\t");
    SerialUSB.print("Gz=");SerialUSB.print(Gyr_z);SerialUSB.print("\t");
    
    SerialUSB.print("Ax=");SerialUSB.print(Acc_x);SerialUSB.print("\t");
    SerialUSB.print("Ay=");SerialUSB.print(Acc_y);SerialUSB.print("\t");
    SerialUSB.print("Az=");SerialUSB.print(Acc_z);SerialUSB.print("\t");
  
    //SerialUSB.print("Cx=");SerialUSB.print(Cmp_x);SerialUSB.print("\t");
    //SerialUSB.print("Cy=");SerialUSB.print(Cmp_y);SerialUSB.print("\t");
    //SerialUSB.print("Cz=");SerialUSB.print(Cmp_z);SerialUSB.print(" ");
    
    
    //SerialUSB.print("Roll="); SerialUSB.print(roll); SerialUSB.print("\t\t");
    //SerialUSB.print("Pitch=");SerialUSB.print(pitch);SerialUSB.print("\t\t");
    //SerialUSB.print("Yaw=");  SerialUSB.print(yaw);  SerialUSB.print("\t\t");
   
    SerialUSB.print("p=");  SerialUSB.print(PRESENT_POSITION[0]); SerialUSB.print("\t"); 
    //SerialUSB.print("S=");  SerialUSB.println(PRESENT_SPEED[0]);//  SerialUSB.print("\t");
    //SerialUSB.print("p=");  SerialUSB.println(PRESENT_POSITION[0]);//  SerialUSB.print("\t");
    
    //SerialUSB.print("USB_Hz=");SerialUSB.print(SerialUSBTaskHz);SerialUSB.print("\t");
    SerialUSB.print("IMU_Hz=");SerialUSB.print(IMUTaskHz);SerialUSB.print("\t");
    
    SerialUSB.print("DXLW_Hz=");SerialUSB.print(DXLWTaskHz);SerialUSB.print("\t");
    SerialUSB.print("DXLR_Hz=");SerialUSB.print(DXLRTaskHz);SerialUSB.print("\t");

    SerialUSB.print("RTC=");   SerialUSB.println(RTC);
}

void Send_Info_packet()
{
    byte i=0;
    fill_buffer_data(); //fill data to send buffer
    digitalWrite(BOARD_LED_PIN, HIGH);
    
    for(i=0;i<=31;i++)
      SerialUSB.write(TX_Buff[i]);
  
    digitalWrite(BOARD_LED_PIN, LOW);
}

void fill_buffer_data()
{  
    byte i=0;  
    //start of packet
    TX_Buff[0]=255; TX_Buff[1]=255;
    
    //gyroscope
    TX_Buff[2]=_LOBYTE(Gyr_x);
    TX_Buff[3]=_HIBYTE(Gyr_x); 
    
    TX_Buff[4]=_LOBYTE(Gyr_y);
    TX_Buff[5]=_HIBYTE(Gyr_y);
    
    TX_Buff[6]=_LOBYTE(Gyr_z);
    TX_Buff[7]=_HIBYTE(Gyr_z);
  
    //accelorometer
    TX_Buff[8]=_LOBYTE(Acc_x);
    TX_Buff[9]=_HIBYTE(Acc_x); 
    
    TX_Buff[10]=_LOBYTE(Acc_y);
    TX_Buff[11]=_HIBYTE(Acc_y);
    
    TX_Buff[12]=_LOBYTE(Acc_z);
    TX_Buff[13]=_HIBYTE(Acc_z);
  
    //Compass
    TX_Buff[14]=_LOBYTE(Cmp_x);
    TX_Buff[15]=_HIBYTE(Cmp_x);
    
    TX_Buff[16]=_LOBYTE(Cmp_y);
    TX_Buff[17]=_HIBYTE(Cmp_y); 
    
    TX_Buff[18]=_LOBYTE(Cmp_z);
    TX_Buff[19]=_HIBYTE(Cmp_z);    
   
    //rool
    TX_Buff[20]=_LOBYTE((signed short)(roll*100));
    TX_Buff[21]=_HIBYTE((signed short)(roll*100));
    //pitch
    TX_Buff[22]=_LOBYTE((signed short)(pitch*100));
    TX_Buff[23]=_HIBYTE((signed short)(pitch*100));
    //yaw
    TX_Buff[24]=_LOBYTE((signed short)(yaw*100));
    TX_Buff[25]=_HIBYTE((signed short)(yaw*100));    
          
    //IMUReadTaskHz
    TX_Buff[26]=_LOBYTE(IMUTaskHz);
    TX_Buff[27]=_HIBYTE(IMUTaskHz);
  
    //system RTC counter 
    TX_Buff[28]=_LOBYTE(RTC);
    TX_Buff[29]=_HIBYTE(RTC);
 
    //calculate checkSum
    Check_Sum=0; 
    for(i=2;i<=29;i++)
    {
      Check_Sum = Check_Sum ^ TX_Buff[i];
    }
    //check sum
    TX_Buff[30]=_LOBYTE(Check_Sum);
    TX_Buff[31]=_HIBYTE(Check_Sum);
}

