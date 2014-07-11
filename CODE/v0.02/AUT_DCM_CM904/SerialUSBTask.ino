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

//Serial Task runs under RTOS
void vSerialUSBTask( void *pvParameters )
{ 
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount ();
  for( ;; )
  {      
    toggleLED();
    //vTaskSuspendAll();   //Suspend all other tasks
    //Send_Info();  
    //Send_Motor_Info(nn-49);
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

//-------------------
void Send_Motor_Info(byte i)
{
    SerialUSB.print("P[");SerialUSB.print(i+1);SerialUSB.print("]=");SerialUSB.print(PRESENT_POSITION[i]);SerialUSB.print("\t\t");
    SerialUSB.print("S[");SerialUSB.print(i+1);SerialUSB.print("]=");SerialUSB.print(PRESENT_SPEED[i]);SerialUSB.print("\t\t");
    SerialUSB.print("L[");SerialUSB.print(i+1);SerialUSB.print("]=");SerialUSB.print(PRESENT_LOAD[i]);SerialUSB.print("\t\t");
    SerialUSB.print("V[");SerialUSB.print(i+1);SerialUSB.print("]=");SerialUSB.print(PRESENT_VOLTAGE[i]);SerialUSB.print("\t\t");
    SerialUSB.print("T[");SerialUSB.print(i+1);SerialUSB.print("]=");SerialUSB.print(PRESENT_TEMPERATURE[i]);SerialUSB.print("\t\t");
    SerialUSB.print("C[");SerialUSB.print(i+1);SerialUSB.print("]=");SerialUSB.println(PRESENT_CURRENT[i]);//SerialUSB.print("\t");
}

//Send information under string format
void Send_Info()
{ 
    SerialUSB.print("AUTMan:\_> ");
    
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
    
    SerialUSB.print("USB_Hz=");SerialUSB.print(SerialUSBTaskHz);SerialUSB.print("\t");
    SerialUSB.print("IMU_Hz=");SerialUSB.print(IMUTaskHz);SerialUSB.print("\t");
    SerialUSB.print("CNT_Hz=");SerialUSB.print(CNTTaskHz);SerialUSB.print("\t");
    SerialUSB.print("DXLW_Hz=");SerialUSB.print(DXLWTaskHz);SerialUSB.print("\t");
    SerialUSB.print("DXLR_Hz=");SerialUSB.print(DXLRTaskHz);SerialUSB.print("\t");

    SerialUSB.print("RTC=");   SerialUSB.println(RTC);
}
