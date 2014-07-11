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
//=====================================================================================================
// The System Debug task for send information to PC
//=====================================================================================================

//Serial Task runs under RTOS
void vSDBGTask( void *pvParameters )
{ 
  int i=0;
  vTaskDelay(1000);
  portTickType xLastWakeTime;
  const portTickType xFrequency = 500;
  xLastWakeTime = xTaskGetTickCount ();
  //-----------------------------------
  for( ;; )
  {  
    SDBGTaskCNT++; 
    
    //show system thr(s) hz
    Send_THR_Hz();  
    
    //show founded motors id
    Serial2.print(" DXLs >>> ");
    for(i=0;i<=NUM_OF_DXL-1;i++)
    {
        if(DXL_Exist[i]==1) 
        {
          Serial2.print(i+1); Serial2.print(" \t");          
        }
    }     
    Serial2.println("> OK");
       
    //show each motors data
    for(i=0;i<=NUM_OF_DXL-1;i++)
    {
        if(DXL_Exist[i]==1) 
        {
          Send_Motor_Info(i);
        }
    }   
   
      
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

//-------------------
void Send_Motor_Info(byte i)
{
    Serial2.print("P[");Serial2.print(i+1);Serial2.print("]=");Serial2.print(PRESENT_POSITION[i]);Serial2.print(" \t");
    Serial2.print("S[");Serial2.print(i+1);Serial2.print("]=");Serial2.print(PRESENT_SPEED[i]);Serial2.print(" \t");
    Serial2.print("L[");Serial2.print(i+1);Serial2.print("]=");Serial2.print(PRESENT_LOAD[i]);Serial2.print(" \t");
    Serial2.print("V[");Serial2.print(i+1);Serial2.print("]=");Serial2.print(PRESENT_VOLTAGE[i]);Serial2.print(" \t");
    Serial2.print("T[");Serial2.print(i+1);Serial2.print("]=");Serial2.print(PRESENT_TEMPERATURE[i]);Serial2.print(" \t");
    Serial2.print("C[");Serial2.print(i+1);Serial2.print("]=");Serial2.println(PRESENT_CURRENT[i]);//SerialUSB.print("\t");
}

//Send information under string format
void Send_THR_Hz()
{ 
    Serial2.print("AUTMan:\_> ");
    
    //SerialUSB.print("Gx=");SerialUSB.print(Gyr_x);SerialUSB.print("\t");
    //SerialUSB.print("Gy=");SerialUSB.print(Gyr_y);SerialUSB.print("\t");
    //SerialUSB.print("Gz=");SerialUSB.print(Gyr_z);SerialUSB.print("\t");
    
    //SerialUSB.print("Ax=");SerialUSB.print(Acc_x);SerialUSB.print("\t");
    //SerialUSB.print("Ay=");SerialUSB.print(Acc_y);SerialUSB.print("\t");
    //SerialUSB.print("Az=");SerialUSB.print(Acc_z);SerialUSB.print("\t");
  
    //SerialUSB.print("Cx=");SerialUSB.print(Cmp_x);SerialUSB.print("\t");
    //SerialUSB.print("Cy=");SerialUSB.print(Cmp_y);SerialUSB.print("\t");
    //SerialUSB.print("Cz=");SerialUSB.print(Cmp_z);SerialUSB.print(" ");
    
    //SerialUSB.print("Roll="); SerialUSB.print(roll); SerialUSB.print("\t\t");
    //SerialUSB.print("Pitch=");SerialUSB.print(pitch);SerialUSB.print("\t\t");
    //SerialUSB.print("Yaw=");  SerialUSB.print(yaw);  SerialUSB.print("\t\t");
    
    Serial2.print("SUSB_Hz=");Serial2.print(SUSBTaskHz);Serial2.print(" \t");
    Serial2.print("SIMU_Hz=");Serial2.print(SIMUTaskHz);Serial2.print(" \t");
    Serial2.print("DXLW_Hz=");Serial2.print(DXLWTaskHz);Serial2.print(" \t");
    Serial2.print("DXLR_Hz=");Serial2.print(DXLRTaskHz);Serial2.print(" \t");
    Serial2.print("SDBG_Hz=");Serial2.print(SDBGTaskHz);Serial2.print(" \t");
    Serial2.print("RTC=");   Serial2.print(RTC); Serial2.print(" ");
}
