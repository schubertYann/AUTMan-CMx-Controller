//=====================================================================================================
// HEXAPOD ROBOT
// OpenCM9.04 + GY-80 
//=====================================================================================================
//
// Mojtaba's implementation of AUTMan IMU algorithm.
// See: http://autman.aut.ac.ir
//      http://mojtaba.info.se/
// Date			Author			        Notes
// 01/01/2014		Mojtaba.K	        	Initial release
//=====================================================================================================
// The SerialUSB task for send information to PC
//=====================================================================================================
signed short Check_Sum=0;
byte TX_Buff[64];

//Serial Task runs under RTOS
void vSUSBTask( void *pvParameters )
{ 
  int i=0;
  vTaskDelay(1000);
  portTickType xLastWakeTime;
  const portTickType xFrequency = 250;
  xLastWakeTime = xTaskGetTickCount ();
  //-----------------------------------
  for( ;; )
  {    
    
    SUSBTaskCNT++;  
    toggleLED();
    //Send_THR_Hz();
    //vTaskSuspendAll();   //Suspend all other tasks
    //Send_Info_packet(); 
    //for(i=0;i<=60;i++) SerialUSB.print("0");
    //SerialUSB.println(" ");
    //xTaskResumeAll();   //Resume all other tasks
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}



//Send information under string format
void Send_THR_Hz()
{ 
    SerialUSB.print("AUTMan:\_> ");
    
    //SerialUSB.print("Gx=");SerialUSB.print(Gyr_x);SerialUSB.print("\t");
    //SerialUSB.print("Gy=");SerialUSB.print(Gyr_y);SerialUSB.print("\t");
    //SerialUSB.print("Gz=");SerialUSB.print(Gyr_z);SerialUSB.print("\t");
    
    //SerialUSB.print("Ax=");SerialUSB.print(Acc_x);SerialUSB.print("\t");
    //SerialUSB.print("Ay=");SerialUSB.print(Acc_y);SerialUSB.print("\t");
    //SerialUSB.print("Az=");SerialUSB.print(Acc_z);SerialUSB.print("\t");
  
    //SerialUSB.print("Cx=");SerialUSB.print(Cmp_x);SerialUSB.print("\t");
    //SerialUSB.print("Cy=");SerialUSB.print(Cmp_y);SerialUSB.print("\t");
    //SerialUSB.print("Cz=");SerialUSB.print(Cmp_z);SerialUSB.print(" ");
    
    SerialUSB.print("Roll="); SerialUSB.print(roll); SerialUSB.print("\t\t");
    SerialUSB.print("Pitch=");SerialUSB.print(pitch);SerialUSB.print("\t\t");
    SerialUSB.print("Yaw=");  SerialUSB.print(yaw);  SerialUSB.print("\t\t");
    
    SerialUSB.print("SUSB_Hz=");SerialUSB.print(SUSBTaskHz);SerialUSB.print(" \t");
    SerialUSB.print("SIMU_Hz=");SerialUSB.print(SIMUTaskHz);SerialUSB.print(" \t");
    SerialUSB.print("DXLW_Hz=");SerialUSB.print(DXLWTaskHz);SerialUSB.print(" \t");
    SerialUSB.print("DXLR_Hz=");SerialUSB.print(DXLRTaskHz);SerialUSB.print(" \t");
    SerialUSB.print("MCNT_Hz=");SerialUSB.print(MCNTTaskHz);SerialUSB.print(" \t");
    SerialUSB.print("RTC=");   SerialUSB.print(RTC); SerialUSB.println(" ");
}


void Send_Info_packet()
{
    int i=0;
    fill_buffer_data(); //fill data to send buffer
    
    for(i=0;i<=((19*15)+35+2);i++)
      SerialUSB.write(TX_Buff[i]);
}

void fill_buffer_data()
{  
    int i=0;  
    
    //start of packet
    TX_Buff[0]=255; 
    TX_Buff[1]=255;

    //------------------------
    TX_Buff[2]=200;  //id of IMU
    //gyroscope
    TX_Buff[3]=_LOBYTE(Gyr_x);
    TX_Buff[4]=_HIBYTE(Gyr_x);    
    TX_Buff[5]=_LOBYTE(Gyr_y);
    TX_Buff[6]=_HIBYTE(Gyr_y);   
    TX_Buff[7]=_LOBYTE(Gyr_z);
    TX_Buff[8]=_HIBYTE(Gyr_z);
  
    //accelorometer
    TX_Buff[9]=_LOBYTE(Acc_x);
    TX_Buff[10]=_HIBYTE(Acc_x);    
    TX_Buff[11]=_LOBYTE(Acc_y);
    TX_Buff[12]=_HIBYTE(Acc_y);   
    TX_Buff[13]=_LOBYTE(Acc_z);
    TX_Buff[14]=_HIBYTE(Acc_z);
  
    //Compass
    TX_Buff[15]=_LOBYTE(Cmp_x);
    TX_Buff[16]=_HIBYTE(Cmp_x);   
    TX_Buff[17]=_LOBYTE(Cmp_y);
    TX_Buff[18]=_HIBYTE(Cmp_y);     
    TX_Buff[19]=_LOBYTE(Cmp_z);
    TX_Buff[20]=_HIBYTE(Cmp_z); 
    //------------------------- 
     
    for(i=0;i<=NUM_OF_DXL-1;i++)
    {
      TX_Buff[(i*15)+21]=id[i];
      
      TX_Buff[(i*15)+22]=_LOBYTE(PRESENT_MODEL_NUMBER[i]);
      TX_Buff[(i*15)+23]=_HIBYTE(PRESENT_MODEL_NUMBER[i]);
      
      TX_Buff[(i*15)+24]=_LOBYTE(PRESENT_POSITION[i]);
      TX_Buff[(i*15)+25]=_HIBYTE(PRESENT_POSITION[i]);
      
      TX_Buff[(i*15)+26]=_LOBYTE(PRESENT_SPEED[i]);
      TX_Buff[(i*15)+27]=_HIBYTE(PRESENT_SPEED[i]);
      
      TX_Buff[(i*15)+28]=_LOBYTE(PRESENT_LOAD[i]);
      TX_Buff[(i*15)+29]=_HIBYTE(PRESENT_LOAD[i]);
      
      TX_Buff[(i*15)+30]=_LOBYTE(PRESENT_VOLTAGE[i]);
      TX_Buff[(i*15)+31]=_HIBYTE(PRESENT_VOLTAGE[i]);
      
      TX_Buff[(i*15)+32]=_LOBYTE(PRESENT_TEMPERATURE[i]);
      TX_Buff[(i*15)+33]=_HIBYTE(PRESENT_TEMPERATURE[i]);
      
      TX_Buff[(i*15)+34]=_LOBYTE(PRESENT_CURRENT[i]);
      TX_Buff[(i*15)+35]=_HIBYTE(PRESENT_CURRENT[i]);
    }
 
    //calculate checkSum
    Check_Sum=0; 
    for(i=2;i<=(19*15)+35;i++)
    {
      Check_Sum = Check_Sum ^ TX_Buff[i];
    }
    //check sum
    TX_Buff[(19*15)+35+1]=_LOBYTE(Check_Sum);
    TX_Buff[(19*15)+35+2]=_HIBYTE(Check_Sum);
}
