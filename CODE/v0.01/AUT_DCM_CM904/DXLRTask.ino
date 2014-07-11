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
// The DXLRead Task for Dynamixel update
//=====================================================================================================


//sensor read thread
void vDXLRTask( void *pvParameters )
{
    int i=0;
    portTickType xLastWakeTime;
    const portTickType xFrequency = 10;  
    xLastWakeTime = xTaskGetTickCount ();
    for(;;)
    {
      DXLRTaskCNT++;    
      if((DXLRTaskCNT%10)==0) togglePin(10);             
        
      for(i=0; i<=11; i++ )
      {
        
        Dxl.setTxPacketId(i+1);
        Dxl.setTxPacketInstruction(INST_READ);
        Dxl.setTxPacketParameter(0, 36);       //start register to read
        Dxl.setTxPacketParameter(1,(69-36)+1); //count register to read
        Dxl.setTxPacketLength(4);
        
        vTaskSuspendAll();   //Suspend all other tasks
        Dxl.txrxPacket();    //Send and resive packet
        xTaskResumeAll();   //Resume all other tasks
        
        if(Dxl.available()==0) //if data was read successfully
        { 
          PRESENT_POSITION[i]=Dxl.makeWord(Dxl.getRxPacketParameter(0), Dxl.getRxPacketParameter(1));
          PRESENT_SPEED[i]=Dxl.makeWord(Dxl.getRxPacketParameter(2), Dxl.getRxPacketParameter(3));
          PRESENT_LOAD[i]=Dxl.makeWord(Dxl.getRxPacketParameter(4), Dxl.getRxPacketParameter(5));
          PRESENT_VOLTAGE[i]=Dxl.getRxPacketParameter(6);
          PRESENT_TEMPERATURE[i]=Dxl.getRxPacketParameter(7);          
          PRESENT_CURRENT[i]=Dxl.makeWord(Dxl.getRxPacketParameter(13), Dxl.getRxPacketParameter(14));
        }     
      }     
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
