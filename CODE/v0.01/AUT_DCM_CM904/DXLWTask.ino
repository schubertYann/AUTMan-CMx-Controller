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
// The DXL Task for Dynamixel controll/update
//=====================================================================================================

byte  id[20];

//sensor read thread
void vDXLWTask( void *pvParameters )
{ 
    //Deffine local varible in this task
    portTickType xLastWakeTime;
    const portTickType xFrequency = 10;
   
    int i=0;
    //--------------------------------------------------------
    for(i=0; i<=11; i++ )
    {
      id[i] = i+1;
      Speed[i] = 100;   //initialize speed for first time
      Position[i]=2048;  //init goal position for first time
    }
    
    for(i=12; i<=19; i++ )
    {
      id[i] = i+1;
      Speed[i] = 100;   //initialize speed for first time
      Position[i]=512;  //init goal position for first time
    }
   
    //--------------------------------------------------------
    xLastWakeTime = xTaskGetTickCount ();
    for( ;; )
    {
      DXLWTaskCNT++;
      
      //if(DXLWTaskCNT==1)
      //{
      //  Dxl.writeByte(254,P_LED,1);
      //}
      
      
      Dxl.setTxPacketId(BROADCAST_ID);
      Dxl.setTxPacketInstruction(INST_SYNC_WRITE);
      Dxl.setTxPacketParameter(0, P_GOAL_POSITION_L);  //start register to write
      Dxl.setTxPacketParameter(1, 4);                  //count of write per motor
  
      for( i=0; i<=19; i++ )
      {
        Dxl.setTxPacketParameter((2+5*i), id[i]);                //id of dynamixel
        
        Dxl.setTxPacketParameter((2+5*i)+1, Dxl.getLowByte(Position[i]));  //goal position low
        Dxl.setTxPacketParameter((2+5*i)+2, Dxl.getHighByte(Position[i])); //goal position high
        
        Dxl.setTxPacketParameter((2+5*i)+3, Dxl.getLowByte(Speed[i]));  //speed low
        Dxl.setTxPacketParameter((2+5*i)+4, Dxl.getHighByte(Speed[i])); //speed high
    
        //SerialUSB.println(GoalPos);
      }
      Dxl.setTxPacketLength((4+1)*18+4);
      
      vTaskSuspendAll();   //Suspend all other tasks
      Dxl.txrxPacket();    //Send and resive packet    
      //or just tx packet??   

      //if(DXLWTaskCNT==1)
      //{
      //  Dxl.writeByte(254,P_LED,0);
      //}
      xTaskResumeAll();    //Resume all other tasks
      vTaskDelayUntil( &xLastWakeTime, xFrequency );      
      
  }
}
