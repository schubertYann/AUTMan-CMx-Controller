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
// The DXL Task for Dynamixel controll/update
//=====================================================================================================

byte  id[20];

//sensor read thread
void vDXLWTask( void *pvParameters )
{ 
    //Deffine local varible in this task
    int i=0; 
    
    for(i=0; i<=NUM_OF_DXL-1; i++ )
    {
      id[i] = i+1;
      D_MOVING_SPEED[i] = 100;   //initialize speed for first time      
      D_GOAL_POSITION[i]=2048;  //init goal position for first time
      D_TORQUE_ENABLE[i]=1;
      D_STATUS_LED[i]=1;
      D_KD_GAIN[i]=0;
      D_KI_GAIN[i]=0;
      D_KP_GAIN[i]=32;
    }     
    
    vTaskDelay(1000); 
    portTickType xLastWakeTime;
    const portTickType xFrequency = 10;  //10ms for each loop run time means 100Hz of task frequency
    xLastWakeTime = xTaskGetTickCount ();    
    //--------------------------------------------------------
    int xxx=0;
    //--------------------------------------------------------   
    for( ;; )
    {
      DXLWTaskCNT++;
      
      if(DXLWTaskCNT==1)for( i=0; i<=NUM_OF_DXL-1; i++ )D_STATUS_LED[i]=1;
      if(DXLWTaskCNT==5)for( i=0; i<=NUM_OF_DXL-1; i++ )D_STATUS_LED[i]=0;
      
      //:)
      xxx++;
      if(xxx>4000)xxx=0;
      D_GOAL_POSITION[2]=xxx;
      
      DXL.setTxPacketId(BROADCAST_ID);
      DXL.setTxPacketInstruction(INST_SYNC_WRITE);
      DXL.setTxPacketParameter(0, P_TORQUE_ENABLE);  //start register to write
      DXL.setTxPacketParameter(1, 5);                  //count of write per motor
      for( i=0; i<=NUM_OF_DXL-1; i++ )
      {
        DXL.setTxPacketParameter((2+6*i), id[i]);                //id of dynamixel
        
        DXL.setTxPacketParameter((2+6*i)+1, D_TORQUE_ENABLE[i]);  //goal position low
        DXL.setTxPacketParameter((2+6*i)+2, D_STATUS_LED[i]); //goal position high
        
        DXL.setTxPacketParameter((2+6*i)+3, D_KD_GAIN[i]);  //speed low
        DXL.setTxPacketParameter((2+6*i)+4, D_KI_GAIN[i]); //speed high
        DXL.setTxPacketParameter((2+6*i)+5, D_KP_GAIN[i]); //speed high
      }
      DXL.setTxPacketLength((5+1)*18+4);
      
      vTaskSuspendAll();   //Suspend all other tasks
      DXL.txrxPacket();    //Send and resive packet 
      xTaskResumeAll();    //Resume all other tasks
      //--------------------------------------------
      
      DXL.setTxPacketId(BROADCAST_ID);
      DXL.setTxPacketInstruction(INST_SYNC_WRITE);
      DXL.setTxPacketParameter(0, P_GOAL_POSITION_L);  //start register to write
      DXL.setTxPacketParameter(1, 4);                  //count of write per motor
      for( i=0; i<=NUM_OF_DXL-1; i++ )
      {
        if(D_TORQUE_ENABLE[i]==1) {DXL.setTxPacketParameter((2+5*i), id[i]);}                //id of dynamixel
        else {DXL.setTxPacketParameter((2+5*i), i+100);}
              
        DXL.setTxPacketParameter((2+5*i)+1, DXL.getLowByte(D_GOAL_POSITION[i]));  //goal position low
        DXL.setTxPacketParameter((2+5*i)+2, DXL.getHighByte(D_GOAL_POSITION[i])); //goal position high
        
        DXL.setTxPacketParameter((2+5*i)+3, DXL.getLowByte(D_MOVING_SPEED[i]));  //speed low
        DXL.setTxPacketParameter((2+5*i)+4, DXL.getHighByte(D_MOVING_SPEED[i])); //speed high
      }
      DXL.setTxPacketLength((4+1)*18+4);
      
      vTaskSuspendAll();   //Suspend all other tasks
      DXL.txrxPacket();    //Send and resive packet 
      xTaskResumeAll();    //Resume all other tasks
      
      vTaskDelayUntil( &xLastWakeTime, xFrequency );      
    }
}

//-------------------------------------------------------------------









