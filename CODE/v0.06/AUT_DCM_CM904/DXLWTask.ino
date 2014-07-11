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
// The DXL Task for Dynamixel controll/update
//=====================================================================================================
byte  id[NUM_OF_DXL];

//sensor read thread
void vDXLWTask( void *pvParameters )
{ 
    //Deffine local varible in this task
    int i=0; 
    
    for(i=0; i<=NUM_OF_DXL-1; i++ )
    {
      Angle[i]=0.0;
    }
    //update the defult arrays for send for first time
    for(i=0; i<=NUM_OF_DXL-1; i++ )
    {
      id[i] = i+1;
      D_MOVING_SPEED[i] = 100;   //initialize speed for first time      
      D_GOAL_POSITION[i]=2048+((int)((Angle[i]*RAD2DEG)*DEG2DXL));  //init goal position for first time
      D_TORQUE_ENABLE[i]=1;
      D_STATUS_LED[i]=1;
      D_KD_GAIN[i]=0;
      D_KI_GAIN[i]=0;
      D_KP_GAIN[i]=32;
    }     
    
    Angle[12]=10*DEG2RAD;
    Angle[15]=-10*DEG2RAD;
    
    Angle[13]=84*DEG2RAD;
    Angle[16]=-84*DEG2RAD;
    
    Angle[14]=-70*DEG2RAD;
    Angle[17]=70*DEG2RAD;
    
    vTaskDelay(1000); 
    portTickType xLastWakeTime;
    const portTickType xFrequency = 20;  //10ms for each loop run time means 100Hz of task frequency
    xLastWakeTime = xTaskGetTickCount ();    
    //--------------------------------------------------------
    //--------------------------------------------------------   
    for( ;; )
    {
      DXLWTaskCNT++;
      
      //this code is just use for test and blink the dynalixels
      if(DXLWTaskCNT==1) for( i=0; i<=NUM_OF_DXL-1; i++ )D_STATUS_LED[i]=1;
      if(DXLWTaskCNT==10)for( i=0; i<=NUM_OF_DXL-1; i++ )D_STATUS_LED[i]=0;
      
      
      //fill the send pocket for sync write of:
      //torque enable
      //status led
      //kd, ki, kp gains
      DXL.setTxPacketId(BROADCAST_ID);
      DXL.setTxPacketInstruction(INST_SYNC_WRITE);
      DXL.setTxPacketParameter(0, P_TORQUE_ENABLE);  //start register to write
      DXL.setTxPacketParameter(1, 5);                  //count of write per motor
      for( i=0; i<=NUM_OF_DXL-1; i++ )
      {
        DXL.setTxPacketParameter((2+6*i), id[i]);                //id of dynamixel
        
        DXL.setTxPacketParameter((2+6*i)+1, D_TORQUE_ENABLE[i]);  //torque enable
        DXL.setTxPacketParameter((2+6*i)+2, D_STATUS_LED[i]);     //status led
        
        DXL.setTxPacketParameter((2+6*i)+3, D_KD_GAIN[i]);  //kd gain
        DXL.setTxPacketParameter((2+6*i)+4, D_KI_GAIN[i]);  //ki gain
        DXL.setTxPacketParameter((2+6*i)+5, D_KP_GAIN[i]);  //kp gain
      }
      DXL.setTxPacketLength((5+1)*18+4);
      
      vTaskSuspendAll();   //Suspend all other tasks
      DXL.txrxPacket();    //Send and resive packet 
      xTaskResumeAll();    //Resume all other tasks
      //--------------------------------------------
      
      //fill the send pocket for sync write of:
      //goal position
      //speed
      DXL.setTxPacketId(BROADCAST_ID);
      DXL.setTxPacketInstruction(INST_SYNC_WRITE);
      DXL.setTxPacketParameter(0, P_GOAL_POSITION_L);  //start register to write
      DXL.setTxPacketParameter(1, 4);                  //count of write per motor
      for( i=0; i<=NUM_OF_DXL-1; i++ )
      {
        if(D_TORQUE_ENABLE[i]==1) {DXL.setTxPacketParameter((2+5*i), id[i]);}     //id of dynamixel
        else {DXL.setTxPacketParameter((2+5*i), i+100);}
        
        D_GOAL_POSITION[i]=2048+((int)((Angle[i]*RAD2DEG)*DEG2DXL));  //init goal position for first time
        
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









