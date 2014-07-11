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
// The DXLRead Task for Dynamixel update
//=====================================================================================================

void vDXLRTask( void *pvParameters )
{
    int i=0;
    vTaskDelay(1000);
    portTickType xLastWakeTime;
    const portTickType xFrequency = 10;  //this mean each 10 mili secound it shoud be run (100Hz)
    xLastWakeTime = xTaskGetTickCount ();
    //-----------------------------------
    for(;;)
    {
      DXLRTaskCNT++;    
      //if((DXLRTaskCNT%10)==0) togglePin(WLED); 
      if((DXLRTaskCNT)==1)  //ping motors each 100 ms
      {        
        for(i=0;i<=NUM_OF_DXL-1;i++)
        {
          vTaskSuspendAll();   //Suspend all other tasks
          PRESENT_MODEL_NUMBER[i]=DXL.readWord(i+1,P_MODEL_NUMBER_L); 
          xTaskResumeAll();   //Resume all other tasks 
        }               
      }
      
      for(i=0; i<=NUM_OF_DXL-1; i++ )
      {
        if(PRESENT_MODEL_NUMBER[i]!=65535)
        { 
          DXL_Exist[i]=1;
          
          DXL.setTxPacketId(i+1);
          DXL.setTxPacketInstruction(INST_READ);
          DXL.setTxPacketParameter(0, P_PRESENT_POSITION_L);       //start register to read
          DXL.setTxPacketParameter(1,(69-P_PRESENT_POSITION_L)+1); //count register to read
          DXL.setTxPacketLength(4);
        
          vTaskSuspendAll();   //Suspend all other tasks
          DXL.txrxPacket();    //Send and resive packet
          xTaskResumeAll();   //Resume all other tasks
        
          if(DXL.available()==0) //if data was read successfully
          { 
            PRESENT_POSITION[i]=DXL.makeWord(DXL.getRxPacketParameter(0), DXL.getRxPacketParameter(1));
            PRESENT_SPEED[i]=DXL.makeWord(DXL.getRxPacketParameter(2), DXL.getRxPacketParameter(3));
            PRESENT_LOAD[i]=DXL.makeWord(DXL.getRxPacketParameter(4), DXL.getRxPacketParameter(5));
            PRESENT_VOLTAGE[i]=DXL.getRxPacketParameter(6);
            PRESENT_TEMPERATURE[i]=DXL.getRxPacketParameter(7);          
            PRESENT_CURRENT[i]=DXL.makeWord(DXL.getRxPacketParameter(13), DXL.getRxPacketParameter(14));
          }  
          else
          {            
            //error acure during read from motor[i]
            //it can be handle from high level
            PRESENT_POSITION[i]=-1;
            PRESENT_SPEED[i]=-1;
            PRESENT_LOAD[i]=-1;
            PRESENT_VOLTAGE[i]=-1;
            PRESENT_TEMPERATURE[i]=-1;          
            PRESENT_CURRENT[i]=-1;
          }
        }
        else
        {
            DXL_Exist[i]=0;
            
            PRESENT_POSITION[i]=-1;
            PRESENT_SPEED[i]=-1;
            PRESENT_LOAD[i]=-1;
            PRESENT_VOLTAGE[i]=-1;
            PRESENT_TEMPERATURE[i]=-1;          
            PRESENT_CURRENT[i]=-1;
        }   
      } 
      vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }    
    //-------------------------------------
}
