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
    int i=0;
    portTickType xLastWakeTime;
    const portTickType xFrequency = 10; 
    xLastWakeTime = xTaskGetTickCount ();    
    //--------------------------------------------------------
    for(i=0; i<=19; i++ )
    {
      id[i] = i+1;
      Speed[i] = 100;   //initialize speed for first time
      Position[i]=2048;  //init goal position for first time
    }    
    ik(0,500, 0, 0, 30, 0);
    ik(1,500, 0, 0, 30, 0);
    Position[13]=2048+1023;
    Position[16]=2048-1023;
    //--------------------------------------------------------
    
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
      xTaskResumeAll();    //Resume all other tasks  
      //or just tx packet??   
      
      
      SerialUSB.print("OK");
      
      //xTaskResumeAll();    //Resume all other tasks
      vTaskDelayUntil( &xLastWakeTime, xFrequency );      
    }
}

//-------------------------------------------------------------------
//the inverse kinematic function for each leg
void ik(byte Leg_NO,int Spd, double X, double Y, double Z, double Yaw)
{
  Z=460-Z;
  int Leg_Direction=1;
  if(Leg_NO==0)
  {
    Y=-Y;
  }
  else
  {
    Leg_Direction=-1;
  }
  int Degree_To_DXL=11.377;
  int Mx_Center=2048;
  
  int Hip_Yaw=(int)((Yaw*Degree_To_DXL)+Mx_Center); 
  double Tmp_X=(X*cos(Yaw)+Y*sin(Yaw));
  double Tmp_Y=(-X*sin(Yaw)+Y*cos(Yaw));
  int Hip_Roll=(int)(atan2(Tmp_Y, Z)*RAD2DEG*Degree_To_DXL)+Mx_Center;
  int Foot_Roll= Hip_Roll;
  double Tmp_Z=sqrt((Tmp_Y*Tmp_Y)+(Z*Z));
  double dfoot=sqrt((Tmp_Z*Tmp_Z)+(Tmp_X*Tmp_X));
  double alpha=atan2(Tmp_X,Tmp_Z);
  double beta=acos(dfoot/(2*230)); 
  int Hip_Pitch=(int)Mx_Center-(Leg_Direction*((alpha+beta)*RAD2DEG)*Degree_To_DXL);
  int Knee=(int)(Leg_Direction*((-2*beta)*RAD2DEG)*Degree_To_DXL)+Mx_Center;
  int Foot_Pitch=(int)(Leg_Direction*((-alpha+beta)*RAD2DEG)*Degree_To_DXL)+Mx_Center;

  /*
  SerialUSB.print("Hip_Yaw=");SerialUSB.print(Hip_Yaw);SerialUSB.print("\t");
  SerialUSB.print("Hip_Roll=");SerialUSB.print(Hip_Roll);SerialUSB.print("\t");
  SerialUSB.print("Hip_Pitch=");SerialUSB.print(Hip_Pitch);SerialUSB.print("\t");
  SerialUSB.print("Knee=");SerialUSB.print(Knee);SerialUSB.print("\t");
  SerialUSB.print("Foot_Pitch=");SerialUSB.print(Foot_Pitch);SerialUSB.print("\t");
  SerialUSB.print("Foot_Roll=");SerialUSB.print(Foot_Roll);SerialUSB.println("\t");
  */
  
  Speed[(Leg_NO*6)+0]=Spd;
  Speed[(Leg_NO*6)+1]=Spd;
  Speed[(Leg_NO*6)+2]=Spd;
  Speed[(Leg_NO*6)+3]=Spd*2;
  Speed[(Leg_NO*6)+4]=Spd;
  Speed[(Leg_NO*6)+5]=Spd;
  
  Position[(Leg_NO*6)+0]=Hip_Yaw; 
  Position[(Leg_NO*6)+1]=Hip_Roll;
  Position[(Leg_NO*6)+2]=Hip_Pitch;  
  Position[(Leg_NO*6)+3]=Knee;  
  Position[(Leg_NO*6)+4]=Foot_Pitch;
  Position[(Leg_NO*6)+5]=Foot_Roll;
  
}









