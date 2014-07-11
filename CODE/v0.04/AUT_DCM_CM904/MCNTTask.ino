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
// The System Debug task for send information to PC
//=====================================================================================================

  double X_offset=0;
  double Y_offset=-12;
  double Z_offset=50;
  
  double Hip_Pitch_Offset=0.05;
  double Hip_Roll_Offset=0.05;
  double Hip_Yaw_Offset=0.05;
  
  double Step_Height=1;
  double Motion_resolution=0.9;
  
  double Vx=150;
  double Vy=0;
  double Vt=0;
  
  double LX=0;
  double LY=0;
  double LZ=0;
  double LRoll=0;
  double LPitch=0;
  double LYaw=0;
  
  double RX=0;
  double RY=0;
  double RZ=0;
  double RRoll=0;
  double RPitch=0;
  double RYaw=0;
  
//Serial Task runs under RTOS
void vMCNTTask( void *pvParameters )
{ 
  double i=0;
  vTaskDelay(1000);
  portTickType xLastWakeTime;
  const portTickType xFrequency = 10;
  xLastWakeTime = xTaskGetTickCount ();
  //-----------------------------------
  
  
  for( ;; )
  {  
    MCNTTaskCNT++; //valu is used for calculate update frequency of this task
    
    //gait one
    for(i=0;i<=180;i+=Motion_resolution)
    {
      //ik(byte Leg_NO,int Spd, double X, double Y, double Z, double Roll, double Pitch, double Yaw);
      vTaskSuspendAll();   //Suspend all other tasks 
      //------------
      LX=(i<=90) ? 0 : ((i/90)-1)*Vx;
      LY=(sin(i*DEG2RAD)*RAD2DEG)+Y_offset;
      LZ=Z_offset+(sin(i*DEG2RAD)*RAD2DEG)*Step_Height;
      LRoll=0;
      LPitch=0;
      LYaw=0;
      
      //-------------
      RX=-(i<=90) ? 0 : ((i/90)-1)*Vx/2;
      RY=(sin(i*DEG2RAD)*RAD2DEG)+Y_offset;
      RZ=Z_offset;
      RRoll=sin(i*DEG2RAD)/4;
      RPitch=0;//-(pitch*2)*DEG2RAD;
      RYaw=0;
      //-------------
      ik(0,100,LX,LY,LZ ,LRoll,LPitch,LYaw);
      ik(1,100,RX,RY,RZ ,RRoll,RPitch,RYaw);    
      xTaskResumeAll();    //Resume all other tasks    
      vTaskDelay(1);
    }    
    
    //gaite two
    for(i=0;i<=180;i+=Motion_resolution)
    {
      //ik(byte Leg_NO,int Spd, double X, double Y, double Z, double Roll, double Pitch, double Yaw);
      vTaskSuspendAll();   //Suspend all other tasks 
      //------------
      LX=-(i<=90) ? 0 : ((i/90)-1)*Vx/2;
      LY=(sin(i*DEG2RAD)*RAD2DEG)+Y_offset;
      LZ=Z_offset;
      
      LRoll=sin(i*DEG2RAD)/4;
      LPitch=0;//-(pitch*2)*DEG2RAD;
      LYaw=0;
      
      //-------------
      RX=(i<=90) ? 0 : ((i/90)-1)*Vx;
      RY=(sin(i*DEG2RAD)*RAD2DEG)+Y_offset;
      RZ=Z_offset+(sin(i*DEG2RAD)*RAD2DEG)*Step_Height;
   
      RRoll=0;
      RPitch=0;
      RYaw=0;
      //-------------
      ik(0,100,LX,LY,LZ ,LRoll,LPitch,LYaw);
      ik(1,100,RX,RY,RZ ,RRoll,RPitch,RYaw);      
      xTaskResumeAll();    //Resume all other tasks
      vTaskDelay(1);
    }   
    
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

//the inverse kinematic function for each leg
void ik(byte Leg_NO,int Spd, double X, double Y, double Z, double Roll, double Pitch, double Yaw)
{
  Pitch+=0.08;
  double leg_lenth=170;
  double Tmp_angle=0;
  
  Z=(leg_lenth*2)-Z;
  
  double Hip_Yaw=(Leg_NO==0) ? -Yaw : Yaw; 
  
  double Tmp_X=(X*cos(Yaw)+Y*sin(Yaw));
  double Tmp_Y=(-X*sin(Yaw)+Y*cos(Yaw));
  Tmp_angle=atan2(Tmp_Y, Z); 
  double Hip_Roll=(Leg_NO==0) ? -Tmp_angle : Tmp_angle;
  double Foot_Roll=Hip_Roll+ ((Leg_NO==0) ?  Roll : -Roll); 
  double Tmp_Z=sqrt((Tmp_Y*Tmp_Y)+(Z*Z)); 
  double dfoot=sqrt((Tmp_Z*Tmp_Z)+(Tmp_X*Tmp_X));
  double alpha=atan2(Tmp_X,Tmp_Z);
  double beta=acos(dfoot/(2*leg_lenth)); 
  double Hip_Pitch=((Leg_NO==0) ? (-1*(alpha+beta)) : (alpha+beta));
  double Knee=((Leg_NO==0) ? (-1*(-2*beta)) : (-2*beta));   
  double Foot_Pitch=((Leg_NO==0) ? ((-alpha+beta)+Pitch) : ((-1*(-alpha+beta))-Pitch));

  
  //SerialUSB.print("Hip_Yaw=");SerialUSB.print(Hip_Yaw);SerialUSB.print("\t");
  //SerialUSB.print("Hip_Roll=");SerialUSB.print(Hip_Roll);SerialUSB.print("\t");
  //SerialUSB.print("Hip_Pitch=");SerialUSB.print(Hip_Pitch);SerialUSB.print("\t");
  //SerialUSB.print("Knee=");SerialUSB.print(Knee);SerialUSB.print("\t");
  //SerialUSB.print("Foot_Pitch=");SerialUSB.print(Foot_Pitch);SerialUSB.print("\t");
  //SerialUSB.print("Foot_Roll=");SerialUSB.print(Foot_Roll);SerialUSB.println("\t");
  
  
  D_MOVING_SPEED[(Leg_NO*6)+0]=Spd;
  D_MOVING_SPEED[(Leg_NO*6)+1]=Spd;
  D_MOVING_SPEED[(Leg_NO*6)+2]=Spd;
  D_MOVING_SPEED[(Leg_NO*6)+3]=Spd*2;
  D_MOVING_SPEED[(Leg_NO*6)+4]=Spd;
  D_MOVING_SPEED[(Leg_NO*6)+5]=Spd;
  
  Angle[(Leg_NO*6)+0]=Hip_Yaw;
  Angle[(Leg_NO*6)+1]=Hip_Roll;  
  Angle[(Leg_NO*6)+2]=Hip_Pitch+((Leg_NO==0) ? -Hip_Pitch_Offset : Hip_Pitch_Offset);
  Angle[(Leg_NO*6)+3]=Knee;
  Angle[(Leg_NO*6)+4]=Foot_Pitch;
  Angle[(Leg_NO*6)+5]=Foot_Roll;
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


