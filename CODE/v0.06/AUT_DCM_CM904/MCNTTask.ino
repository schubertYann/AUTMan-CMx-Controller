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

#define Left_Arm_Pitch      (12)
#define Right_Arm_Pitch     (15)


#define Left_Hip_Yaw      (0)
#define Left_Hip_Roll     (1)
#define Left_Hip_Pitch    (2)
#define Left_Knee         (3)
#define Left_Foot_Pitch   (4)
#define Left_Foot_Roll    (5)

#define Right_Hip_Yaw      (6)
#define Right_Hip_Roll     (7)
#define Right_Hip_Pitch    (8)
#define Right_Knee         (9)
#define Right_Foot_Pitch   (10)
#define Right_Foot_Roll    (11)

  double leg_lenth=230; //170;  //the len of legs

  //offset of each leg 
  double X_Offset=0;     // X offset of legs in milimeter
  double Y_Offset=-1;   // Y offset of legs in milimeter
  double Z_Offset=90;    // Z offset of legs in milimeter
  
  double Roll_Offset=0.0;   // Roll offset of legs foot in radian
  double Pitch_Offset=0.0;  // Pitch offset of legs foot in radian
  double Yaw_Offset=0.0;    // Yaw offset of legs foot in radian
  
  // hip joint direct offset in radian
  double Hip_Roll_Offset=0.0;
  double Hip_Pitch_Offset=0.12;
  double Hip_Yaw_Offset=0.0;
  
  // foot joint direct offset in radian
  double Foot_Roll_Offset=0.0;
  double Foot_Pitch_Offset=0.0;
  
  double Step_Height=1.0;          //the step height can be 0.1 to 1 which 1 mean the highest(norman in sin) value of sine in each gait
  double Gait_Frequency=0.991;       //the gait frequency ( changes 0 to 1) >> 0 is the lowest frequency, 1 is higest frequency
  double Motion_Resolution=5;      // resolution of motion trajectory
  double Motion_Speed=1.0;         //the speed of motion (this mean is directly motors speed)
  double Double_Support_Sleep=0.03; //the double support time gap (0 is no time for double support, and 1 is the maximum time)
  byte Flying_Leg_Slope=5;      //is the propotional gain of the actuators when leg is flying
  byte Support_Leg_Slope=35;     //propotional gain of actuators when leg is support
  
  double Vx=30;      //velocity of X (forward) direction
  double Vy=0;//-20;        //velocity of Y (sideward) direction
  double Vt=0;        //velocity of T (rotate) speed
  
  
  
  
  
  
  //support leg ik value
  double Support_X=0;
  double Support_Y=0;
  double Support_Z=0;
  double Support_Roll=0;
  double Support_Pitch=0;
  double Support_Yaw=0;
  
  //Flyer leg ik value
  double Flying_X=0;
  double Flying_Y=0;
  double Flying_Z=0;
  double Flying_Roll=0;
  double Flying_Pitch=0;
  double Flying_Yaw=0;
  
  //the varible of ik function
  double L_X;
  double L_Y;
  double L_Z;
  double L_Roll;
  double L_Pitch;
  double L_Yaw;
  
  double R_X;
  double R_Y;
  double R_Z;
  double R_Roll;
  double R_Pitch;
  double R_Yaw;
  
//Serial Task runs under RTOS
void vMCNTTask( void *pvParameters )
{ 
  double i=0;
  byte Gait_Direction=0;
  
  vTaskDelay(1000);
  portTickType xLastWakeTime;
  const portTickType xFrequency = 50;
  xLastWakeTime = xTaskGetTickCount ();
  /*
  for( ;; )
  {
    Angle[Right_Hip_Roll]=0.0;
    vTaskDelay(500);
    Angle[Right_Hip_Roll]=0.2;
    vTaskDelay(500);
  }
  */
  //-----------------------------------  
  for( ;; )
  {  
    //MCNTTaskCNT++; //valu is used for calculate update frequency of this task
    
    if(Gait_Direction==0) Gait_Direction=1; else Gait_Direction=0;
    
    //gait generate
    for(i=0;i<=180;i+=Motion_Resolution)
    {
      vTaskSuspendAll();   //Suspend all other tasks    
      //------------
      
      Flying_X =sin((i/2)*DEG2RAD)*Vx;  // ((i<=90) ? 0 : ((i/90)-1)*Vx);       
      Support_X=(i<90) ?   sin((90-i)*DEG2RAD)*Vx :   -sin((i-90)*DEG2RAD)*Vx; //(-(i<=90) ? 0 : ((i/90)-1)*Vx/2);   
          
      if(Vy>0)
      {    
        if(Gait_Direction==0)
        {  
          Y_Offset=10;
          Flying_Y = ((sin(i*DEG2RAD)*RAD2DEG)+Y_Offset)+Vy; 
          Support_Y= ((sin(i*DEG2RAD)*RAD2DEG)+Y_Offset);  
        }
      }
      if(Vy<0)
      {
        if(Gait_Direction>0)
        {
          Y_Offset=10;
          Flying_Y = ((sin(i*DEG2RAD)*RAD2DEG)+Y_Offset) + (-1)*Vy; 
          Support_Y= ((sin(i*DEG2RAD)*RAD2DEG)+Y_Offset);
        }
      }
      if(Vy==0)
      {
        Y_Offset=0;
        Flying_Y = ((sin(i*DEG2RAD)*RAD2DEG)+Y_Offset); 
        Support_Y= ((sin(i*DEG2RAD)*RAD2DEG)+Y_Offset);
      }
      
      Flying_Z = (Z_Offset+(sin(i*DEG2RAD)*RAD2DEG)*Step_Height); 
      Support_Z= (Z_Offset);
      
      Flying_Roll = (0);
      Support_Roll= (sin(i*DEG2RAD)/4);
                      
      Flying_Pitch = (0);
      Support_Pitch= (0);
      
      if(Vt>0)
      {
        if(Gait_Direction==0)
        {
          Flying_Yaw =sin((i/2)*DEG2RAD)*Vt;    // (Vt/2);
          Support_Yaw=(i<90) ?   sin((90-i)*DEG2RAD)*Vt : 0; //  -sin((i-90)*DEG2RAD)*Vt;
        }
        else
        {
          Flying_Yaw =0;    // (Vt/2);
          Support_Yaw=0; //  -sin((i-90)*DEG2RAD)*Vt;
        }
      }
      
      if(Vt<0)
      {
        if(Gait_Direction>0)
        {
          Flying_Yaw =sin((i/2)*DEG2RAD)*(-1)*Vt;    // (Vt/2);
          Support_Yaw=(i<90) ?   sin((90-i)*DEG2RAD)*(-1)*Vt : 0; //  -sin((i-90)*DEG2RAD)*Vt;
        }
        else
        {
          Flying_Yaw =0;    // (Vt/2);
          Support_Yaw=0; //  -sin((i-90)*DEG2RAD)*Vt;
        }
      }
      
      //-------------
      /*
      ik(100,
      FX,FY,FZ,
      FRoll,FPitch,FYaw,
      SX,SY,SZ,
      SRoll,SPitch,SYaw);
      */
      
      if(Gait_Direction==0)
      {        
        //Angle[Left_Arm_Pitch]=;
        
        D_KP_GAIN[Left_Hip_Yaw]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Left_Hip_Roll]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Left_Hip_Pitch]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Left_Knee]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Left_Foot_Pitch]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Left_Foot_Roll]=(byte)Flying_Leg_Slope;
        
        L_X=Flying_X;
        L_Y=Flying_Y;
        L_Z=Flying_Z;
        L_Roll=Flying_Roll;
        L_Pitch=Flying_Pitch;
        L_Yaw=Flying_Yaw;
        
        
        D_KP_GAIN[Right_Hip_Yaw]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Right_Hip_Roll]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Right_Hip_Pitch]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Right_Knee]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Right_Foot_Pitch]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Right_Foot_Roll]=(byte)Support_Leg_Slope;
        
        R_X=Support_X;
        R_Y=Support_Y;
        R_Z=Support_Z;
        R_Roll=Support_Roll;
        R_Pitch=Support_Pitch;
        R_Yaw=Support_Yaw;    
      }  
      else
      {     
         
        D_KP_GAIN[Left_Hip_Yaw]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Left_Hip_Roll]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Left_Hip_Pitch]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Left_Knee]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Left_Foot_Pitch]=(byte)Support_Leg_Slope;
        D_KP_GAIN[Left_Foot_Roll]=(byte)Support_Leg_Slope;
        
        L_X=Support_X;
        L_Y=Support_Y;
        L_Z=Support_Z;
        L_Roll=Support_Roll;
        L_Pitch=Support_Pitch;
        L_Yaw=Support_Yaw;
        
        
        D_KP_GAIN[Right_Hip_Yaw]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Right_Hip_Roll]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Right_Hip_Pitch]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Right_Knee]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Right_Foot_Pitch]=(byte)Flying_Leg_Slope;
        D_KP_GAIN[Right_Foot_Roll]=(byte)Flying_Leg_Slope;
        
        R_X=Flying_X;
        R_Y=Flying_Y;
        R_Z=Flying_Z;
        R_Roll=Flying_Roll;
        R_Pitch=Flying_Pitch;
        R_Yaw=Flying_Yaw;
      }
      
      ik(Motion_Speed);  //call inverse kinematic
      //vTaskDelay(1);
      MCNTTaskCNT++; //valu is used for calculate update frequency of this task
      xTaskResumeAll();    //Resume all other tasks 
      vTaskDelay((1-Gait_Frequency)*1000);
      //for(byte tmp_i=0;tmp_i<=(1000-(Gait_Frequency*1000));tmp_i++) vTaskDelay(10); //calculate gait frequency using sleep
    }      
    vTaskDelay(Double_Support_Sleep*1000);
    //for(byte tmp_i=0;tmp_i<=(1000-(Double_Support_Sleep*1000));tmp_i++) vTaskDelay(10); //double support sleep
    //vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
}

//the inverse kinematic function for each leg
void ik(double Speed)//, double L_X, double L_Y, double L_Z, double L_Roll, double L_Pitch, double L_Yaw, double R_X, double R_Y, double R_Z, double R_Roll, double R_Pitch, double R_Yaw)
{
  if(Speed<=0) Speed=0.0001;
  if(Speed>=1) Speed=1;
  int Knee_Speed=(int)(Speed*1023);
  int J_Speed=(int)(Knee_Speed/2);
  byte i=0;
  for(i=0;i<=11;i++)
  { 
    D_MOVING_SPEED[i]=J_Speed;
  } 
  D_MOVING_SPEED[Left_Knee]=Knee_Speed;
  D_MOVING_SPEED[Right_Knee]=Knee_Speed;
  
  double Tmp_angle=0;
  
  L_Z=(leg_lenth*2)-L_Z;
  R_Z=(leg_lenth*2)-R_Z;
  
  Angle[Left_Hip_Yaw] = -L_Yaw;
  Angle[Right_Hip_Yaw]=  R_Yaw; 
  
  double Tmp_L_X=(L_X*cos(L_Yaw)+L_Y*sin(L_Yaw));
  double Tmp_L_Y=(-L_X*sin(L_Yaw)+L_Y*cos(L_Yaw));
  Tmp_angle=atan2(Tmp_L_Y, L_Z); 
  Angle[Left_Hip_Roll]=-Tmp_angle;
  Angle[Left_Foot_Roll]=Angle[Left_Hip_Roll]+L_Roll;
  
  
  double Tmp_R_X=(R_X*cos(R_Yaw)+R_Y*sin(R_Yaw));
  double Tmp_R_Y=(-R_X*sin(R_Yaw)+R_Y*cos(R_Yaw));
  Tmp_angle=atan2(Tmp_R_Y, R_Z); 
  Angle[Right_Hip_Roll]=Tmp_angle;
  Angle[Right_Foot_Roll]=Angle[Right_Hip_Roll]-R_Roll;
  
  double Tmp_L_Z=sqrt((Tmp_L_Y*Tmp_L_Y)+(L_Z*L_Z)); 
  double L_dfoot=sqrt((Tmp_L_Z*Tmp_L_Z)+(Tmp_L_X*Tmp_L_X));
  double L_alpha=atan2(Tmp_L_X,Tmp_L_Z);
  double L_beta=acos(L_dfoot/(2*leg_lenth)); 
  Angle[Left_Hip_Pitch]=-(L_alpha+L_beta)-Hip_Pitch_Offset;
  Angle[Left_Knee]=-(-2*L_beta);   
  Angle[Left_Foot_Pitch]=(-L_alpha+L_beta)+L_Pitch;
  
  double Tmp_R_Z=sqrt((Tmp_R_Y*Tmp_R_Y)+(R_Z*R_Z)); 
  double R_dfoot=sqrt((Tmp_R_Z*Tmp_R_Z)+(Tmp_R_X*Tmp_R_X));
  double R_alpha=atan2(Tmp_R_X,Tmp_R_Z);
  double R_beta=acos(R_dfoot/(2*leg_lenth)); 
  Angle[Right_Hip_Pitch]=(R_alpha+R_beta)+Hip_Pitch_Offset;
  Angle[Right_Knee]=(-2*R_beta);   
  Angle[Right_Foot_Pitch]=-(-R_alpha+R_beta)-R_Pitch;
  
  //SerialUSB.print("Hip_Yaw=");SerialUSB.print(Hip_Yaw);SerialUSB.print("\t");
  //SerialUSB.print("Hip_Roll=");SerialUSB.print(Hip_Roll);SerialUSB.print("\t");
  //SerialUSB.print("Hip_Pitch=");SerialUSB.print(Hip_Pitch);SerialUSB.print("\t");
  //SerialUSB.print("Knee=");SerialUSB.print(Knee);SerialUSB.print("\t");
  //SerialUSB.print("Foot_Pitch=");SerialUSB.print(Foot_Pitch);SerialUSB.print("\t");
  //SerialUSB.print("Foot_Roll=");SerialUSB.print(Foot_Roll);SerialUSB.println("\t");
}

