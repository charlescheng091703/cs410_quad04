#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include <stdint.h>
#include <signal.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <curses.h>
#include "vive.h"
//gcc -o week1 week_1.cpp -lwiringPi -lncurses -lm

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C


enum Ascale {
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_16G
};
 
enum Gscale {
  GFS_250DPS = 0,
  GFS_500DPS,
  GFS_1000DPS,
  GFS_2000DPS
};
 
int setup_imu();
void calibrate_imu();      
void read_imu();    
void update_filter();
int check_imu_limits();

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; //gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;

struct Keyboard {
  int keypress;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num;
};


Keyboard* shared_memory; 

//function to add
void setup_keyboard()
{

  int segment_id;   
  struct shmid_ds shmbuffer; 
  int segment_size; 
  const int shared_segment_size = 0x6400; 
  int smhkey=33222;
  
  /* Allocate a shared memory segment.  */ 
  segment_id = shmget (smhkey, shared_segment_size,IPC_CREAT | 0666); 
  /* Attach the shared memory segment.  */ 
  shared_memory = (Keyboard*) shmat (segment_id, 0, 0); 
  printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size  =               shmbuffer.shm_segsz; 
  printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 

}

 //declare global struct
Position local_p;

int main (int argc, char *argv[])
{

    
    setup_imu();
    init_shared_memory();
	setup_keyboard();
    
    while(1)
    {
     
//run this command at the start of the while(1) loop to refresh vive data
 local_p=*position;  
 Keyboard keyboard=*shared_memory;
	
 printf("\n\r Vive data: version %d x=%f y=%f z=%f yaw=%f \n\r",local_p.version, local_p.x,local_p.y, local_p.z,local_p.yaw);
 printf("Joystick data: key=%d pitch=%d roll=%d yaw=%d thrust=%d seq_num=%d\n\r", keyboard.keypress,keyboard.pitch,keyboard.roll,keyboard.yaw,keyboard.thrust,keyboard.sequence_num);
 read_imu();
 update_filter();
 printf("IMU reports roll %f pitch%f\n\r",roll_angle,pitch_angle);
 //now you can use the vive sensor values: 
 /*local_p.version
 local_p.x
 local_p.y
 local_p.z
 local_p.yaw*/
     // printf("rp %f %f\n\r",roll_angle,pitch_angle);
    }
      

  
}

void calibrate_imu()
{
 
  
  float sums[6];
  for(int i=0;i<6;i++)
  {
    sums[i]=0;  
  }
  
  for(int i=0;i<1000;i++)
  {
    int address=59;
    float ax=0;
    float az=0;
    float ay=0; 
    float gx=0;
    float gy=0;
    float gz=0;
    int vh,vl;
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }          
    ax=((float)vw* 2.0/32768.0);
    
    
    address=59+2;
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }          
    ay=((float)vw* 2.0/32768.0);
    
    
    address=59+4;
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }          
    az=((float)vw* 2.0/32768.0);
    
    
    address=67;
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }          
    gx=(float)vw* 500.0/32768.0;
  //  printf(" %d %d ",vh,vl);
    address=67+2;
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }          
    gy=(float)vw* 500.0/32768.0;
  
    address=67+4;
    vh=wiringPiI2CReadReg8(imu,address);
    vl=wiringPiI2CReadReg8(imu,address+1);
    vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
    if(vw>0x8000)
    {
      vw=vw ^ 0xffff;
      vw=-vw-1;
    }          
    gz=(float)vw* 500.0/32768.0;
      
    //printf(" %f  %f  %f\n\r",gx,gy,gz );
      
      
      

    //complementary filter for roll
    sums[3]+= atan2f(-ay,-az)*180/M_PI;//pitch accel  
    sums[4]+= atan2f(ax,-az)*180/M_PI;//roll accel
    
                
     sums[0]+=gx;
     sums[1]+=gy;
     sums[2]+=gz;
     sums[5]+=az;
        
  }
  
  x_gyro_calibration=-sums[0]/1000;
  y_gyro_calibration=-sums[1]/1000;
  z_gyro_calibration=-sums[2]/1000;
  roll_calibration=-sums[4]/1000;
  pitch_calibration=-sums[3]/1000;
  accel_z_calibration=-sums[5]/1000;
  
printf("calibration complete, %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);


}

void read_imu()
{
  int address=59;
  float ax=0;
  float az=0;
  float ay=0; 
  int vh,vl;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  int vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[3]=((float)vw* 2.0/32768.0);
  
  
  address=59+2;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[4]=((float)vw* 2.0/32768.0);
  
  
  address=59+4;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[5]=accel_z_calibration+((float)vw* 2.0/32768.0);
  
  
  address=67;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[0]=x_gyro_calibration+(float)vw* 500.0/32768.0;
  
  address=67+2;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
 imu_data[1]=y_gyro_calibration+(float)vw* 500.0/32768.0;
  
  address=67+4;
  vh=wiringPiI2CReadReg8(imu,address);
  vl=wiringPiI2CReadReg8(imu,address+1);
  vw=(((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if(vw>0x8000)
  {
    vw=vw ^ 0xffff;
    vw=-vw-1;
  }          
  imu_data[2]=z_gyro_calibration+(float)vw* 500.0/32768.0;
  
 

  
  //printf("imu reading gyro(xyz) %10.5f %10.5f %10.5f accel(xyz) %10.5f %10.5f %10.5f \n",imu_data[0],imu_data[1],imu_data[2],imu_data[3],imu_data[4],imu_data[5]);
  

}

void update_filter()
{

  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  float imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  
  yaw+=imu_data[2]*imu_diff;
  
  float delta_x_rotate=imu_data[0]*imu_diff;
  float delta_y_rotate=imu_data[1]*imu_diff;
  float delta_z_rotate=imu_data[2]*imu_diff;
  float exp_filter_comp=.99;
  
//complementary filter for roll
  float pitch_accel= (pitch_calibration+(atan2f(-imu_data[4],-imu_data[5])*180/M_PI));
  pitch_angle=pitch_accel*(1-exp_filter_comp)+exp_filter_comp*(pitch_angle+delta_x_rotate);
  
  float roll_accel= (roll_calibration+(atan2f(imu_data[3],-imu_data[5])*180/M_PI));
  roll_angle=roll_accel*(1-exp_filter_comp)+exp_filter_comp*(roll_angle+delta_y_rotate);
  
}

int check_imu_limits()
{

   if(fabs(roll_angle)>45||fabs(pitch_angle)>45)
   {
     printf("imu limits, angle\n\r");
   }
   if(fabs(imu_data[0])>200||fabs(imu_data[1])>200||fabs(imu_data[2])>200)
   {
   
     printf("imu limits, rate\n\r");
   }

}
      
      
int setup_imu()
{
  wiringPiSetup ();
  
  
  //setup imu on I2C
  imu=wiringPiI2CSetup (0x68) ; //accel/gyro address
  
  if(imu==-1)
  {
    printf("-----cant connect to I2C device %d --------\n",imu);
    return -1;
  }
  else
  {
  
    printf("connected to i2c device %d\n",imu);
    printf("imu who am i is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
    uint8_t Ascale = AFS_2G;     // AFS_2G, AFS_4G, AFS_8G, AFS_16G
    uint8_t Gscale = GFS_500DPS; // GFS_250DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS
    
    
    //init imu
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x00);
    printf("                    \n\r");
    wiringPiI2CWriteReg8(imu,PWR_MGMT_1, 0x01);
    wiringPiI2CWriteReg8(imu, CONFIG, 0x00);  
    wiringPiI2CWriteReg8(imu, SMPLRT_DIV, 0x00); //0x04        
    int c=wiringPiI2CReadReg8(imu,  GYRO_CONFIG);
    wiringPiI2CWriteReg8(imu,  GYRO_CONFIG, c & ~0xE0);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c & ~0x18);
    wiringPiI2CWriteReg8(imu, GYRO_CONFIG, c | Gscale << 3);       
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG);
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG, c | Ascale << 3);      
    c=wiringPiI2CReadReg8(imu, ACCEL_CONFIG2);         
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2, c & ~0x0F); //
    wiringPiI2CWriteReg8(imu,  ACCEL_CONFIG2,  c | 0x00);
  }
  return 0;
}


