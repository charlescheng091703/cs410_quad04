// Week 1
// Author: Charles Cheng

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

// To compile the code, run 
// gcc -o week1 week1_student.cpp -lwiringPi -lm

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

//global variables
int imu;
float x_gyro_calibration=0;
float y_gyro_calibration=0;
float z_gyro_calibration=0;
float roll_calibration=0;
float pitch_calibration=0;
float accel_z_calibration=0;
float imu_data[6]; // gyro xyz, accel xyz
long time_curr;
long time_prev;
struct timespec te;
float yaw=0;
float pitch_angle=0;
float roll_angle=0;
 
int main (int argc, char *argv[])
{
    setup_imu();
    calibrate_imu();

    while(1)
    {
      read_imu();      
      update_filter();   
      printf("vx:%10.5f\tvy:%10.5f\tvz:%10.5f\tpitch:%10.5f\troll:%10.5f\n", imu_data[0], imu_data[1], imu_data[2], pitch_angle, roll_angle);
    }
}

// Helper function read_raw
// takes in address as input
// outputs accel/gyro raw values 
int read_raw(int address)
{
  int vh = wiringPiI2CReadReg8(imu, address);
  int vl = wiringPiI2CReadReg8(imu, address+1);
  int vw = (((vh<<8)&0xff00)|(vl&0x00ff))&0xffff;
  if (vw > 0x8000)
  {
    vw = vw^0xffff;
    vw = -vw-1;
  }
  return vw;
}

// Assumes the initial position of the quad 
// is at rest, level on a flat surface
void calibrate_imu()
{
  int vx_sum, vy_sum, vz_sum = 0;
  float ax_sum, ay_sum, az_sum = 0;

  // Sample size of 1000 
  // Taking average of accel and gyro raw values
  for (int i = 0; i < 1000; i ++) {
    ax_sum += read_raw(59); // accel x 
    ay_sum += read_raw(61); // accel y 
    az_sum += read_raw(63); // accel z 
    vx_sum += read_raw(67); // gyro x 
    vy_sum += read_raw(69); // gyro y 
    vz_sum += read_raw(71); // gyro z 
  }

  x_gyro_calibration = -vx_sum/32767.0*0.5;
  y_gyro_calibration = -vy_sum/32767.0*0.5;
  z_gyro_calibration = -vz_sum/32767.0*0.5;
  
  float ax_avg = ax_sum/32767.0*2/1000;
  float ay_avg = ay_sum/32767.0*2/1000;
  float az_avg = az_sum/32767.0*2/1000;

  roll_calibration = -atan2(ax_avg, -az_avg)/M_PI*180;
  pitch_calibration = -atan2(ay_avg, -az_avg)/M_PI*180;

  accel_z_calibration = -1.0-az_avg; // gravity points downwards, -z axis

  printf("Calibration complete: %f %f %f %f %f %f\n\r",x_gyro_calibration,y_gyro_calibration,z_gyro_calibration,roll_calibration,pitch_calibration,accel_z_calibration);
}

void read_imu()
{
  int vw;
  float ax, ay, az;

  vw = read_raw(59); // accel x    
  imu_data[3]=vw/32767.0*2; // convert vw from raw values to "g's"
  
  vw = read_raw(61); // accel y 
  imu_data[4]=vw/32767.0*2; // convert vw from raw values to "g's"
  
  vw = read_raw(63); // accel z
  imu_data[5]=accel_z_calibration+vw/32767.0*2; // convert vw from raw values to g's
  
  vw = read_raw(67); // gyro x    
  imu_data[0]=-(x_gyro_calibration+vw/32767.0*500); // convert vw from raw values to degrees/second
  
  vw = read_raw(69); // gyro y      
  imu_data[1]=y_gyro_calibration+vw/32767.0*500; // convert vw from raw values to degrees/second
  
  vw = read_raw(71); // gyro z            
  imu_data[2]=z_gyro_calibration+vw/32767.0*500; // convert vw from raw values to degrees/second

  ax = imu_data[3];
  ay = imu_data[4];
  az = imu_data[5];
  roll_angle = roll_calibration+atan2(ax, -az)/M_PI*180;
  pitch_angle = pitch_calibration+atan2(ay, -az)/M_PI*180;
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
  
  //comp. filter for roll, pitch here: 
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
  
    printf("Connected to I2C device %d\n",imu);
    printf("IMU who am I is %d \n",wiringPiI2CReadReg8(imu,0x75));
    
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