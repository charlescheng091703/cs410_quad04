// Week 6
// Author: Charles Cheng and Sushma

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

// To transfer files from local to remote, run
// pscp -pw raspberry week6.cpp pi@192.168.0.1:/home/pi/flight_controller

// To transfer files from remote to local, run
// pscp -pw raspberry pi@192.168.0.1:/home/pi/flight_controller/resource/yaw.csv /home/charles/cs410_quad04

// To compile the code, run 
// gcc -o week6 week6.cpp -lwiringPi -lm

// To start the client to the server publishing controller inputs, run 
// ./client 192.168.0.26

#define frequency 25000000.0
#define CONFIG           0x1A
#define SMPLRT_DIV       0x19
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define A_COMP_FILTER    0.004
#define ANG_VEL_LIMIT    500
#define ROLL_LIMIT       45
#define PITCH_LIMIT      45
#define PWM_MAX 1950
#define PWM_MIN 1000
#define frequency 25000000.0
#define LED0 0x6			
#define LED0_ON_L 0x6		
#define LED0_ON_H 0x7		
#define LED0_OFF_L 0x8		
#define LED0_OFF_H 0x9		
#define LED_MULTIPLYER 4
#define P_ROLL 5.0
#define I_ROLL 0.04
#define D_ROLL 0.5
#define P_PITCH 8.0
#define I_PITCH 0.02 
#define D_PITCH 0.7
#define P_YAW 2.0
#define JOY_THRUST_MAX 1550 // low: 1580 // high: 1550
#define NTRL_Thrust 1480 // low: 1500 // high: 1475
#define JOY_THRUST_MIN 1400
#define I_CAP 100.0
#define JOY_PITCH 20 // divide by 2 = max/min desired pitch angle // fine: 10
#define JOY_ROLL 20 //./f divide by 2 = max/min desired pitch angle // fine: 10
#define JOY_YAW 80 // divide by 2 = max/min desired yaw 

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
void write_to_csv();
void setup_keyboard();
void trap(int);
void get_joystick();
void init_pwm();
void init_motor(uint8_t);
void set_PWM(uint8_t, float);
void pid_update();
void kill_motors();

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
float imu_diff;
struct timespec te;
float yaw=0;
float pitch_accel=0;
float roll_accel=0;
float roll_angle=0;
float pitch_angle=0;
float roll_gyro=0;
float pitch_gyro=0;
float real_time=0;
float heartbeat_time=0; // time since last heartbeat
int last_heartbeat=0;
FILE *fpt;
float motor_cntrl0;
float motor_cntrl1;
float motor_cntrl2;
float motor_cntrl3;
struct Keyboard {
  int keypress;
  int pitch;
  int roll;
  int yaw;
  int thrust;
  int sequence_num; // heartbeat
};
Keyboard* shared_memory; 
int run_program=1;
int pwm;
float intg_pitch=0;
float intg_roll=0;
float Thrust=NTRL_Thrust;
float desired_pitch=0.0;
float desired_roll=0.0;
float desired_yaw=0.0;

int main (int argc, char *argv[])
{
    init_pwm();
    init_motor(0);
    init_motor(1);
    init_motor(2);
    init_motor(3);
    delay(1000);
    setup_imu();
    calibrate_imu();
    
    // Clear file contents
    fclose(fopen("resource/motor_pitch.csv", "w"));
    fclose(fopen("resource/motor_pitch2.csv", "w"));
    fclose(fopen("resource/yaw.csv", "w"));
    
    setup_keyboard();
    signal(SIGINT, &trap);
    
    // Wait for user to unpause
    Keyboard keyboard=*shared_memory;
    while (keyboard.keypress != 34) {
      keyboard=*shared_memory;
    }
    printf("Unpaused.\n\r");
    timespec_get(&te,TIME_UTC);
    time_prev=te.tv_nsec;

    while(run_program==1)
    {
      read_imu();      
      update_filter();   
      write_to_csv();
      //printf("%f\t%f\n", desired_pitch, pitch_angle);
      get_joystick();
      pid_update();
    }
    kill_motors();

    return 0;
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
  roll_accel = roll_calibration+atan2(ax, -az)/M_PI*180;
  pitch_accel = pitch_calibration+atan2(ay, -az)/M_PI*180;
}

void update_filter()
{
  //get current time in nanoseconds
  timespec_get(&te,TIME_UTC);
  time_curr=te.tv_nsec;
  //compute time since last execution
  imu_diff=time_curr-time_prev;           
  
  //check for rollover
  if(imu_diff<=0)
  {
    imu_diff+=1000000000;
  }
  //convert to seconds
  imu_diff=imu_diff/1000000000;
  time_prev=time_curr;
  real_time += imu_diff;
  
  //comp. filter for roll, pitch here: 
  roll_gyro += imu_data[1]*imu_diff;
  pitch_gyro += imu_data[0]*imu_diff;
  roll_angle = roll_accel*A_COMP_FILTER + (1-A_COMP_FILTER)*(roll_angle+imu_data[1]*imu_diff);
  pitch_angle = pitch_accel*A_COMP_FILTER + (1-A_COMP_FILTER)*(pitch_angle+imu_data[0]*imu_diff);
}

void write_to_csv()
{
  // fpt = fopen("resource/roll.csv", "a");
  // fprintf(fpt, "%f, %f, %f, %f\n", real_time, roll_angle, roll_accel, roll_gyro);
  // fclose(fpt);
  fpt = fopen("resource/yaw.csv", "a");
  fprintf(fpt, "%f, %f, %f, %f, %f\n", motor_cntrl0, motor_cntrl1, motor_cntrl2, motor_cntrl3, imu_data[2]);
  fclose(fpt);

  fpt = fopen("resource/motor_pitch.csv", "a");
  fprintf(fpt, "%f, %f, %f, %f, %f, %f, %f\n", real_time, motor_cntrl0, motor_cntrl1, motor_cntrl2, motor_cntrl3, pitch_angle, imu_data[0]);
  fclose(fpt);

  fpt = fopen("resource/motor_pitch2.csv", "a");
  fprintf(fpt, "%f, %f, %f, %f, %f, %f, %f, %f\n", real_time, motor_cntrl0, motor_cntrl1, motor_cntrl2, motor_cntrl3, pitch_angle, pitch_accel, pitch_gyro);
  fclose(fpt);
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
  // printf ("shared memory attached at address %p\n", shared_memory); 
  /* Determine the segment's size. */ 
  shmctl (segment_id, IPC_STAT, &shmbuffer); 
  segment_size = shmbuffer.shm_segsz; 
  // printf ("segment size: %d\n", segment_size); 
  /* Write a string to the shared memory segment.  */ 
  //sprintf (shared_memory, "test!!!!."); 
}

// when cntrl+c pressed, kill motors
void trap(int signal)
{
   printf("ending program\n\r");
   run_program=0;
}

void get_joystick()
{
  Keyboard keyboard=*shared_memory;
  if (abs(imu_data[0]) > ANG_VEL_LIMIT || abs(imu_data[1]) > ANG_VEL_LIMIT || abs(imu_data[2]) > ANG_VEL_LIMIT) {
    run_program = 0;
    printf("Ending program. Gyro rate exceeded limit.\n\r");
  }
  else if (abs(roll_angle) > ROLL_LIMIT) {
    run_program = 0;
    printf("Ending program. Roll exceeded limit.\n\r");
  }
  else if (abs(pitch_angle) > PITCH_LIMIT) {
    run_program = 0;
    printf("Ending program. Pitch exceeded limit.\n\r");
  }
  else if (keyboard.sequence_num != last_heartbeat) {
    last_heartbeat = keyboard.sequence_num;
    heartbeat_time = 0;
  }
  else if (heartbeat_time > 0.3) {
    run_program = 0;
    printf("Ending program. Joystick timeout.\n\r");
  }

  if (keyboard.keypress == 32) {
    run_program = 0;
    printf("Ending program. Killed.\n\r");
  }
  else if (keyboard.keypress == 33) {
    kill_motors();
    printf("Paused.\n\r");
    while (keyboard.keypress != 34) { // do nothing if unpaused not pressed
      keyboard=*shared_memory;
      if (keyboard.keypress == 35) { // calibrate if button pressed when paused
        calibrate_imu();
        printf("Calibrated.\n\r");
      }
    }
    printf("Unpaused.\n\r");
    timespec_get(&te,TIME_UTC);
    time_prev=te.tv_nsec;
  }
  else if (keyboard.keypress == 35) {
    printf("Can calibrate only when paused.\n\r");
  }

  desired_pitch = keyboard.pitch/224.0*JOY_PITCH-0.57143*JOY_PITCH;
  desired_roll = -keyboard.roll/224.0*JOY_ROLL+0.57143*JOY_ROLL;
  if (keyboard.thrust > 128) {
    Thrust = NTRL_Thrust + keyboard.thrust/96.0*(JOY_THRUST_MAX-NTRL_Thrust) - 1.33333*(JOY_THRUST_MAX-NTRL_Thrust);
  }
  else {
    Thrust = NTRL_Thrust + (keyboard.thrust/128.0-1.0)*(NTRL_Thrust-JOY_THRUST_MIN);
  }
  
  desired_yaw = keyboard.yaw/224.0*JOY_YAW-0.57143*JOY_YAW;
  heartbeat_time += imu_diff;
}

void init_pwm()
{
    pwm=wiringPiI2CSetup (0x40);
    if(pwm==-1)
    {
      printf("-----cant connect to I2C device %d --------\n",pwm);
    }
    else
    {
      float freq =400.0*.95;
      float prescaleval = 25000000;
      prescaleval /= 4096;
      prescaleval /= freq;
      prescaleval -= 1;
      uint8_t prescale = floor(prescaleval+0.5);
      int settings = wiringPiI2CReadReg8(pwm, 0x00) & 0x7F;
      int sleep	= settings | 0x10;
      int wake 	= settings & 0xef;
      int restart = wake | 0x80;
      wiringPiI2CWriteReg8(pwm, 0x00, sleep);
      wiringPiI2CWriteReg8(pwm, 0xfe, prescale);
      wiringPiI2CWriteReg8(pwm, 0x00, wake);
      delay(10);
      wiringPiI2CWriteReg8(pwm, 0x00, restart|0x20);
    }
}

void init_motor(uint8_t channel)
{
	int on_value=0;

	int time_on_us=900;
	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	time_on_us=1200;
	off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);

	time_on_us=1000;
	off_value=round((time_on_us*4096.f)/(1000000.f/400.0));

	wiringPiI2CWriteReg8(pwm, LED0_ON_L + LED_MULTIPLYER * channel, on_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_ON_H + LED_MULTIPLYER * channel, on_value >> 8);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_L + LED_MULTIPLYER * channel, off_value & 0xFF);
	wiringPiI2CWriteReg8(pwm, LED0_OFF_H + LED_MULTIPLYER * channel, off_value >> 8);
	delay(100);
}

void set_PWM( uint8_t channel, float time_on_us)
{
  if(run_program==1)
  {
    if(time_on_us>PWM_MAX)
    {
      time_on_us=PWM_MAX;
    }
    else if(time_on_us<PWM_MIN)
    {
      time_on_us=PWM_MIN;
    }
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
  else
  {  
    time_on_us=1000;   
  	uint16_t off_value=round((time_on_us*4096.f)/(1000000.f/400.0));
  	wiringPiI2CWriteReg16(pwm, LED0_OFF_L + LED_MULTIPLYER * channel,off_value);
  }
}

void kill_motors() {
  set_PWM(0, 0); 
  set_PWM(3, 0); 
  set_PWM(1, 0); 
  set_PWM(2, 0); 
}

void pid_update()
{
  float pitch_error = pitch_angle-desired_pitch;
  float roll_error = roll_angle-desired_roll;
  float yaw_error = imu_data[2]-desired_yaw;
  intg_pitch += pitch_error*I_PITCH;
  intg_roll += roll_error*I_ROLL;

  // upper and lower limits on integrated pitch and roll
  if (intg_pitch > I_CAP) {
    intg_pitch = I_CAP;
  }
  else if (intg_pitch < -I_CAP) {
    intg_pitch = -I_CAP;
  }
  if (intg_roll > I_CAP) {
    intg_roll = I_CAP;
  }
  else if (intg_roll < -I_CAP) {
    intg_roll = -I_CAP;
  }
  
  motor_cntrl0 = Thrust+pitch_error*P_PITCH-roll_error*P_ROLL+intg_pitch-intg_roll+imu_data[0]*D_PITCH-imu_data[1]*D_ROLL-yaw_error*P_YAW;
  motor_cntrl1 = Thrust-pitch_error*P_PITCH-roll_error*P_ROLL-intg_pitch-intg_roll-imu_data[0]*D_PITCH-imu_data[1]*D_ROLL+yaw_error*P_YAW;
  motor_cntrl2 = Thrust-pitch_error*P_PITCH+roll_error*P_ROLL-intg_pitch+intg_roll-imu_data[0]*D_PITCH+imu_data[1]*D_ROLL-yaw_error*P_YAW;
  motor_cntrl3 = Thrust+pitch_error*P_PITCH+roll_error*P_ROLL+intg_pitch+intg_roll+imu_data[0]*D_PITCH+imu_data[1]*D_ROLL+yaw_error*P_YAW;
  set_PWM(0, motor_cntrl0); 
  set_PWM(1, motor_cntrl1); 
  set_PWM(2, motor_cntrl2); 
  set_PWM(3, motor_cntrl3); 
}