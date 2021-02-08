//======================================================
//                  External classes
//======================================================

//----------------  Motor rate control  ----------------
#include "src/rateControl.h"

#define         MOTOR_L_PIN_1   9
#define         MOTOR_L_PIN_2   8
#define         MOTOR_R_PIN_1   11
#define         MOTOR_R_PIN_2   10
#define         BAND_IN         1
#define         BAND_OUT        2
constexpr float KP          =   12;
constexpr float KD          =   1;
constexpr float F_STATIC    =   50;

rateControl <MOTOR_L_PIN_1, MOTOR_L_PIN_2, BAND_IN, BAND_OUT, &KP, &KD, &F_STATIC> motorL;
rateControl <MOTOR_R_PIN_1, MOTOR_R_PIN_2, BAND_IN, BAND_OUT, &KP, &KD, &F_STATIC> motorR;


//----------------    Motor encoder     ----------------
#include <Encoder.h>

#define         ENC_L_PIN_1     18
#define         ENC_L_PIN_2     19 
#define         ENC_R_PIN_1     2
#define         ENC_R_PIN_2     3

Encoder encoderL( ENC_L_PIN_1, ENC_L_PIN_2 );
Encoder encoderR( ENC_R_PIN_1, ENC_R_PIN_2 );


//----------------     Vehicle model    ----------------
#include "src/trackVehicleModel.h"
//                                          Unit                           
#define         REV_COUNT       300    
constexpr float XCM_A       =   115;        // mm
constexpr float XCM_B       =   -115;       // mm
constexpr float DIAM_A      =   59.0;       // mm
constexpr float DIAM_B      =   59.0;       // mm
constexpr float TRACK_X     =   45;         // mm
constexpr float TRACK_Y     =   185;        // mm
constexpr float BODY_X      =   100;        // mm
constexpr float V_STAT      =   10;         // mm/s
constexpr float U_DYN       =   0.66;
constexpr float GRAVITY     =   9.81e3;     // mm/s^2

trackVehicleModel <REV_COUNT, &XCM_A, &XCM_B, &DIAM_A, &DIAM_B, 
                   &TRACK_X, &TRACK_Y, &BODY_X, &V_STAT, &U_DYN, &GRAVITY> model; 
                  
                  
//----------------    MPU6050 sensor    ----------------
#include "src/basicMPU6050.h"

#define         AX_OFFSET       552
#define         AY_OFFSET       -241
#define         AZ_OFFSET       -2880
constexpr float AX_SCALE    =   1.00457;
constexpr float AY_SCALE    =   0.99170;
constexpr float AZ_SCALE    =   0.98317;
constexpr float GX_SCALE    =   0.99764;
constexpr float GY_SCALE    =   1.0;
constexpr float GZ_SCALE    =   1.01037;

basicMPU6050<6, 1, 0, 
             AX_OFFSET, AY_OFFSET, AZ_OFFSET, 
             &AX_SCALE, &AY_SCALE, &AZ_SCALE,
             &GX_SCALE, &GY_SCALE, &GZ_SCALE> mpu;

            
//----------------  IMU Sensor Fusion   ----------------
#include "src/imuCompFilter.h"

constexpr float GAIN_GYRO   =   0.1;

imuCompFilter <&GAIN_GYRO> imu;


//----------------  Double integration  ----------------
#include "src/accIntegral.h"
//                                          Unit
constexpr float SD_ACC      =   400;        // mm/s^2
constexpr float SD_VEL      =   40;         // mm/s

accIntegral <&SD_ACC, &SD_VEL, &GRAVITY> accInt;


//----------------     Optical flow     ----------------
#include "src/camMotion.h"
//                                          Unit
#define         LEFT_RESET      45 
#define         LEFT_CS         47 
#define         RIGHT_RESET     44 
#define         RIGHT_CS        46
constexpr float DISTANCE    =   60;         // mm
constexpr float PIXEL_SCALE =   50/100.0;   // pixel/mm
constexpr float LEFT_THETA  =   -90;        // deg
constexpr float RIGHT_THETA =   -270;       // deg
#define         RESOLUTION      false

camMotion <LEFT_RESET, LEFT_CS, RIGHT_RESET, RIGHT_CS, 
           &DISTANCE, &PIXEL_SCALE, &LEFT_THETA, &RIGHT_THETA, RESOLUTION> opflow;

           
//---------------- Bluetooth waypoints  ----------------
#define SERIAL_3
#include "src/BLmodule.h"

#define         BAUD_RATE       115200
#define         CHAR_MAX        30
#define         POINT_MAX       5

BLmodule <BAUD_RATE, CHAR_MAX, POINT_MAX> BLcontrol;


//======================================================
//              Position configuration
//======================================================

//------ States
#define DISABLE                 0
#define ENABLE                  1
#define KINETIC                 2
#define DYNAMIC                 3

//------ Settings
#define MODEL                   DYNAMIC
#define GYRO                    ENABLE
#define OPFLOW                  ENABLE
#define ACCEL_INT               ENABLE

//----------------   Global variables   ----------------
uint32_t last_time = 0;
uint8_t  p_step    = 0;         // Current waypoint step 
vec3_t x   = {0,0,0};           // Position
vec3_t dxn = {0,1,0};           // Normalized position error [Heading vector]
vec3_t vlp = {0,0,0};           // Low passed velocity
quat_t qt  = {1,0,0,0};         // Heading quaternion


//======================================================
//                    Main Program
//======================================================

void setup() {  
  // Initialize waypoints
  BLcontrol.setup();

  // Calibrate IMU
  mpu.setup();
  mpu.setBias();
  imu.setup( mpu.ax(), mpu.ay(), mpu.az() );    // Must precede other variables for stability -> delay causes large stepsize
  
  // Enable pins and initialize timers
  motorL.setup();
  motorR.setup();
  model.setup();
  accInt.setup();
  opflow.setup();
  last_time = micros();
}

void loop() {
  //------------------------------------------------------
  //                Position estimation
  //------------------------------------------------------
  
  //---- Update timer 

  float dt = float( micros() - last_time )*1e-6;
  last_time = micros();
  
  //---- Use IMU to estimate vehicle orientation

  #if GYRO == ENABLE
    // Read IMU measurements
    vec3_t acc = { mpu.ax(), mpu.ay(), mpu.az() };
    vec3_t wl = { mpu.gx(), mpu.gy(), mpu.gz() };

    // Get Fusion quaternion
    float q_imu[4];
    imu.getQuat( q_imu );
    
    qt = {q_imu};
  #endif

  //---- Estimate velocity via model of vehicle

  // Velocity via steady state approximation
  #if MODEL == KINETIC
  
    #if GYRO == DISSABLE
      model.kinetic( motorL.rate(), motorR.rate() ); 
      qt = model.getQuat();
    
    #else
      model.kinetic( qt, motorL.rate(), motorR.rate() );
    #endif  
  
  // Velocity by integrating acceleration
  #elif MODEL == DYNAMIC
    
    #if GYRO == DISSABLE 
      model.dynamic( motorL.rate(), motorR.rate() );
      qt = model.getQuat();
    
    #else
      model.dynamic( qt, wl.z, motorL.rate(), motorR.rate() );
    #endif
  
  #endif

  // Velocity in global frame
  vec3_t vm = model.vel(false);
    
  //---- Measure velocity via optical flow
  
  #if OPFLOW == ENABLE
  
    // Retrieve local velocity
    vec3_t wf;
    vec3_t vf;
    opflow.velocity( &vf.x, &vf.y, &wf.z ); 

    // Update heading quaternion
    #if MODEL == DISSABLE || GYRO == ENABLE
      quat_t dq;
      dq.setRotation( true, wf*dt );
      qt *= dq;
    #endif
    
    // Velocity in global frame
    vf = qt.rotate( false, vf );
  #else 
    vec3_t vf = vm; 
  #endif

  //---- Fuse model velocity estimates with optical flow [Complementary filter]

  // High pass on model, low pass on optical flow
  #if OPFLOW == ENABLE && MODEL != DISABLE
    
    constexpr float GAIN = 0.25;
    vlp += (vm - vf - vlp)*GAIN;
    vec3_t vhp = vm - vlp; 
  
  #else
    vec3_t vhp = vf;
  #endif

  //---- Estimate velocity via IMU double integration

  #if ACCEL_INT == ENABLE && GYRO == ENABLE 
    // Scale acceleration
    acc *= GRAVITY;
    
    // Integrate acceleration
    vec3_t v = accInt.update( qt, acc, vhp );
    
    // Correct acceleration via velocity estimate
    acc = accInt.removeCentrifugal( qt, wl, acc ); 
  #else 
    vec3_t v = vhp;
  #endif
  
  //---- Update position and heading 
    
  x += v*dt;
  
  #if GYRO == ENABLE
    imu.update( wl.x, wl.y, wl.z, acc.x, acc.y, acc.z );
    mpu.updateBias();
  #endif

  //------------------------------------------------------
  //                    Waypoint control
  //------------------------------------------------------
  
  //---- Update waypoints
  
  BLcontrol.checkSerial();
  
  // Retrieve waypoint
  float xt[2];
  BLcontrol.getStack(&p_step, xt);
  
  //---- Select target position
  
  constexpr float RAD_MIN = 15;
  
  // Position error
  vec3_t dx = { xt[0] - x.x , 
                xt[1] - x.y };
  float dx_mag = dx.mag(); 

  // Update heading vector if far from waypoint
  if( dx_mag > RAD_MIN ) {
    dxn = dx/dx_mag;
  } else {
    // Select new waypoint if close to waypoint
    uint8_t len = BLcontrol.stackSize();
    len = len < 1 ? 1 : len;
    p_step = p_step < len-1 ? p_step + 1 : 0; 
  }

  //---- Define steering inputs

  // Local heading and displacement errors
  vec3_t vy = qt.axisY(false);
  vec3_t da = vy.cross( dxn );

  float et = da.dot( qt.axisZ(false) );
  float el = dx.dot( vy );

  // Proportional controller on turn input
  constexpr float TURN_MAX = 100;
  constexpr float GAIN_ANG = 300;
  
  // Saturate 
  float turn = et * GAIN_ANG; 
  turn = turn >  TURN_MAX ?  TURN_MAX :
         turn < -TURN_MAX ? -TURN_MAX :
                                 turn ;
                                 
  // Propotional controller on throttle input
  constexpr float DRIVE_MAX = 120;
  constexpr float GAIN_DIR  = 4;
  constexpr float ANG_LIMIT = 0.25;

  // Saturate   
  float drive = abs(et) > ANG_LIMIT ? 0 : (el * GAIN_DIR);
  drive = drive >  DRIVE_MAX ?  DRIVE_MAX :
          drive < -DRIVE_MAX ? -DRIVE_MAX :
                                    drive ;
                                    
  //---- Update motor velocity

  motorL.update( drive - turn, encoderL.read() );
  motorR.update( drive + turn, encoderR.read() );
}
