#include <arduino.h>

// Vector and quaternions
#include "C:\Users\bobby\Desktop\US College\ISU - Cardinal Mining club\Code\Tracked robot testbed\Individual classes\vector_datatype\vector_type.h"
#include "C:\Users\bobby\Desktop\US College\ISU - Cardinal Mining club\Code\Tracked robot testbed\Individual classes\vector_datatype\quaternion_type.h"

#ifndef trackVehicleModel_h
#define trackVehicleModel_h

template< const uint16_t  REV_COUNT ,
          const float*    XCM_A     ,
          const float*    XCM_B     , 
          const float*    DIAM_A    , 
          const float*    DIAM_B    ,
          const float*    LEN_X     ,
          const float*    LEN_Y     ,
          const float*    BODY_X    ,
          const float*    V_STAT    ,
          const float*    U_DYN     ,
          const float*    GRAVITY   > 
class trackVehicleModel {   
  private:
    static constexpr float V_CONST_A = PI*(*DIAM_A)/float(REV_COUNT); 
    static constexpr float V_CONST_B = PI*(*DIAM_B)/float(REV_COUNT); 

    float dt = 0;
    vec3_t wg;
    vec3_t vg;
    quat_t qt;

    //----------- Kinetic model:

    void odometry( float vel_L, float vel_R, vec3_t *v_out, float *w_out ) {        
      // Constants:  
      constexpr float INV_XCM       = 1/( *XCM_A - *XCM_B);
      constexpr float S_XCM         = ( *XCM_A + *XCM_B )*0.5;
      constexpr float FACTOR_UG     = (*U_DYN) * (*GRAVITY);
      constexpr float FACTOR_UG_SQ  = FACTOR_UG * FACTOR_UG;
      
      // Estimate velocities:
      vel_L *= V_CONST_A;
      vel_R *= V_CONST_B;
      float angVel = ( vel_R - vel_L )*INV_XCM;
      float vel    = ( vel_L + vel_R )*0.5 - angVel*S_XCM;  
    
        // Correct for slip:
      float accel    = angVel * vel;
      float accel_sq = accel * accel; 
      float mag      = 1.0/( FACTOR_UG_SQ + accel_sq );
    
        // Components
      float vx = vel * mag*accel_sq;
      float vy = vel * mag*FACTOR_UG_SQ; 
      
      // Outputs:     
      *v_out = { vx, vy, 0 };
      *w_out = angVel;  
    }

    //----------- Dynamic model:
   
    void trackForce( vec3_t force     , float inputL    , float inputR,    // Inputs
                     vec3_t vel       , float ang_vel   ,                  // Motion inputs
                     vec3_t *force_xy , float *torque_z ) {                // Outputs     
      //---- Constants:
      constexpr float LEVER_ARM   = ( *BODY_X + *LEN_X )*0.5;
      constexpr float BASE_LEN    = *BODY_X + 2*(*LEN_X);
      constexpr float MOMENT_INV  = 3.0/( BASE_LEN*BASE_LEN + (*LEN_Y)*(*LEN_Y) );
      constexpr float RADIUS_P2   = ( (*LEN_X)*(*LEN_X) + (*LEN_Y)*(*LEN_Y) )*0.25;    
      constexpr float V_STAT_P2   = (*V_STAT) * (*V_STAT);
      constexpr float V_STAT_P3   = (*V_STAT) * V_STAT_P2;

      //---- Track velocities:
      
      vec3_t velL = { vel.x, vel.y - ang_vel*LEVER_ARM - inputL*V_CONST_A };
      vec3_t velR = { vel.x, vel.y + ang_vel*LEVER_ARM - inputR*V_CONST_B };
      
      // Slip velocities:
      float v_term = ( ang_vel*ang_vel )*RADIUS_P2 + V_STAT_P2;  
           
      float slipL_p2 = v_term + velL.dot(velL);
      float slipR_p2 = v_term + velR.dot(velR);
      
      float slipL = sqrt(slipL_p2);
      float slipR = sqrt(slipR_p2);

      //---- Kinetic Friction:
      
      float norm = force.z > 0 ? 0 : -force.z*(*U_DYN);
      float norm_half = norm * 0.5;

      // Track force coefficients
      velL /= slipL;
      velR /= slipR;
      
      *force_xy = -( velL + velR )*norm_half;     
      *torque_z = -( ang_vel*( 0.25/slipL + 0.25/slipR )*RADIUS_P2 + ( velR.y - velL.y )*LEVER_ARM )*norm_half;
      *torque_z *= MOMENT_INV;

      //---- Static Friction:

      // Breakaway force:
      float v_gain = V_STAT_P3*( 0.5/(slipL*slipL_p2) + 0.5/(slipR*slipR_p2) );     // Velocity gain - Derivative of kinetic friction
      float fs_max = norm * v_gain;

      // Limit static friction
      vec3_t fxy = { force.x, force.y };
      vec3_t fs = -fxy;
      
      float fmag = fxy.mag();
      if( fmag > fs_max ) {
        fs *= fs_max/fmag;
      }

      // Limit total friction
      *force_xy += fs;
      
      fmag = force_xy->mag();
      if( fmag > norm ) {
        *force_xy *= norm/fmag;
      }
      *force_xy += fxy;
    }

    //----------- Velocity update: 

    void kineticUpdate( float inputL, float inputR ) {      
      // Estimate local velocity
      float wz;
      odometry( inputL, inputR, &vg, &wz );
      wg = { 0, 0, wz };
      
      // Update global velocities
      vg = qt.rotate( false, vg );
      wg = qt.rotate( false, wg );
    }
    
    void dynamicUpdate( float inputL, float inputR ) {
      // Get local velocity
      vg = qt.rotate( true, vg );

        // Local z-velocity is zero
      vg.z = 0;

      // Gravity in local frame
      vec3_t acc_ex = { 0, 0, -(*GRAVITY) };
      acc_ex = qt.rotate( true, acc_ex );
     
      // Estimate local velocity:
      vec3_t a;
      vec3_t v;
      float t;
      float w;

        // Apply Runga-Kutta Iterations
      dt *= 0.333333;     // 3 steps
    
      trackForce( acc_ex, inputL, inputR, vg, wg.z, &a, &t );  
      w = wg.z + 0.5*(t)*dt;
      v = vg + 0.5*(a)*dt;
      
      trackForce( acc_ex, inputL, inputR, v, w, &a, &t );  
      w = w + 0.5*(t)*dt;
      v = v + 0.5*(a)*dt;  
    
      trackForce( acc_ex, inputL, inputR, v, w, &a, &t ); 
      w = ( wg.z + 2*w + (t)*dt )*0.333333;
      v = ( vg + 2*v + (a)*dt )*0.333333;

      // Update global velocities
      wg = { 0, 0, w };
      wg = qt.rotate( false, wg ); 
      vg = qt.rotate( false, v );    
    }

  public:  
    //----------- Miscellaneous:
    void setup() {
      dt = float( micros() );
    }

      //-- State outputs:
      
    vec3_t angVel( const bool TO_LOCAL ) {
      if( TO_LOCAL ) {
        return qt.rotate( true, wg );
      } else {
        return wg;
      }
    }

    vec3_t vel( const bool TO_LOCAL ) {
      if( TO_LOCAL ) {
        return qt.rotate( true, vg );
      } else {
        return vg;
      }
    }
    
    quat_t getQuat() {
      return qt;
    }
    
    //----------- Kinetic model:

      // Simulated heading:

    void kinetic( float inputL, float inputR ) {
      // Set timer
      dt = ( float( micros() ) - dt )*1e-6;
      float last_time = float( micros() );

      // Rotate heading
      quat_t dq;
      dq.setRotation( true, wg*dt );
      qt *= dq;
      qt = qt.norm();

      // Update velocity 
      kineticUpdate( inputL, inputR );
      
      // Update timer
      dt = last_time;
    }

      // Given heading:

    void kinetic( quat_t q_in, float inputL, float inputR ) {
      // Update timer
      dt = float( micros() );

      // Set quaternion
      qt = q_in;
    
      // Update velocity 
      kineticUpdate( inputL, inputR );
    }

    //----------- Dynamic model:

      // Simulated heading:
      
    void dynamic( float inputL, float inputR ) {
      // Set timer
      dt = ( float(micros()) - dt )*1e-6;
      float last_time = float(micros());

      // Rotate heading
      quat_t dq;
      dq.setRotation( true, wg*dt );
      qt *= dq; 
      qt = qt.norm();

      // Update velocity 
      wg = qt.rotate( true, wg );
      dynamicUpdate( inputL, inputR );
      
      // Update timer
      dt = last_time;
    }

      // Given heading:
      
    void dynamic( quat_t q_in, float wz, float inputL, float inputR ) {
      // Set timer
      dt = ( float(micros()) - dt )*1e-6;
      float last_time = float(micros());

      // Set quaternion
      qt = q_in;

      // Update velocity
      wg = { 0, 0, wz };
      dynamicUpdate( inputL, inputR );

      // Update timer
      dt = last_time;
    }
};

#endif
