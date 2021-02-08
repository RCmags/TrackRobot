#include <arduino.h>

// Vector type
#include "C:\Users\bobby\Desktop\US College\ISU - Cardinal Mining club\Code\Tracked robot testbed\Individual classes\vector_datatype\vector_type.h"
#include "C:\Users\bobby\Desktop\US College\ISU - Cardinal Mining club\Code\Tracked robot testbed\Individual classes\vector_datatype\quaternion_type.h"

#ifndef accIntegral_h
#define accIntegral_h

template <const float* SD_ACC, const float* SD_VEL, const float* GRAVITY>
class accIntegral {
  private:
    vec3_t vel          = {0,0,0};      
    vec3_t acc_mean     = {0,0,0};  
    vec3_t acc1         = {0,0,0};
    float var_vel       = 0;
    float var_acc       = 0;
    uint32_t last_time  = 0;

  public:  
    void setup() {
      last_time = micros(); 
    }

    vec3_t removeCentrifugal( quat_t qt, vec3_t wl, vec3_t accl ) {
      // Centrifugal in global frame
      vec3_t wg = qt.rotate( false, wl );
      vec3_t acc_cf = wg.cross( vel );

      // Remove in local frame
      acc_cf = qt.rotate( true, acc_cf );
      return accl - acc_cf;
    }

    vec3_t update( quat_t qt, vec3_t acc_in, vec3_t vel_t ) { 
      constexpr float VAR_ACC   = (*SD_ACC)*(*SD_ACC);
      constexpr float VAR_VEL   = (*SD_VEL)*(*SD_VEL);
      constexpr float VAR_COMB  = VAR_ACC*VAR_VEL;

      //-------- Remove acceleration bias:

        // Global gravity vector
      vec3_t acc = qt.rotate( false, acc_in );
      acc.z -= *GRAVITY;
      
        // Update variance
      float acc_mag = acc.dot(acc);
      float gain_acc = VAR_ACC/(var_acc + VAR_ACC);
 
      var_acc += acc_mag - 0.5*(1 - gain_acc)*var_acc;        // Halved decay to smooth gain transition

        // Update mean 
      float dt = float( micros() - last_time )*1e-6;
      last_time = micros();
               
      acc -= acc_mean;
      acc_mean += acc * gain_acc * dt;
    
        // Deadband on acceleration  
      if( acc_mag < VAR_ACC ) {
        acc = {0,0,0}; 
      } else {
        acc *= 1 - sqrt(VAR_ACC/acc_mag); 
      }
      
      //-------- Integrate acceleration:
      vec3_t dv = vel_t - vel;
      
        // Update Variance
      float dv_mag = dv.dot(dv);
      float gain_vel = VAR_COMB/(var_vel*var_acc + VAR_COMB);
      
      var_vel += dv_mag - 0.5*(1 - gain_vel)*var_vel;         // Halved decay to smooth gain transition

        // Integrate 
      vel += 0.5*(acc + acc1)*dt + dv*gain_vel;
      acc1 = acc;
      
      return vel; 
    }
};

#endif
