#include <arduino.h>

#define PWM_MAX 255

#ifndef rateControl_h
#define rateControl_h

template <uint8_t PIN_MOTOR_1   , 
          uint8_t PIN_MOTOR_2   ,
          uint8_t BAND_IN       , 
          uint8_t BAND_OUT      ,
          const float *KP       ,  
          const float *KD       , 
          const float *F_STATIC >
class rateControl {
  private:   
    float x_target = 0;
    int x1 = 0; 
    int x2 = 0; 
    int x3 = 0;
    int x4 = 0;
    int x5 = 0;

    int8_t band = BAND_IN;
    float dt = 0;
    float vel = 0;

    // H-bridge control
    void torque( int input ) {

      // Prevent overflow
      if( input >= PWM_MAX ) {
        input = PWM_MAX;
      } else if ( input <= -PWM_MAX ) {
        input = -PWM_MAX;
      }

      // Set motor direction
      if( input >= 0 ) {
        digitalWrite( PIN_MOTOR_2, LOW );
        analogWrite( PIN_MOTOR_1, input );
      } else {
        digitalWrite( PIN_MOTOR_1, LOW );
        analogWrite( PIN_MOTOR_2, -input );
      }
    }

    // Derivative approximation - 6 point least squares
    float dxdt( int x ) {
      // least squares - 4 point
      float dx = float( 5*(x - x5) + 3*(x4 - x3) + (x2 - x1) )/35.0;

      // Store past values
      x5 = x4;
      x4 = x3;
      x3 = x2;
      x2 = x1;
      x1 = x;

      return 1.5*float(dx)/dt;
    }
    
  public:
    void setup() {
      // Timer
      dt = micros();

      // H-bridge pins
      pinMode( PIN_MOTOR_1, OUTPUT );
      pinMode( PIN_MOTOR_2, OUTPUT );
      digitalWrite( PIN_MOTOR_1, LOW );
      digitalWrite( PIN_MOTOR_2, LOW );
    }

    float rate() {
      return vel;
    }

    void update( float rate_t, int pos ) {  
      // Update timer
      float curr_time = micros(); 
      dt = ( curr_time - dt )*1e-6;
    
      // Measured velocity
      vel = dxdt( pos );

      // Update target position
      x_target += ( rate_t - vel )*dt;

      // Prevent integral windup
      if( x_target > PWM_MAX ) {
        x_target = PWM_MAX;
      } else if ( x_target < -PWM_MAX ) {
        x_target = -PWM_MAX;
      }

      // Truncate position
      int dx = int( x_target );
      
      // Apply deadband with hysteresis
      if( dx >= band ) {
        dx = dx - band;
        if( band == BAND_OUT ) {
          band = BAND_IN;
        }
      } else if ( dx <= -band ) {
        dx = dx + band;
        if( band == BAND_OUT ) {
          band = BAND_IN;
        }
      } else {
        dx = 0;
        if( band == BAND_IN ) {
          band = BAND_OUT;
        }
      }
   
      // Compensate for static friction
      constexpr float SLOPE = 1 - (*F_STATIC)/PWM_MAX;
      constexpr float BIAS  = (*F_STATIC) - (*KP);

      float output = float(dx)*(*KP);
      //
      if( output > 0 ) {
        output = output*SLOPE + BIAS;
      } else if ( output < 0 ) {
        output = output*SLOPE - BIAS;
      } 
      
      // Apply PD controller      
      torque( int(output - (*KD)*vel) );

      // Store current time
      dt = curr_time;
    }
};

#endif
