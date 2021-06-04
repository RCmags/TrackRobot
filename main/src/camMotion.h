#include <arduino.h>
#include <ADNS3080.h>

#ifndef camMotion_h
#define camMotion_h

#define SETUP_TIME    1e5

template < const int    LEFT_RESET  , const int    LEFT_CS      , 
           const int    RIGHT_RESET , const int    RIGHT_CS     ,
           const float *DISTANCE    , const float *PIXEL_SCALE  ,
           const float *LEFT_THETA  , const float *RIGHT_THETA  ,
           const bool   RESOLUTION  >
class camMotion {

  private:
    ADNS3080 <LEFT_RESET, LEFT_CS> sensor_L;
    ADNS3080 <RIGHT_RESET, RIGHT_CS> sensor_R;
    uint32_t last_time;

    // Rotate frame of reference:
   
    template <const float *ANG>
    void rotate( float *vx, float *vy ) {
      // Common terms:
      constexpr float RAD = (*ANG) * PI/180.0;
      constexpr float SIN = sin(RAD);
      constexpr float COS = cos(RAD);

      // Components:
      float _vx = *vx;
      *vx = (_vx)*COS - (*vy)*SIN;
      *vy = (*vy)*COS + (_vx)*SIN;
    }

  public:
    // Initialize sensors
    void setup() {
      sensor_L.setup( false, RESOLUTION );
      sensor_R.setup( false, RESOLUTION );    

      // Update and reset displacement registers
      last_time = micros();
      float dx, dy, dang;
      
      while( micros() - last_time < SETUP_TIME ) {
        displacement( &dx, &dy, &dang ); 
      }
      clear();
    }
  
    // Reset motion registers
    void clear() {
      sensor_L.motionClear();
      sensor_R.motionClear();
    }

    void displacement( float *dx, float *dy, float *dang ) {  
      // Displacements
      int8_t dx_L, dy_L;
      int8_t dx_R, dy_R;        
        
      // Read sensors 
      sensor_L.displacement( &dx_L, &dy_L );
      sensor_R.displacement( &dx_R, &dy_R );
    
      // Rotate components
      float fdx_L = float(dx_L);
      float fdy_L = float(dy_L);
      float fdx_R = float(dx_R);
      float fdy_R = float(dy_R);      
      
      rotate<LEFT_THETA>( &fdx_L, &fdy_L );
      rotate<RIGHT_THETA>( &fdx_R, &fdy_R );
    
      // Return combined velocities:
      constexpr float RES_SCALE = RESOLUTION ? 0.5 : 1;
      constexpr float LIN_SCALE = RES_SCALE * (*PIXEL_SCALE)*0.5; 
      constexpr float ANG_SCALE = RES_SCALE * (*PIXEL_SCALE)/(*DISTANCE);
        
      *dx   = ( fdx_L + fdx_R )*LIN_SCALE;
      *dy   = ( fdy_L + fdy_R )*LIN_SCALE;
      *dang = ( fdy_R - fdy_L )*ANG_SCALE;         
    }

    void velocity( float *vx, float *vy, float *wz ) {
      // Update timer
      float dt = float( micros() - last_time )*1e-6;
      last_time = micros();

      // Calculate rate
      displacement( vx, vy, wz );
      *vx /= dt;
      *vy /= dt;
      *wz /= dt;
    }
}; 

#endif
