#include "Arduino.h"
#include <Wire.h>

#ifndef basicMPU6050_h
#define basicMPU6050_h

#define MPU_ADDRESS             0x68
#define ACCEL_LBS_0             16384.0
#define N_AXIS                  3
constexpr float DEFAULT_ONE   = 1;

template < uint8_t  DLPF_CFG  = 0           ,
           uint8_t  FS_SEL    = 0           ,
           uint8_t  AFS_SEL   = 0           ,
           int      AX_OFS    = 0           ,
           int      AY_OFS    = 0           ,
           int      AZ_OFS    = 0           ,
           const float* AX_S  = &DEFAULT_ONE,
           const float* AY_S  = &DEFAULT_ONE,
           const float* AZ_S  = &DEFAULT_ONE,
           const float* GX_S  = &DEFAULT_ONE,
           const float* GY_S  = &DEFAULT_ONE,
           const float* GZ_S  = &DEFAULT_ONE,
           uint16_t GYRO_BAND = 10          ,
           uint32_t N_BIAS    = 10000       >
class basicMPU6050 {
   private:       
    float mean[N_AXIS]  = {0};    
        
    static constexpr float ACCEL_LBS =   AFS_SEL < 0 || AFS_SEL >  3 ? 1         : 
                                                  1.0/( AFS_SEL == 0 ? 16384.0   : 
                                                        AFS_SEL == 1 ? 8192.0    :
                                                        AFS_SEL == 2 ? 4096.0    :
                                                                       2048.0    );
    static constexpr float GYRO_LBS =   FS_SEL < 0 || FS_SEL >  3 ? 1      :
                                         (PI/180.0)/( FS_SEL == 0 ? 131.0  :
                                                      FS_SEL == 1 ? 65.5   :
                                                      FS_SEL == 2 ? 32.8   :
                                                                    16.4   );
    static constexpr float MEAN = 1.0/float(N_BIAS);
    
    void setRegister( uint8_t reg, uint8_t mode ) {
      Wire.beginTransmission(MPU_ADDRESS);                                        
      Wire.write(reg);                                                   
      Wire.write(mode);                                                    
      Wire.endTransmission();
    }
    
    void readRegister( uint8_t reg ) {
      Wire.beginTransmission(MPU_ADDRESS);                                       
      Wire.write(reg);                                                   
      Wire.endTransmission();                                               
      Wire.requestFrom( MPU_ADDRESS, 2, true );       
    }

    int readWire() {
      return int( Wire.read()<<8|Wire.read() );
    }

   public:       
    void setup() {      
      setRegister( 0x6B, 0x00 );
      setRegister( 0x1A, DLPF_CFG <= 6 ? DLPF_CFG : 0x00 );
      
      setRegister( 0x1B, FS_SEL == 1 ? 0x08   :
                         FS_SEL == 2 ? 0x10   :
                         FS_SEL == 3 ? 0x18   :
                                       0x00   );
                                             
      setRegister( 0x1C, AFS_SEL == 1 ? 0x08  :
                         AFS_SEL == 2 ? 0x10  :
                         AFS_SEL == 3 ? 0x18  :
                                        0x00  );
    }

    //-- Raw measurements
    
    // Accel
    int rawAx() {
      readRegister( 0x3B );
      return readWire();       
    }
    int rawAy() {
      readRegister( 0x3D );
      return readWire();
    }
    int rawAz() {
      readRegister( 0x3F );
      return readWire();
    }

    // Temp
    int rawTemp() {
      readRegister( 0x41 );                                                                                                           
      return readWire();                                 
    }

    // Gyro
    int rawGx() {
      readRegister( 0x43 );   
      return readWire();    
    }
    int rawGy() {
      readRegister( 0x45 );
      return readWire();
    }
    int rawGz() {
      readRegister( 0x47 );
      return readWire();
    }

    //-- Scaled measurements

    // Accel
    float ax() {
      constexpr float SCALE  = (*AX_S) * ACCEL_LBS;
      constexpr float OFFSET = (*AX_S) * float(AX_OFS)/ACCEL_LBS_0;   
      //
      return float( rawAx() )*SCALE - OFFSET;       
    }
    float ay() {
      constexpr float SCALE  = (*AY_S) * ACCEL_LBS;
      constexpr float OFFSET = (*AY_S) * float(AY_OFS)/ACCEL_LBS_0;  
      //
      return float( rawAy() )*SCALE - OFFSET;         
    }
    float az() {  
      constexpr float SCALE  = (*AZ_S) * ACCEL_LBS;
      constexpr float OFFSET = (*AZ_S) * float(AZ_OFS)/ACCEL_LBS_0; 
      //
      return float( rawAz() )*SCALE - OFFSET;         
    }

    // Temp
    float temp() {
      constexpr float TEMP_MUL = 1.0/340.0;                                  
      return rawTemp()*TEMP_MUL + 36.53;
    }

    // Gyro
    float gx() {
      constexpr float SCALE = (*GX_S) * GYRO_LBS;
      return ( float( rawGx() ) - mean[0] )*SCALE;  
    }
    float gy() {
      constexpr float SCALE = (*GY_S) * GYRO_LBS;
      return ( float( rawGy() ) - mean[1] )*SCALE;  
    }
    float gz() {
      constexpr float SCALE = (*GZ_S) * GYRO_LBS;
      return ( float( rawGz() ) - mean[2] )*SCALE;  
    }

    //-- Bias estimate        
    void setBias() {    
      for( int count = 0; count < N_BIAS; count += 1 ) { 
        int gyro[] = { rawGx(), rawGy(), rawGz() };
        //
        for( int index = 0; index < N_AXIS; index += 1 ) {
          mean[index] += float( gyro[index] ); 
        } 
      }
      //
      for( int index = 0; index < N_AXIS; index += 1 ) {
        mean[index] *= MEAN; 
      }    
    }

    void updateBias() {
      constexpr float GYRO_BAND_SQ = GYRO_BAND*GYRO_BAND;
      //
      int gyro[N_AXIS] = { rawGx(), rawGy(), rawGz() };
      
      for( int index = 0; index < N_AXIS; index += 1 ) {
        float dx = float( gyro[index] ) - mean[index];
        float gain = GYRO_BAND_SQ/( GYRO_BAND_SQ + dx*dx );
        //
        mean[index] += dx*( MEAN*gain );
      }
    }
};
 
#endif
