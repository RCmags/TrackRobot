#include <arduino.h>

#ifndef BLmodule_h
#define BLmodule_h

// Select Serial connection:
#ifdef SERIAL_3 
  #define SERIAL_N Serial3
#endif

#ifdef SERIAL_2
  #define SERIAL_N Serial2
#endif

#ifdef SERIAL_1
  #define SERIAL_N Serial
#endif

template<uint32_t BAUD_RATE, int CHAR_MAX, int POINT_MAX>
class BLmodule {
  private:
    int stack[POINT_MAX][2] = {0};
    char input[CHAR_MAX + 1];
    uint8_t index_i = 0;
    uint8_t index_s = 0;
    
  public:
    void setup() {
      SERIAL_N.begin(BAUD_RATE);
	  SERIAL_N.flush();
      
	  // Configure stack before entering main loop
      SERIAL_N.println(">> Setup stack\n");
      while( true ) {
        checkSerial();
        if( strcmp("EXIT", input) == 0 ) {
          SERIAL_N.println(input);
          break;
        }
      }
      SERIAL_N.println("\n>> Setup complete\n");
    }
  
    uint8_t stackSize() {
      return index_s;
    }

	// Retrieve coordinate from stack
    void getStack( uint8_t *index, float value[2] ) {
      *index = *index > index_s ? index_s : *index;
      value[0] = stack[*index][0];
      value[1] = stack[*index][1];
    }
 
	// Receive inputs and update stack
    void checkSerial() {
      if( SERIAL_N.available() ) {
        char charIn = SERIAL_N.read();
        
		// Accumulate characters to form string 
        if( charIn != '\n' && index_i != CHAR_MAX ) {
          input[index_i] = charIn;
          index_i += 1;
        
		// Terminate string once newline or limit is reached
        } else {
          input[index_i] = '\0';
          index_i = 0;
          
		  //-- Parse string as command:
          
		  // Add coordinate to the stack
		  if( strncmp("PUSH", input, 4) == 0 ) {
            SERIAL_N.println(input);
            
			// Strings that are expected to have numbers
            char* str_num1 = input + 4;
            char* str_num2 = strchr(str_num1, ',') + 1;
			
			// Convert strings to intergers
            if( index_s != POINT_MAX ) {
              stack[index_s][0] = atoi(str_num1);
              stack[index_s][1] = atoi(str_num2);  
              index_s += 1;
            } else {
              SERIAL_N.println(">> Full stack");
            }
          
		  // Display current stack values
          } else if( strncmp("STACK", input, 5) == 0 ) {
            SERIAL_N.println(input);
			
            // Display all points up to the stack index
            if( index_s != 0 ) {
              for( int i = 0; i < index_s; i += 1 ) {
                SERIAL_N.print("(");
                SERIAL_N.print( stack[i][0] );
                SERIAL_N.print(",");
                SERIAL_N.print( stack[i][1] );
                SERIAL_N.print(")");
              }
              SERIAL_N.println();
            } else {
              SERIAL_N.println(">> Empty stack");
            }
          
		  // Remove last coordinate from the stack
          } else if( strncmp("POP", input, 3) == 0 ) {
            SERIAL_N.println(input);
            
			// Shift stack index to a lower value
            if( index_s != 0 ) {
              index_s -= 1;
            } else {
              SERIAL_N.println(">> Empty stack" );
            }  
          } 
        }
      }      
    }
};

#endif
