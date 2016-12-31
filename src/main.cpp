
#include "Arduino.h"

// the setup function runs once when you press reset or power the board
// To use VescUartControl stand alone you need to define a config.h file, that should contain the Serial or you have to comment the line
// #include Config.h out in VescUart.h

//Include libraries copied from VESC
 #include <VescUart.h>
#include <datatypes.h>


#define DEBUG
unsigned long count;
unsigned long last_comm_read=0;
unsigned long last_pos_print=0;
unsigned long last_control_loop=0;

int message_timeout = 50; // ms, set current to 0 if no messages received back in this time
unsigned long last_message=0;

int rotor_position_read_interval = 5; //ms // 100hz // sometimes freezes at 200hz?
int rotor_position_print_interval = 100; //ms // 50hz

int control_loop_interval = 5; //ms // 100hz

float position_error=0.0;
float rotor_position = 0.0;
float last_rotor_position =0.0;

void setup() {
	//Setup UART port
	Serial1.begin(115200);
  #ifdef DEBUG
	//SEtup debug port
	Serial.begin(115200);
	#endif

  VescUartSetDetect(DISP_POS_MODE_ENCODER);
  delay(100);
  VescUartSetDetect(DISP_POS_MODE_ENCODER);
}

struct bldcMeasure measuredValues;

// the loop function runs over and over again until power down or reset
void loop() {
  // position control
  /*
  float desired_position = 180*sin(millis()*6.28/1000.0);
  VescUartSetPosition(desired_position);

  Serial.println(desired_position);
  delay(1);
  */

  // 5ms interval ok, 4ms ok, 1ms freezes prints, 2ms freezes, 3ms ok
  if(micros() - last_comm_read > rotor_position_read_interval*1000) {
    // update last comm read time
    last_comm_read = micros();

    int comm_status = VescUartCheckComm(measuredValues);
    if (comm_status) {
      // update last rotor position
      last_rotor_position = rotor_position;
      // extract position
      rotor_position = measuredValues.rotorPosition;

      //
      // if(micros() - last_pos_print > rotor_position_print_interval*1000) {
      //   // last print time
      //   last_pos_print = micros();
      //   // Serial.println(rotor_position);
      //   VescUartSetDetect(DISP_POS_MODE_ENCODER);
      // }
      last_message = micros();
    }
    else if(comm_status==-1) {
      Serial.println("Unable to parse message");
    } else {

      // no message received
    }
  }
  if(micros() - last_control_loop > control_loop_interval*1000) {
    last_control_loop = micros();

    position_error = 180.0 - rotor_position;
    float Kp = 0.05;//0.03;
    //float Kd = 0;
    float Kd = 0.001;

    float position_difference = rotor_position - last_rotor_position;
    // broken roll over speed detection
    /*
    if(last_rotor_position < 360 && rotor_position > 0) {
      position_difference += 360;
    }
    if(last_rotor_position > 0 && rotor_position < 360) {
      position_difference -= 360;
    }
    */

    float velocity = position_difference *1000 / (float)rotor_position_read_interval;



    float current_command = -Kp*position_error + Kd*velocity;

    constrain(current_command,-10.0,10.0);

    if(micros()-last_message > message_timeout*1000) {
      VescUartSetCurrent(0.0);
    }
    else {
      VescUartSetCurrent(current_command);
    }

    if(micros() - last_pos_print > rotor_position_print_interval*1000) {
      last_pos_print = micros();
      Serial.print(velocity);
      Serial.print(" ");
      Serial.print(position_error);
      Serial.print(" ");
      Serial.print(current_command);
      Serial.print(" ");
      Serial.print(-Kp*position_error);
      Serial.print(" ");
      Serial.print(Kd*velocity);
      Serial.println();
    }


    /*
    Serial.print(rotor_position);
    Serial.print(" ");
    Serial.print(last_rotor_position);
    Serial.print(" ");
    Serial.print(position_difference);
    Serial.println();
    */



  }
}
