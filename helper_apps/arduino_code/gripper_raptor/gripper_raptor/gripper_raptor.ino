#include <Servo.h>

// define Servo Port numbers
#define SRV_PORT_1 4
#define SRV_PORT_2 5



// define boundary constraints
#define SRV_1_CLOSE 111
#define SRV_1_OPEN 34
#define SRV_1_INIT 34

// OPEN:34 CLOSE:110

#define SRV_2_CLOSE 60
#define SRV_2_OPEN 137
#define SRV_2_INIT 138
// OPEN:138 CLOSE: 60

// Declare Servo Objects
Servo Servo_1;
Servo Servo_2;

// create buffer for Serial receiving (RX)
const int RX_BUFFER_SIZE = 2;
uint8_t rx_buffer[RX_BUFFER_SIZE];

void setup() {
  // initialize Servos
  Servo_1.attach(SRV_PORT_1);
  Servo_2.attach(SRV_PORT_2);


  Servo_1.write(SRV_1_INIT);
  Servo_2.write(SRV_2_INIT);
  //  initialize Serial
  Serial.begin(115200);
}

int cap(int val, int lower_boundary, int upper_boundary) {
  if (val < lower_boundary) {
    return lower_boundary;
  }
  if (val > upper_boundary) {
    return upper_boundary;
  }
  return val;
}

void loop() {

  /* SERVO TEST */
  // Servo_1.write(90);
  // Servo_2.write(90);
  // Servo_3.write(0);
  // // Servo_2.write(0);
  // delay(1000);
  // // Servo_1.write(110);
  // Servo_3.write(110);
  // Servo_1.write(170);
  // Servo_2.write(170);
  // delay(1000);

  float factor = 77.0/90.0;
  /* SERIAL CONNECTION */
  if (Serial.available()) {
    // read in command
    Serial.readBytes(rx_buffer, RX_BUFFER_SIZE);
    // write command to servo
    Servo_1.write(SRV_1_CLOSE + float(rx_buffer[0])*factor);
    Servo_2.write(SRV_2_CLOSE - float(rx_buffer[1])*factor);
  }
}
