#include <Servo.h>

// define Servo Port numbers
#define SRV_PORT_1 4

// define boundary constraints
#define SRV_1_MAX 180
#define SRV_1_MIN 0

// Declare Servo Objects
Servo Servo_1;

// create buffer for Serial receiving (RX)
const int RX_BUFFER_SIZE = 1;
uint8_t rx_buffer[RX_BUFFER_SIZE];

void setup() {
  // initialize Servos
  Servo_1.attach(SRV_PORT_1);

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

  /* SERIAL CONNECTION */
  if (Serial.available()) {
    // read in command
    Serial.readBytes(rx_buffer, RX_BUFFER_SIZE);
    // write command to servo
    Servo_1.write(cap(rx_buffer[0], SRV_1_MIN, SRV_1_MAX));
  }
}
