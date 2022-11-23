#include <chrono>
#include <iostream>
#include <thread>

// serialib
#include "serialib.h"

#define SERIAL_PORT "/dev/ttyACM2"

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

int main(int argc, char *argv[]) {

  if (argc != 2) {
    std::cout << "ERROR: no serial port given as input argument" << std::endl;
    return 0;
  }
  std::string serial_port = argv[1];
  std::cout << "Initializing serial connection to port " << serial_port
            << " ..." << std::endl;

  // initialize Serial object
  serialib serial;

  // If connection fails, return the error code otherwise, display a success
  // message
  int serialResult = serial.openDevice(serial_port.c_str(), 115200);
  if (serialResult == -2) {
    std::cout << "Could not initialize serial connection to " << serial_port
              << std::endl;
    return -1;
  }
  std::cout << "Successful connection to " << serial_port << std::endl;

  // initialize Data
  unsigned char Command_Data[] = {0};

  for (int i = 0; i < 180; i++) {
    /* SERIAL SENDING */

    //  write bytes (8bit = cnversion to char type)
    Command_Data[0] = i;

    serial.writeBytes(&Command_Data, sizeof(Command_Data));
    std::cout << "just wrote" << (int)Command_Data[0] << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  serial.closeDevice();
  return 0;
}