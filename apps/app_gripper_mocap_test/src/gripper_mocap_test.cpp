#include "DataStreamClient.h"
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string.h>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h> // For sleep()
#include <vector>

using namespace ViconDataStreamSDK::CPP;

namespace {
std::string Adapt(const bool i_Value) { return i_Value ? "True" : "False"; }

// Set time standard
std::string Adapt(const TimecodeStandard::Enum i_Standard) {
  switch (i_Standard) {
  default:
  case TimecodeStandard::None:
    return "0";
  case TimecodeStandard::PAL:
    return "1";
  case TimecodeStandard::NTSC:
    return "2";
  case TimecodeStandard::NTSCDrop:
    return "3";
  case TimecodeStandard::Film:
    return "4";
  case TimecodeStandard::NTSCFilm:
    return "5";
  case TimecodeStandard::ATSC:
    return "6";
  }
}

// Doubt - Set the direction for i axis
std::string Adapt(const Direction::Enum i_Direction) {
  switch (i_Direction) {
  case Direction::Forward:
    return "Forward";
  case Direction::Backward:
    return "Backward";
  case Direction::Left:
    return "Left";
  case Direction::Right:
    return "Right";
  case Direction::Up:
    return "Up";
  case Direction::Down:
    return "Down";
  default:
    return "Unknown";
  }
}

// Enable forceplate input device (not relevant)
std::string Adapt(const DeviceType::Enum i_DeviceType) {
  switch (i_DeviceType) {
  case DeviceType::ForcePlate:
    return "ForcePlate";
  case DeviceType::Unknown:
  default:
    return "Unknown";
  }
}

// Set the unit for state variable
std::string Adapt(const Unit::Enum i_Unit) {
  switch (i_Unit) {
  case Unit::Meter:
    return "Meter";
  case Unit::Volt:
    return "Volt";
  case Unit::NewtonMeter:
    return "NewtonMeter";
  case Unit::Newton:
    return "Newton";
  case Unit::Kilogram:
    return "Kilogram";
  case Unit::Second:
    return "Second";
  case Unit::Ampere:
    return "Ampere";
  case Unit::Kelvin:
    return "Kelvin";
  case Unit::Mole:
    return "Mole";
  case Unit::Candela:
    return "Candela";
  case Unit::Radian:
    return "Radian";
  case Unit::Steradian:
    return "Steradian";
  case Unit::MeterSquared:
    return "MeterSquared";
  case Unit::MeterCubed:
    return "MeterCubed";
  case Unit::MeterPerSecond:
    return "MeterPerSecond";
  case Unit::MeterPerSecondSquared:
    return "MeterPerSecondSquared";
  case Unit::RadianPerSecond:
    return "RadianPerSecond";
  case Unit::RadianPerSecondSquared:
    return "RadianPerSecondSquared";
  case Unit::Hertz:
    return "Hertz";
  case Unit::Joule:
    return "Joule";
  case Unit::Watt:
    return "Watt";
  case Unit::Pascal:
    return "Pascal";
  case Unit::Lumen:
    return "Lumen";
  case Unit::Lux:
    return "Lux";
  case Unit::Coulomb:
    return "Coulomb";
  case Unit::Ohm:
    return "Ohm";
  case Unit::Farad:
    return "Farad";
  case Unit::Weber:
    return "Weber";
  case Unit::Tesla:
    return "Tesla";
  case Unit::Henry:
    return "Henry";
  case Unit::Siemens:
    return "Siemens";
  case Unit::Becquerel:
    return "Becquerel";
  case Unit::Gray:
    return "Gray";
  case Unit::Sievert:
    return "Sievert";
  case Unit::Katal:
    return "Katal";

  case Unit::Unknown:
  default:
    return "Unknown";
  }
}
#ifdef WIN32
bool Hit() {
  bool hit = false;
  while (_kbhit()) {
    getchar();
    hit = true;
  }
  return hit;
}
#endif

class NullBuffer : public std::streambuf {
public:
  int overflow(int c) { return c; }
};

NullBuffer Null;
std::ostream NullStream(&Null);

} // namespace

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

  for (unsigned char i = 0; i < 180; i += 45) {
    /* SERIAL SENDING */

    //  write bytes (8bit = cnversion to char type)
    Command_Data[0] = i;

    serial.writeBytes(&Command_Data, sizeof(Command_Data));
    std::cout << "just wrote" << (int)Command_Data[0] << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  serial.closeDevice();
  if (argc > 11) {
    std::cout << "[ERROR] Mocap publisher only supports 10 objects. Change "
                 "code to allow for more."
              << std::endl;
    return 0;
  }

  std::string falcon_name = "srl_falcon";
  std::string object_name = "srl_object";
  std::vector<double> falcon_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> object_pose{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  /* VICON Datastream Settings */
  std::vector<std::string> Hosts;
  // HARD CODE Vicon Address from LEO C6
  Hosts.push_back("10.10.10.5");
  if (Hosts.empty()) {
    Hosts.push_back("localhost:801");
  }

  bool bOptimizeWireless = false;
  bool bQuiet = false;

  std::vector<std::string> HapticOnList(0);
  unsigned int ClientBufferSize = 0;
  std::string AxisMapping = "ZUp";
  std::vector<std::string> FilteredSubjects;
  std::vector<std::string> LocalAdapters;

  std::ostream &OutputStream(bQuiet ? NullStream : std::cout);

  bool First = true;
  std::string HostName;
  for (const auto &rHost : Hosts) {
    if (!First) {
      HostName += ";";
    }
    HostName += rHost;
    First = false;
  }

  // Make a new client
  ViconDataStreamSDK::CPP::Client DirectClient;

  if (bOptimizeWireless) {
    const Output_ConfigureWireless ConfigureWirelessResult =
        DirectClient.ConfigureWireless();

    if (ConfigureWirelessResult.Result != Result::Success) {
      std::cout << "Wireless Config: " << ConfigureWirelessResult.Error
                << std::endl;
    }
  }

  std::cout << "Connecting to " << HostName << " ..." << std::flush;
  while (!DirectClient.IsConnected().Connected) {
    // Direct connection
    const Output_Connect ConnectResult = DirectClient.Connect(HostName);
    const bool ok = (ConnectResult.Result == Result::Success);

    if (!ok) {
      std::cout << "Warning - connect failed... ";
      switch (ConnectResult.Result) {
      case Result::ClientAlreadyConnected:
        std::cout << "Client Already Connected" << std::endl;
        break;
      case Result::InvalidHostName:
        std::cout << "Invalid Host Name" << std::endl;
        break;
      case Result::ClientConnectionFailed:
        std::cout << "Client Connection Failed" << std::endl;
        break;
      default:
        std::cout << "Unrecognized Error: " << ConnectResult.Result
                  << std::endl;
        break;
      }
    }

    std::cout << ".";

    sleep(1);

    // }

    std::cout << std::endl;

    std::cout << std::endl;
    // Enable some different data types
    DirectClient.EnableSegmentData();

    DirectClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    // }

    // Set the global up axis
    DirectClient.SetAxisMapping(Direction::Forward, Direction::Left,
                                Direction::Up); // Z-up

    if (AxisMapping == "YUp") {
      DirectClient.SetAxisMapping(Direction::Forward, Direction::Up,
                                  Direction::Right); // Y-up
    } else if (AxisMapping == "XUp") {
      DirectClient.SetAxisMapping(Direction::Up, Direction::Forward,
                                  Direction::Left); // Y-up
    }

    Output_GetAxisMapping _Output_GetAxisMapping =
        DirectClient.GetAxisMapping();
    // std::cout << "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis) <<
    // " Y-" << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
    //          << Adapt(_Output_GetAxisMapping.ZAxis) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = DirectClient.GetVersion();
    // std::cout << "Version: " << _Output_GetVersion.Major << "." <<
    // _Output_GetVersion.Minor << "." << _Output_GetVersion.Point << "."
    //           << _Output_GetVersion.Revision << std::endl;

    if (ClientBufferSize > 0) {
      DirectClient.SetBufferSize(ClientBufferSize);
      std::cout << "Setting client buffer size to " << ClientBufferSize
                << std::endl;
    }

    bool bSubjectFilterApplied = false;

    ViconDataStreamSDK::CPP::Client &MyClient(DirectClient);

    size_t Counter = 0;
    const std::chrono::high_resolution_clock::time_point StartTime =
        std::chrono::high_resolution_clock::now();
    // Loop until a key is pressed

    while (true)

    {
      // Get a frame
      // OutputStream << "Waiting for new frame...";
      while (MyClient.GetFrame().Result != Result::Success) {
        // Sleep a little so that we don't lumber the CPU with a busy poll

        sleep(1);

        // OutputStream << ".";
      }
      // OutputStream << std::endl;

      // We have to call this after the call to get frame, otherwise we don't
      // have any subject info to map the name to ids
      if (!bSubjectFilterApplied) {
        for (const auto &rSubject : FilteredSubjects) {
          Output_AddToSubjectFilter SubjectFilterResult =
              MyClient.AddToSubjectFilter(rSubject);
          bSubjectFilterApplied = bSubjectFilterApplied ||
                                  SubjectFilterResult.Result == Result::Success;
        }
      }

      const std::chrono::high_resolution_clock::time_point Now =
          std::chrono::high_resolution_clock::now();

      // Get the frame number
      Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      // OutputStream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber
      // << std::endl;

      ////////////////////////////////////////////////////////
      // for (int i = 0; i < N; i++) {
      //   mocap_msg.at(i).header.timestamp =
      //   _Output_GetFrameNumber.FrameNumber;
      // }
      // for (int i = 0; i < argc - 1; i++) {
      //   msg[i].header.timestamp = _Output_GetFrameNumber.FrameNumber;
      // }
      ////////////////////////////////////////////////////////
      Output_GetFrameRate Rate = MyClient.GetFrameRate();
      // OutputStream << "Frame rate: " << Rate.FrameRateHz << std::endl;

      // Show frame rates
      for (unsigned int FramerateIndex = 0;
           FramerateIndex < MyClient.GetFrameRateCount().Count;
           ++FramerateIndex) {
        std::string FramerateName =
            MyClient.GetFrameRateName(FramerateIndex).Name;
        double FramerateValue = MyClient.GetFrameRateValue(FramerateName).Value;

        // OutputStream << FramerateName << ": " << FramerateValue << "Hz" <<
        // std::endl;
      }
      // OutputStream << std::endl;

      // Get the timecode
      Output_GetTimecode _Output_GetTimecode = MyClient.GetTimecode();

      // OutputStream << "Timecode: " << _Output_GetTimecode.Hours << "h " <<
      // _Output_GetTimecode.Minutes << "m " << _Output_GetTimecode.Seconds <<
      // "s
      // "
      //   << _Output_GetTimecode.Frames << "f " << _Output_GetTimecode.SubFrame
      //   << "sf " << Adapt(_Output_GetTimecode.FieldFlag) << " "
      //   << Adapt(_Output_GetTimecode.Standard) << " " <<
      //   _Output_GetTimecode.SubFramesPerFrame << " " <<
      //   _Output_GetTimecode.UserBits
      //   << std::endl
      //   << std::endl;

      // Get the latency
      // OutputStream << "Latency: " << MyClient.GetLatencyTotal().Total << "s"
      // << std::endl;

      ///////////////////////////////////////////////////////////////
      // for (int i = 0; i < N; i++) {
      //   mocap_msg.at(i).latency = MyClient.GetLatencyTotal().Total;
      // }
      // for (int i = 0; i < argc - 1; i++) {
      //   msg[i].latency = MyClient.GetLatencyTotal().Total;
      // }

      for (unsigned int LatencySampleIndex = 0;
           LatencySampleIndex < MyClient.GetLatencySampleCount().Count;
           ++LatencySampleIndex) {
        std::string SampleName =
            MyClient.GetLatencySampleName(LatencySampleIndex).Name;
        double SampleValue = MyClient.GetLatencySampleValue(SampleName).Value;

        // OutputStream << "  " << SampleName << " " << SampleValue << "s" <<
        // std::endl;
      }
      // OutputStream << std::endl;

      Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber =
          MyClient.GetHardwareFrameNumber();
      // OutputStream << "Hardware Frame Number: " <<
      // _Output_GetHardwareFrameNumber.HardwareFrameNumber << std::endl;

      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // OutputStream << "Subjects (" << SubjectCount << "):" << std::endl;

      // loop over all subject
      for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount;
           ++SubjectIndex) { ////////////////////////////////////////////for
        /// loop starts here
        // OutputStream << "  Subject #" << SubjectIndex << std::endl;

        // Get the subject name
        std::string SubjectName =
            MyClient.GetSubjectName(SubjectIndex).SubjectName;
        // OutputStream << "    Name: " << SubjectName << std::endl;

        ////////////////////////////////////////////

        // Get the root segment
        std::string RootSegment =
            MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
        // OutputStream << "    Root Segment: " << RootSegment << std::endl;

        // Count the number of segments
        unsigned int SegmentCount =
            MyClient.GetSegmentCount(SubjectName).SegmentCount;
        // OutputStream << "    Segments (" << SegmentCount << "):" <<
        // std::endl;

        // loop over all segments
        for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount;
             ++SegmentIndex) {
          // OutputStream << "      Segment #" << SegmentIndex << std::endl;

          // Get the segment name
          std::string SegmentName =
              MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
          // OutputStream << "        Name: " << SegmentName << std::endl;

          // Get the segment parent
          std::string SegmentParentName =
              MyClient.GetSegmentParentName(SubjectName, SegmentName)
                  .SegmentName;
          // OutputStream << "        Parent: " << SegmentParentName <<
          // std::endl;

          // Get the segment's children
          unsigned int ChildCount =
              MyClient.GetSegmentChildCount(SubjectName, SegmentName)
                  .SegmentCount;
          // OutputStream << "     Children (" << ChildCount << "):" <<
          // std::endl;
          for (unsigned int ChildIndex = 0; ChildIndex < ChildCount;
               ++ChildIndex) {
            std::string ChildName =
                MyClient
                    .GetSegmentChildName(SubjectName, SegmentName, ChildIndex)
                    .SegmentName;
            // OutputStream << "       " << ChildName << std::endl;
          }

          // Get the static segment scale
          Output_GetSegmentStaticScale _Output_GetSegmentStaticScale =
              MyClient.GetSegmentStaticScale(SubjectName, SegmentName);

          // Get the static segment translation
          Output_GetSegmentStaticTranslation
              _Output_GetSegmentStaticTranslation =
                  MyClient.GetSegmentStaticTranslation(SubjectName,
                                                       SegmentName);

          // Get the static segment rotation in helical co-ordinates
          Output_GetSegmentStaticRotationHelical
              _Output_GetSegmentStaticRotationHelical =
                  MyClient.GetSegmentStaticRotationHelical(SubjectName,
                                                           SegmentName);

          // Get the static segment rotation as a matrix
          Output_GetSegmentStaticRotationMatrix
              _Output_GetSegmentStaticRotationMatrix =
                  MyClient.GetSegmentStaticRotationMatrix(SubjectName,
                                                          SegmentName);

          // Get the static segment rotation in quaternion co-ordinates
          Output_GetSegmentStaticRotationQuaternion
              _Output_GetSegmentStaticRotationQuaternion =
                  MyClient.GetSegmentStaticRotationQuaternion(SubjectName,
                                                              SegmentName);
          // Get the static segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentStaticRotationEulerXYZ
              _Output_GetSegmentStaticRotationEulerXYZ =
                  MyClient.GetSegmentStaticRotationEulerXYZ(SubjectName,
                                                            SegmentName);

          // Get the global segment translation
          Output_GetSegmentGlobalTranslation
              _Output_GetSegmentGlobalTranslation =
                  MyClient.GetSegmentGlobalTranslation(SubjectName,
                                                       SegmentName);

          if (SubjectName.compare(falcon_name) == 0) {
            falcon_pose.at(0) =
                _Output_GetSegmentGlobalTranslation.Translation[0] / 1000.0;
            falcon_pose.at(1) =
                _Output_GetSegmentGlobalTranslation.Translation[1] / 1000.0;
            falcon_pose.at(2) =
                _Output_GetSegmentGlobalTranslation.Translation[2] / 1000.0;
          }
          if (SubjectName.compare(object_name) == 0) {
            object_pose.at(0) =
                _Output_GetSegmentGlobalTranslation.Translation[0] / 1000.0;
            object_pose.at(1) =
                _Output_GetSegmentGlobalTranslation.Translation[1] / 1000.0;
            object_pose.at(2) =
                _Output_GetSegmentGlobalTranslation.Translation[2] / 1000.0;
          }

          ////////////////////////////////////////////

          // Get the global segment rotation in helical co-ordinates
          Output_GetSegmentGlobalRotationHelical
              _Output_GetSegmentGlobalRotationHelical =
                  MyClient.GetSegmentGlobalRotationHelical(SubjectName,
                                                           SegmentName);

          // Get the global segment rotation as a matrix
          Output_GetSegmentGlobalRotationMatrix
              _Output_GetSegmentGlobalRotationMatrix =
                  MyClient.GetSegmentGlobalRotationMatrix(SubjectName,
                                                          SegmentName);

          // Get the global segment rotation in quaternion co-ordinates
          Output_GetSegmentGlobalRotationQuaternion
              _Output_GetSegmentGlobalRotationQuaternion =
                  MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                              SegmentName);

          // Get the global segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentGlobalRotationEulerXYZ
              _Output_GetSegmentGlobalRotationEulerXYZ =
                  MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName,
                                                            SegmentName);

          // publish euler Rotation to DDS in degrees
          if (SubjectName.compare(falcon_name) == 0) {
            falcon_pose.at(3) =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0] *
                (180.0 / M_PI);
            falcon_pose.at(4) =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1] *
                (180.0 / M_PI);
            falcon_pose.at(5) =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2] *
                (180.0 / M_PI);
          }
          if (SubjectName.compare(object_name) == 0) {
            object_pose.at(3) =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0] *
                (180.0 / M_PI);
            object_pose.at(4) =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1] *
                (180.0 / M_PI);
            object_pose.at(5) =
                _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2] *
                (180.0 / M_PI);
          }
          // Get the local segment translation
          Output_GetSegmentLocalTranslation _Output_GetSegmentLocalTranslation =
              MyClient.GetSegmentLocalTranslation(SubjectName, SegmentName);

          // Get the local segment rotation in helical co-ordinates
          Output_GetSegmentLocalRotationHelical
              _Output_GetSegmentLocalRotationHelical =
                  MyClient.GetSegmentLocalRotationHelical(SubjectName,
                                                          SegmentName);

          Output_GetSegmentLocalRotationMatrix
              _Output_GetSegmentLocalRotationMatrix =
                  MyClient.GetSegmentLocalRotationMatrix(SubjectName,
                                                         SegmentName);

          // Get the local segment rotation in quaternion co-ordinates
          Output_GetSegmentLocalRotationQuaternion
              _Output_GetSegmentLocalRotationQuaternion =
                  MyClient.GetSegmentLocalRotationQuaternion(SubjectName,
                                                             SegmentName);

          // Get the local segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentLocalRotationEulerXYZ
              _Output_GetSegmentLocalRotationEulerXYZ =
                  MyClient.GetSegmentLocalRotationEulerXYZ(SubjectName,
                                                           SegmentName);
        }
      }

      std::cout << "new frame" << std::endl;
      double alpha = atan2(falcon_pose.at(2) - object_pose.at(2),
                           object_pose.at(0) - falcon_pose.at(0));
      unsigned char Command_Data[] = {0};

      Command_Data[0] = alpha;

      serial.writeBytes(&Command_Data, sizeof(Command_Data));

      // std::this_thread::sleep_for(std::chrono::milliseconds(500));
      // delay ??
    }

    ++Counter;
  }
  serial.closeDevice();
}