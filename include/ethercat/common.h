#ifndef ESA_EWDL_ETHERCAT_COMMON_H
#define ESA_EWDL_ETHERCAT_COMMON_H
// STL
#include <string>
#include <map>
// SOEM
#include "ethercat.h"


namespace yaskawa { namespace ethercat {

// Motor specific limits
int gear_ratio[6] = { 25, 25, 50, 50, 50, 50 };
float rated_torque[6] = { 15.92, 15.92, 31.85, 31.85, 31.85, 63.5 }; // Motor rated torque in Nm
// float torque_limits_nm[6] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };  // Torque limit on drives in Nm
float velocity_limits[6] = { 1, 1, 1, 10, 1, 1 };  // rad/s
float position_limits[12] = { -1.3, 1.45,
                              -M_PI/2, M_PI/2,
                              -M_PI/2, M_PI/2,
                              -1000, 1000,
                              -M_PI/5, M_PI/5,
                              -0.25, M_PI/5 }; // rad
// Position limit format: { lower limit 1, upper limit 1, ..., lower limit 6, upper limit 6}
uint32 torque_denominator = 256;

uint16 torque_limits[6] = { 14000, 100, 7000, 7000, 7000, 100 };  // 9Nm in unit (1/256)% of rated torque
// eg. for 10Nm on motor 4: 10/(31.85*(1/256)*(1/100))



// Change torque unit from 1/10% to 1/256% 
// Modes of operation
enum mode_of_operation_t : int8
{
  PROFILE_POSITION = 1,              // Profile Position Mode
  PROFILE_VELOCITY = 3,              // Profile Velocity Mode
  TORQUE_PROFILE = 4,                // Torque Profile Mode
  HOMING = 6,                        // Homing Mode
  CYCLIC_SYNCHRONOUS_POSITION = 8,   // Cyclic Synchronous Position Mode
  CYCLIC_SYNCHRONOUS_VELOCITY = 9,   // Cyclic Synchronous Velocity Mode
  CYCLIC_SYNCHRONOUS_TORQUE = 10,   // Cyclic Synchronous Torque Mode
};


// Error codes
static std::map<uint16, std::string> error_codes
{
  { 0x7500, "EtherCAT communication error"},
  { 0xFF01, "Over Current"},
  { 0xFF02, "Over Voltage"},
  { 0xFF03, "Over Temperature"},
  { 0xFF04, "Open Motor Winding"},
  { 0xFF05, "Internal Voltage Bad"},
  { 0xFF06, "Position Limit"},
  { 0xFF07, "Bad Encoder"},
  { 0xFF08, "reserved"},
  { 0xFF09, "reserved"},
  { 0xFF0A, "Excess regen"},
  { 0xFF0B, "Safe Torque Off"},
  { 0xFF31, "CW Limit"},
  { 0xFF32, "CCW Limit"},
  { 0xFF33, "CW Limit and CCW Limit"},
  { 0xFF34, "Current Foldback"},
  { 0xFF35, "Move while Disabled"},
  { 0xFF36, "Under Voltage"},
  { 0xFF37, "Blank Q segment"},
  { 0xFF41, "Save Failed"},
  { 0xFFFF, "Other Error"},
};


// Alarm codes
static std::map<uint32, std::string> alarm_codes
{
  { 0x00000001, "Position Limit"},
  { 0x00000002, "CCW Limit"},
  { 0x00000004, "CW Limit"},
  { 0x00000008, "Over Temperature"},
  { 0x00000010, "Internal Voltage Bad"},
  { 0x00000020, "Over Voltage"},
  { 0x00000040, "Under Voltage"},
  { 0x00000080, "Over Current"},
  { 0x00000100, "Open Motor Winding"},
  { 0x00000200, "Bad Encoder"},
  { 0x00000400, "Communication Error"},
  { 0x00000800, "Save Failed"},
  { 0x00001000, "No Move"},
  { 0x00002000, "Current Foldback"},
  { 0x00004000, "Blank Q Segment"},
  { 0x00008000, "NV Memory Double Error"},
  { 0x00010000, "Excess Regen"},
  { 0x00020000, "-"},
  { 0x00040000, "Safe Torque Off"},
  { 0x00080000, "-"},
  { 0x00100000, "-"},
};


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, FALSE, size_of_data, &data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int writeSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOwrite(slave, index, sub_index, TRUE, size_of_data, data, EC_TIMEOUTRXM);

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T &value)
{
  int wkc = 0;

  T data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, FALSE, &size_of_data, &data, EC_TIMEOUTRXM);

  value = data;

  return wkc;
}


template<class T>
int readSDO(const uint16 slave, const uint16 index, const uint8 sub_index, T *value)
{
  int wkc = 0;

  T *data = value; int size_of_data = sizeof(data);
  wkc += ec_SDOread(slave, index, sub_index, TRUE, &size_of_data, data, EC_TIMEOUTRXM);

  *value = *data;

  return wkc;
}


inline void print_ec_state(uint16 slave_idx)
{
  switch (ec_slave[slave_idx].state)
  {
    case EC_STATE_NONE:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "NONE");
      break;
    case EC_STATE_INIT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "INIT");
      break;
    case EC_STATE_PRE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "PRE_OP");
      break;
    case EC_STATE_BOOT:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "BOOT");
      break;
    case EC_STATE_SAFE_OP:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "SAFE_OP");
      break;
    case EC_STATE_OPERATIONAL:
      printf("%s: EC_STATE: %s\n", ec_slave[slave_idx].name, "OPERATIONAL");
      break;
    //case EC_STATE_ACK:
    //  ROS_INFO("%s: ESM: %s", ec_slave[slave].name, "EC_STATE_ACK");
    //  break;
    case EC_STATE_PRE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "PRE_OP", "ERROR");
      break;
    case EC_STATE_SAFE_OP + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "SAFE_OP", "ERROR");
      break;
    case EC_STATE_OPERATIONAL + EC_STATE_ERROR:
      printf("%s: EC_STATE: %s + %s\n", ec_slave[slave_idx].name, "OPERATIONAL", "ERROR");
      break;
  }
}


inline void print_operation_mode(uint16 slave, int8 operation_mode)
{
  switch (operation_mode)
  {
    case 1:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "PP (Profile Position)");
      break;
    case 3:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "PV (Profile Velocity)");
      break;
    case 4:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "TQ (Torque Profile)");
      break;
    case 6:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "HM (Homing)");
      break;
    case 8:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "CSP (Cyclic Synchronous Position)");
      break;
    case 9:
      printf("%s: Mode of Operation: %s\n", ec_slave[slave].name, "CSV (Cyclic Synchronous Velocity)");
      break;
  }
}


inline void print_status_word(uint16 slave, uint16 status_word)
{
  // Ready to Switch On
  if ((status_word >> 0) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Ready to Swith On");
  }
  // Switched On
  if ((status_word >> 1) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Switched On");
  }
  // Operation Enabled
  if ((status_word >> 2) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Operation Enabled");
  }
  // Fault
  if ((status_word >> 3) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Fault");
  }
  // Voltage Enabled
  if ((status_word >> 4) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Voltage Enabled");
  }
  // Quick Stop
  if ((status_word >> 5) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Quick Stop");
  }
  // Switch On Disabled
  if ((status_word >> 6) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Switch On Disabled");
  }
  // Warning
  if ((status_word >> 7) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Warning");
  }
  // Remote
  if ((status_word >> 9) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Remote");
  }
  // Target Reached
  if ((status_word >> 10) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Target Reached");
  }
  // Internal Limit Active
  if ((status_word >> 11) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Internal Limit Active");
  }
  // Set Point ACK
  if (!(status_word >> 12) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "Previous set point already processed, waiting for new set point");
  }
  else
  {
    printf("%s: %s", ec_slave[slave].name, "Previous set point still in process, set point overwriting shall be accepted");
  }
  // Following Error
  if (!(status_word >> 13) & 0x01)
  {
    printf("%s: %s", ec_slave[slave].name, "No following error");
  }
  else
  {
    printf("%s: %s", ec_slave[slave].name, "Following Error");
  }
}


inline void print_error_code(uint16 slave, uint16 error_code)
{
  printf("%s: Error Code: 0x%.4x %s\n", ec_slave[slave].name, error_code, error_codes[error_code].c_str());
}


inline void print_alarm_code(uint16 slave, uint32 alarm_code)
{
  printf("%s: Alarm Code: 0x%.8x %s\n", ec_slave[slave].name, alarm_code, alarm_codes[alarm_code].c_str());
}

std::string toBinary(int n)
{
    std::string r;
    while(n!=0) {
      r = (n % 2 == 0 ? "0" : "1") + r; 
      n /= 2;}
    return r;
}

// inline void convert_torque_limits(float64 torque_nm[], uint16 torque_limits[], uint32 torque_denominator, bool print_result)
// {
//   int n = sizeof(torque_nm)/sizeof(torque_nm[0]);
//   for (int i = 0; i < n; i++){
//     float torque_converted = torque_limits_nm[i]*(1/torque_denominator)*100;
//     if(torque_converted > pow(2, 16)/2){
//       throw "Torque limit overflow!";
//     }
//     torque_limits[i] = static_cast<int16>(torque_converted);
//   }
//   if (print_result == true)
//   {
//     for (size_t i = 0; i < n; i++) {
//       std::cout << "Torque limit" << torque_limits[i] << ' ';
//     }
//   }
// }

} } // namespace
#endif
