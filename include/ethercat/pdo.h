#ifndef ESA_EWDL_ETHERCAT_RXPDO_H
#define ESA_EWDL_ETHERCAT_RXPDO_H

// Default PDO mappings from Yaskawa Sigma-7 documentation

namespace yaskawa { namespace ethercat { namespace pdo {

// Need to add other default PDO mappings. Only cyclic synchronous position/velocity here.


// PDO1 (Custom Synchronous Torque)
struct RxPDO1
{
  uint16 control_word;
  int32 target_position;
  int32 target_velocity;
  int16 target_torque;
  uint16 max_torque;
  int8 operation_mode;
  uint16 touch_probe_function;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_position >> 0) & 0xFF;
    *data_ptr++ = (target_position >> 8) & 0xFF;
    *data_ptr++ = (target_position >> 16) & 0xFF;
    *data_ptr++ = (target_position >> 24) & 0xFF;


    *data_ptr++ = (target_velocity >> 0) & 0xFF;
    *data_ptr++ = (target_velocity >> 8) & 0xFF;
    *data_ptr++ = (target_velocity >> 16) & 0xFF;
    *data_ptr++ = (target_velocity >> 24) & 0xFF;

    *data_ptr++ = (target_torque >> 0) & 0xFF;
    *data_ptr++ = (target_torque >> 8) & 0xFF;

    *data_ptr++ = (max_torque >> 0) & 0xFF;
    *data_ptr++ = (max_torque >> 8) & 0xFF;

    *data_ptr++ = (operation_mode >> 0) & 0xFF;
  }
};


struct TxPDO1
{
  uint16 status_word;
  int32 position_actual_value;
  int16 torque_actual_value;
  int32 following_error;
  int8 operation_mode_display;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    position_actual_value = 0x00000000;
    torque_actual_value = 0x0000;
    operation_mode_display = 0x00;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;

    torque_actual_value |= (0x00FF & *data_ptr++) << 0;
    torque_actual_value |= (0x00FF & *data_ptr++) << 8;

    following_error |= (0x000000FF & *data_ptr++) << 0;
    following_error |= (0x000000FF & *data_ptr++) << 8;
    following_error |= (0x000000FF & *data_ptr++) << 16;
    following_error |= (0x000000FF & *data_ptr++) << 24;

    operation_mode_display |= (0x00FF & *data_ptr++) << 0;
  }
};




// // PDO1 (Custom Synchronous Torque)
// struct RxPDO1
// {
//   uint16 control_word;
//   int32 target_position;
//   int16 target_torque;
//   int8 operation_mode;

//   void operator>>(uint8 *data_ptr)
//   {
//     *data_ptr++ = (control_word >> 0) & 0xFF;
//     *data_ptr++ = (control_word >> 8) & 0xFF;

//     *data_ptr++ = (target_position >> 0) & 0xFF;
//     *data_ptr++ = (target_position >> 8) & 0xFF;
//     *data_ptr++ = (target_position >> 16) & 0xFF;
//     *data_ptr++ = (target_position >> 24) & 0xFF;

//     *data_ptr++ = (target_torque >> 0) & 0xFF;
//     *data_ptr++ = (target_torque >> 8) & 0xFF;

//     *data_ptr++ = (operation_mode >> 0) & 0xFF;
//   }
// };


// struct TxPDO1
// {
//   uint16 status_word;
//   int32 position_actual_value;
//   int16 torque_actual_value;
//   int8 operation_mode_display;

//   void operator<<(uint8 *data_ptr)
//   {
//     status_word = 0x0000;
//     position_actual_value = 0x00000000;
//     torque_actual_value = 0x0000;
//     operation_mode_display = 0x00;

//     status_word |= (0x00FF & *data_ptr++) << 0;
//     status_word |= (0x00FF & *data_ptr++) << 8;

//     position_actual_value |= (0x000000FF & *data_ptr++) << 0;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 8;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 16;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 24;

//     torque_actual_value |= (0x00FF & *data_ptr++) << 0;
//     torque_actual_value |= (0x00FF & *data_ptr++) << 8;

//     operation_mode_display |= (0x00FF & *data_ptr++) << 0;
//   }
// };




// // PDO1 (Custom Synchronous Torque)
// struct RxPDO1
// {
//   uint16 control_word;
//   int16 target_torque;

//   void operator>>(uint8 *data_ptr)
//   {
//     *data_ptr++ = (control_word >> 0) & 0xFF;
//     *data_ptr++ = (control_word >> 8) & 0xFF;

//     *data_ptr++ = (target_torque >> 0) & 0xFF;
//     *data_ptr++ = (target_torque >> 8) & 0xFF;
//   }
// };


// struct TxPDO1
// {
//   uint16 status_word;
//   int32 position_actual_value;
//   int32 velocity_actual_value;
//   int16 torque_actual_value;
//   uint16 error_code;

//   void operator<<(uint8 *data_ptr)
//   {
//     status_word = 0x0000;
//     position_actual_value = 0x00000000;
//     velocity_actual_value = 0x00000000;
//     torque_actual_value = 0x0000;
//     error_code = 0x0000;

//     status_word |= (0x00FF & *data_ptr++) << 0;
//     status_word |= (0x00FF & *data_ptr++) << 8;

//     position_actual_value |= (0x000000FF & *data_ptr++) << 0;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 8;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 16;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 24;

//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 0;
//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 8;
//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 16;
//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 24;

//     torque_actual_value |= (0x00FF & *data_ptr++) << 0;
//     torque_actual_value |= (0x00FF & *data_ptr++) << 8;

//     error_code |= (0x00FF & *data_ptr++) << 0;
//     error_code |= (0x00FF & *data_ptr++) << 8;
//   }
// };



// PDO2 (Cyclic Synchronous Position)
struct RxPDO2
{
  uint16 control_word;
  int32 target_position;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_position >> 0) & 0xFF;
    *data_ptr++ = (target_position >> 8) & 0xFF;
    *data_ptr++ = (target_position >> 16) & 0xFF;
    *data_ptr++ = (target_position >> 24) & 0xFF;
  }
};

struct TxPDO2
{
  uint16 status_word;
  int32 position_actual_value;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    position_actual_value = 0x00000000;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;
  }
};

// PDO3 (Cyclic Synchronous Velocity)
struct RxPDO3
{
  uint16 control_word;
  int32 target_velocity;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_velocity >> 0) & 0xFF;
    *data_ptr++ = (target_velocity >> 8) & 0xFF;
    *data_ptr++ = (target_velocity >> 16) & 0xFF;
    *data_ptr++ = (target_velocity >> 24) & 0xFF;
  }
};

struct TxPDO3
{
  uint16 status_word;
  int32 position_actual_value;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    position_actual_value = 0x00000000;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;
  }
};

// PDO4 (Cyclic Synchronous Torque)
struct RxPDO4
{
  uint16 control_word;
  int16 target_torque;

  void operator>>(uint8 *data_ptr)
  {
    *data_ptr++ = (control_word >> 0) & 0xFF;
    *data_ptr++ = (control_word >> 8) & 0xFF;

    *data_ptr++ = (target_torque >> 0) & 0xFF;
    *data_ptr++ = (target_torque >> 8) & 0xFF;
  }
};



// struct TxPDO4
// {
//   uint16 status_word;
//   int32 position_actual_value;
//   int32 velocity_actual_value;
//   int16 torque_actual_value;

//   void operator<<(uint8 *data_ptr)
//   {
//     status_word = 0x0000;
//     position_actual_value = 0x00000000;
//     torque_actual_value = 0x0000;

//     status_word |= (0x00FF & *data_ptr++) << 0;
//     status_word |= (0x00FF & *data_ptr++) << 8;

//     position_actual_value |= (0x000000FF & *data_ptr++) << 0;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 8;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 16;
//     position_actual_value |= (0x000000FF & *data_ptr++) << 24;

//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 0;
//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 8;
//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 16;
//     velocity_actual_value |= (0x000000FF & *data_ptr++) << 24;

//     torque_actual_value |= (0x00FF & *data_ptr++) << 0;
//     torque_actual_value |= (0x00FF & *data_ptr++) << 8;
//   }
// };

struct TxPDO4
{
  uint16 status_word;
  int32 position_actual_value;
  int16 torque_actual_value;

  void operator<<(uint8 *data_ptr)
  {
    status_word = 0x0000;
    position_actual_value = 0x00000000;
    torque_actual_value = 0x0000;

    status_word |= (0x00FF & *data_ptr++) << 0;
    status_word |= (0x00FF & *data_ptr++) << 8;

    position_actual_value |= (0x000000FF & *data_ptr++) << 0;
    position_actual_value |= (0x000000FF & *data_ptr++) << 8;
    position_actual_value |= (0x000000FF & *data_ptr++) << 16;
    position_actual_value |= (0x000000FF & *data_ptr++) << 24;

    torque_actual_value |= (0x00FF & *data_ptr++) << 0;
    torque_actual_value |= (0x00FF & *data_ptr++) << 8;
  }
};




} } } // namespace
#endif
