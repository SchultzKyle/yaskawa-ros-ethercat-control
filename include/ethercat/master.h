#ifndef ESA_EWDL_ETHERCAT_MASTER_H
#define ESA_EWDL_ETHERCAT_MASTER_H
#include <cstring>
#include <string>
#include <vector>
// SOEM
#include "ethercat.h"
// Custom headers
#include "ethercat/common.h"
#include "ethercat/registry_idx.h"
#include "ethercat/pdo.h"


namespace yaskawa { namespace ethercat {

inline int slave_setup(uint16 slave_idx)
{

    /* From Yaskawa manual:

  Setting Procedure for PDO Mappings:

  1. Disable the assignments between the Sync Manager and PDOs.
  (Set subindex 0 of objects 1C12h to 1C13h to 0.)

  2. Set all of the mapping entries for the PDO mapping objects.
  (Set objects 1600h to 1603h and 1A00h to 1A03h.)

  3. Set the number of mapping entries for the PDO mapping objects.
  (Set subindex 0 of objects 1600h to 1603h and 1A00h to 1A03h.)

  4. Set the assignments between the Sync Manager and PDOs.
  (Set subindex 1 of objects 1C12h to 1C13h.)

  5. Enable the assignments between the Sync Manager and PDOs.
  (Set subindex 0 of objects 1C12h to 1C13h to 1.)

  The PDO mapping objects (indexes 1600h to 1603h and 1A00h to 1A03h) and the Sync
  Manager PDO assignment objects (index 1C12h and 1C13h) can be written only in Pre-
  Operational state.

  */

  int wkc = 0;

  // Step 1:
  uint8 disable = 0x00;
  wkc += writeSDO<uint8>(slave_idx, 0x1c12, 0x00, disable);
  wkc += writeSDO<uint8>(slave_idx, 0x1c13, 0x00, disable);

  // // Step 2:
  // uint32 control_word_idx = 0x6040;
  // uint32 target_position_idx = 0x607a;
  // uint32 target_torque_idx = 0x6071;
  // uint32 operation_mode_idx = 0x6060;

  // wkc += writeSDO<uint32>(slave_idx, 0x1600, 0x01, control_word_idx);
  // wkc += writeSDO<uint32>(slave_idx, 0x1600, 0x02, target_position_idx);
  // wkc += writeSDO<uint32>(slave_idx, 0x1600, 0x03, target_torque_idx);
  // wkc += writeSDO<uint32>(slave_idx, 0x1600, 0x04, operation_mode_idx);


  // uint32 status_word_idx = 0x6041;
  // uint32 position_actual_value_idx = 0x6064;
  // uint32 torque_actual_value_idx = 0x6077;
  // uint32 operation_mode_display_idx = 0x6061;

  // wkc += writeSDO<uint32>(slave_idx, 0x1a00, 0x01, status_word_idx);
  // wkc += writeSDO<uint32>(slave_idx, 0x1a00, 0x02, position_actual_value_idx);
  // wkc += writeSDO<uint32>(slave_idx, 0x1a00, 0x03, torque_actual_value_idx);
  // wkc += writeSDO<uint32>(slave_idx, 0x1a00, 0x04, operation_mode_display_idx);

  // Step 3:
  uint8 num_objects_rx = 6;
  uint8 num_objects_tx = 5;
  wkc += writeSDO<uint8>(slave_idx, 0x1600, 0x00, num_objects_rx);
  wkc += writeSDO<uint8>(slave_idx, 0x1a00, 0x00, num_objects_tx);


  // Step 4:
  uint16 rx_pdo1 = 0x1600;
  uint16 tx_pdo1 = 0x1a00; 
  wkc += writeSDO<uint16>(slave_idx, 0x1c12, 0x01, rx_pdo1);
  wkc += writeSDO<uint16>(slave_idx, 0x1c13, 0x01, tx_pdo1);

  // Step 5
  uint8 enable = 1;
  wkc += writeSDO<uint8>(slave_idx, 0x1c12, 0x00, enable);
  wkc += writeSDO<uint8>(slave_idx, 0x1c13, 0x00, enable);

  return wkc;
}

class Master {
private:
  const static size_t MAX_IO_MAP_SIZE = 4096;

  int ec_state = EC_STATE_NONE;
  int wkc = 0;
  uint8 io_map[MAX_IO_MAP_SIZE];

  std::string ifname;
  std::vector<std::string> slaves;

  bool network_configuration()
  {
    for (int i = 0; i < slaves.size(); i++)
    {
      const uint16 slave_idx = 1 + i;
      if (strcmp(ec_slave[slave_idx].name, slaves[i].c_str()))
      {
        return false;
      }
    }
    return true;
  }

  void ec_sync(uint64 t_ref, uint64 cycletime, int32 *t_off)
  {
    static int32 integral = 0;
    int32 delta = (t_ref - (cycletime / 2)) % cycletime;

    if (delta > (cycletime / 2))
    {
      delta = delta - cycletime;
    }
    if (delta > 0) integral++;
    if (delta < 0) integral--;

    *t_off = -(delta / 100) - (integral / 20);
  }

public:
  unsigned int t_cycle; int t_off = 0;

  yaskawa::ethercat::pdo::RxPDO1 rx_pdo[7];
  yaskawa::ethercat::pdo::TxPDO1 tx_pdo[7];

  Master() { }

  Master(unsigned int cycletime, const std::string &ifname, const std::vector<std::string> &slaves) :
    t_cycle(cycletime),
    ifname(ifname),
    slaves(slaves) { }


  bool init()
  {
    if (ec_init(ifname.c_str()) > 0)
    {
      printf("EtherCAT socket on: %s\n", ifname.c_str());
    }
    else
    {
      printf("Couldn't initialize EtherCAT Master socket on: %s\n", ifname.c_str());
      return false;
    }

    if (ec_config_init(FALSE) > 0)
    {
      printf("Slaves found and configured: %d\n", ec_slavecount);
    }
    else
    {
      printf("Couldn't find and configure any slave.\n");
      return false;
    }

    // PRE OPERATIONAL
    ec_state = ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);

    // network configuration
    if (!network_configuration())
    {
      printf("Mismatch of network units!\n");
      return false;
    }

    // Distributed Clock
    ec_configdc();
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      ec_dcsync0(slave_idx, TRUE, t_cycle, 0);
    }

    // Pre-Operational -> Safe-Operational
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      set_torque_unit(slave_idx, yaskawa::ethercat::torque_denominator);
      ec_slave[slave_idx].PO2SOconfig = slave_setup;
    }

    int used_mem = ec_config_map(&io_map);
    if (used_mem > sizeof(io_map))
    {
      printf("IO Map size: %d > MAX_IO_MAP_SIZE: %lu\n", used_mem, sizeof(io_map));
      return false;
    }
    printf("io_map size: %d\n", used_mem);


    // SAFE OPERATIONAL
    ec_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
    print_ec_state(0);


    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      fault_reset(slave_idx);
      clear_alarm(slave_idx);

      set_mode_of_operation(slave_idx, mode_of_operation_t::CYCLIC_SYNCHRONOUS_POSITION);
      set_torque_limit(slave_idx, yaskawa::ethercat::torque_limits[i]);
    }

    return true;
  }

    // ROS Subscriber Callback Function

  /* Class method for subscriber callback function initialized in main.
   * This updates the target velocities for the motors to follow from
   * kinematics via a ROS message. */

  void commandUpdateCallback(const control::MotorCommands msg)
  {
    int8 torque_mode = 10;
    int8 position_mode = 8;

    for (int i = 0; i < ec_slavecount; i++)
    {
    const uint16 slave_idx = 1 + i;
    int32_t pos = msg.displacements[i]*yaskawa::ethercat::gear_ratio[i]*pow(2, 20)/(2*M_PI);  // Convert rad to count
    rx_pdo[slave_idx].target_position  = pos;

    int16 torque = msg.torques[i]*100*yaskawa::ethercat::torque_denominator/yaskawa::ethercat::rated_torque[i];
    rx_pdo[slave_idx].target_torque = torque;
    rx_pdo[slave_idx].max_torque = yaskawa::ethercat::torque_limits[i];

    rx_pdo[slave_idx].operation_mode = msg.modes[i];

    // if (i==0)
    // {
    // printf("Motor: %d PDO Target Position (Counts): %d PDO Target Torque (Unit): %d \n",
    //       slave_idx,  rx_pdo[slave_idx].target_position, rx_pdo[slave_idx].target_torque);
    // }


    }
  }


  // Fault Reset

  /* When Fault happens, after the condition that caused the error has been
   * resolved, write 80h to object 0x6040 to clear the error code in object
   * 0x603F and object 0x200F. */

  int fault_reset(uint16 slave_idx)
  {
    uint16 control_word = 0x0080;
    wkc += writeSDO<uint16>(slave_idx, CONTROL_WORD_IDX, 0x00, control_word);
    return wkc;
  }

  // Clear Alarm

  /* When Warning happens, after the condition that caused the error has been
   * resolved, write 01h to object 0x2006 to clear the error code in object
   * 0x603F and object 0x200F. */

  int clear_alarm(uint16 slave_idx)
  {
    uint8 clear_alarm = 0x01;
    wkc += writeSDO<uint8>(slave_idx, CLEAR_ALARM_IDX, 0x00, clear_alarm);
    return wkc;
  }


  bool start()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;

      rx_pdo[slave_idx].control_word = 0x0006;
      // rx_pdo[slave_idx].target_position = 0;
    }

    update();

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
      ec_writestate(slave_idx);
    }

    // OPERATIONAL
    ec_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
    print_ec_state(0);

    return true;
  }


  int update()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx] >> ec_slave[slave_idx].outputs;
    }

    ec_send_processdata();
    wkc += ec_receive_processdata(EC_TIMEOUTRET);

    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      tx_pdo[slave_idx] << ec_slave[slave_idx].inputs;
    }

    ec_sync(ec_DCtime, t_cycle, &t_off);
    
    return wkc;
  }


  void close()
  {
    ec_close();
  }


  bool fault_reset()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word |= 0x0080;
    }
    return true;
  }


  bool ready_to_switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0006;
    }
    return true;
  }


  bool switch_on()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x0007;
    }
    return true;
  }


  bool switch_off()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word &= 0xFFFE;
    }
    return true;
  }


  bool enable_operation()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;
      rx_pdo[slave_idx].control_word = 0x000F;
    }
    return true;
  }


  int set_mode_of_operation(uint16 slave_idx, mode_of_operation_t mode_of_operation)
  {
    wkc = 0;
    rx_pdo[slave_idx].control_word = 0x000F;
    wkc += writeSDO<int8>(slave_idx, MODE_OF_OPERATION_IDX, 0x00, mode_of_operation);
    rx_pdo[slave_idx].operation_mode = mode_of_operation;
    return wkc;
  }


  bool halt()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word |= 0x0100;
    }
    return true;
  }


  bool quick_stop()
  {
    for (uint16 slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++)
    {
      rx_pdo[slave_idx].control_word &= 0b1111111111111011;
    }
    return true;
  }

  int set_torque_unit(uint16 slave_idx, uint32 denominator)
  {
    // Sets denominator of torque user unit - allows for finer torque resolution

    /*To change units, from Yaskawa manual 15-17:
    1. Change the SERVOPACK to the Switch ON Disabled state.
    2. Set the new parameter settings.
    3. Set user parameter configuration (2700h) to 1. The parameter settings will be enabled.
    */

    uint32 parameter_config = 1;
    wkc = 0;
    // rx_pdo[slave_idx].control_word = 0x000F;
    wkc += writeSDO<uint32>(slave_idx, TORQUE_USER_UNIT_IDX, 0x02, denominator);
    wkc += writeSDO<uint32>(slave_idx, 0x2700, 0x00, parameter_config);
    return wkc;
  }

  int set_torque_limit(const uint16 slave_idx, uint16 max_torque)
  {
    
    wkc += writeSDO<uint16>(slave_idx, MAX_TORQUE, 0x00, max_torque);
    return wkc;
  }



  int get_error_code(const uint16 slave_idx, uint16 &error_code)
  {
    wkc += readSDO<uint16>(slave_idx, ERROR_CODE_IDX, 0x00, error_code);
    return wkc;
  }


  int set_zero_position()
  {
    for (int i = 0; i < ec_slavecount; i++)
    {
      const uint16 slave_idx = 1 + i;

      uint8 zero_position = 0x01;
      wkc += writeSDO<uint8>(slave_idx, ZERO_POSITION_IDX, 0x00, zero_position);

      zero_position = 0x00;
      wkc += writeSDO<uint8>(slave_idx, ZERO_POSITION_IDX, 0x00, zero_position);
    }

    return wkc;
  }


  int get_alarm_code(const uint16 slave_idx, uint32 &alarm_code)
  {
    wkc += readSDO<uint32>(slave_idx, ALARM_CODE_IDX, 0x00, alarm_code);
    return wkc;
  }

};

} }  // namespace
#endif
