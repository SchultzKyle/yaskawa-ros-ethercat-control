// Boost
#include <boost/program_options.hpp>
// ROS
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "control/MotorCommands.h"
// Custom header
#include "ethercat/master.h"
// Math
#include <math.h>
// Time
#include "ethercat/time.h"

// Structure to pass arguments to control_loop function
struct control_thread_struct {
    ros::Publisher pub;
    sensor_msgs::JointState msg;
    yaskawa::ethercat::Master* master;
};


void* control_loop(void* arg)
{

  struct control_thread_struct *loop_structure = (struct control_thread_struct *)arg;

  yaskawa::ethercat::Master* ec_master = loop_structure -> master;
  ros::Publisher pub = loop_structure -> pub;
  sensor_msgs::JointState msg = loop_structure -> msg;

  struct timespec t, t_1, t0_cmd;
  clock_gettime(CLOCK_MONOTONIC, &t);

  for (int i = 0; i < ec_slavecount; i++)
  {
    const uint16 slave_idx = 1 + i;
    int32 position_actual_value = ec_master->tx_pdo[slave_idx].position_actual_value;
    float64 position_actual_rad = position_actual_value*2*M_PI/(yaskawa::ethercat::gear_ratio[i]*pow(2, 20));
    ec_master->rx_pdo[slave_idx].target_position = position_actual_value;
    printf("Motor %d Position (rad): %f\n", slave_idx, position_actual_rad);
  }
  ec_master->update();


  int iter = 0;
  int fault = 0;

  while (true)
  {
    yaskawa::ethercat::add_timespec(&t, ec_master->t_cycle + ec_master->t_off);
    struct timespec t_left;
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t_left);

    struct timespec t_period;
    yaskawa::ethercat::diff_timespec(t, t_1, &t_period);

    iter++;
    if (iter == 10)
    {
      std::cout << "Fault Reset ... ";
      if (ec_master->fault_reset())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 20)
    {
      std::cout << "Ready to Switch On ... ";
      if (ec_master->ready_to_switch_on())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 30)
    {
      std::cout << "Switch On ... ";
      if (ec_master->switch_on())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 40)
    {
      std::cout << "Enable Operation ... ";
      if (ec_master->enable_operation())
      {
        std::cout << "SUCCESS" << std::endl;
      }
      else
      {
        std::cout << "FAILURE" << std::endl;
        return 0;
      }
    }

    if (iter == 50)
    {
      printf("Starting control loop...\n");
    }
    if (iter >= 50)
    {
      for (int i = 0; i < ec_slavecount; i++)
      {
        const uint16 slave_idx = 1 + i;
        uint16 status_word = ec_master->tx_pdo[slave_idx].status_word;
        int32 position_actual_value = ec_master->tx_pdo[slave_idx].position_actual_value;
        int16 torque_actual_value = ec_master->tx_pdo[slave_idx].torque_actual_value;   // In units of 0.1% of rated torque
        int32 following_error = ec_master->tx_pdo[slave_idx].following_error;
        int8 operation_mode_display = ec_master->tx_pdo[slave_idx].operation_mode_display;
        std::string stat_word_bin = yaskawa::ethercat::toBinary(status_word);
        int str_len = stat_word_bin.length();
        std::string fault_string = "";
        fault_string += stat_word_bin[str_len - 7]; 
        fault_string += stat_word_bin[str_len - 4];  
        fault_string += stat_word_bin[str_len - 3]; 
        fault_string += stat_word_bin[str_len - 2]; 
        fault_string += stat_word_bin[str_len - 1];

        if (stat_word_bin[str_len - 8] == '1' || fault_string == "01000")
        {
          perror("ERROR OCCURRED!");
          int ret1  = 11;
          pthread_exit(&ret1);
        }

        float64 torque_actual_nm = torque_actual_value*yaskawa::ethercat::rated_torque[i]/(100*yaskawa::ethercat::torque_denominator);

        if (abs(torque_actual_value) >= 0.99*yaskawa::ethercat::torque_limits[i])
        {
          printf("Motor %d torque saturating with torque %f Nm \n", slave_idx, torque_actual_nm);
        }
        float64 position_actual_rad = position_actual_value*2*M_PI/(yaskawa::ethercat::gear_ratio[i]*pow(2, 20));  // Calculate motor positon in radians, 20-bit encoder
        float64 following_error_rad = following_error*2*M_PI/(yaskawa::ethercat::gear_ratio[i]*pow(2, 20));  // Calculate motor positon in radians, 20-bit encoder

        msg.position[i] = position_actual_rad;
        msg.effort[i] = torque_actual_nm;
        msg.header.stamp = ros::Time::now();

        int32 target_position = ec_master->rx_pdo[slave_idx].target_position; // target position is updated from callback in ec_master class
        float64 target_position_rad = target_position*2*M_PI/(yaskawa::ethercat::gear_ratio[i]*pow(2, 20));
      }
    }

    pub.publish(msg);
    ros::spinOnce();

    ec_master->update();
    t_1 = t;
  }

  std::cout << "Finished." << std::endl;
}

int main(int argc, char* argv[])
{
  // EtherCAT
  std::string ifname;
  std::vector<std::string> slaves;
  int limit;

  boost::program_options::options_description opt("Allowed options");
  opt.add_options()
    ("help,h", "Print this help and exit.")
    ("ifname,i", boost::program_options::value<std::string>(&ifname), "EtherCAT interface to use.")
    ("slaves,s", boost::program_options::value<std::vector<std::string>>(&slaves), "EtherCAT slaves.");

  boost::program_options::positional_options_description pos_opt;
  pos_opt.add("ifname", 1);
  pos_opt.add("slaves", -1);

  boost::program_options::command_line_parser cmd_line_parser(argc, argv);
  cmd_line_parser.options(opt);
  cmd_line_parser.positional(pos_opt);
  boost::program_options::variables_map var;
  boost::program_options::store(cmd_line_parser.run(), var);
  boost::program_options::notify(var);

  if (var.count("help"))
  {
    std::cout << "Usage: run [options] <ifname> <slaves>...\n"
              << opt
              << std::endl;
    return 1;
  }

  // Init
  // Timeout needs to be 1-32 multiple of 125000ns, was 2000000U
  for(int i = 0; i < slaves.size(); i++)
    std::cout << slaves[i] << ' ';
  
  // std::cout << ifname;
  while (true)
  {
    yaskawa::ethercat::Master ec_master(4000000U, ifname, slaves);

    if (!ec_master.init())
    {
      return 1;
    }

    // ROS initialization
    ros::init(argc, argv, "motor_control"); 
    ros::NodeHandle n;

    // ROS Publisher for motor angles (encoder)
    ros::Publisher motor_states_pub = n.advertise<sensor_msgs::JointState>("motor_states", 10);
    sensor_msgs::JointState msg; 

    msg.name.resize(6);                 // 6 slaves
    msg.position.resize(6); 
    msg.effort.resize(6); 

    msg.name[0] = "motor1";
    msg.name[1] = "motor2";
    msg.name[2] = "motor3";
    msg.name[3] = "motor4";
    msg.name[4] = "motor5";
    msg.name[5] = "motor6";

    // ROS Subscriber of positon commands, target position is updated from callback in ec_master class
    ros::Subscriber sub = n.subscribe("motor_commands", 10, &yaskawa::ethercat::Master::commandUpdateCallback, &ec_master);

    // Start

    const int n_slaves = slaves.size();
    for (int i = 0; i < n_slaves; i++)
    {
      const uint16 slave_idx = 1 + i;
    }

    if (!ec_master.start())
    {
      return 1;
    }

    // POSIX Thread
    pthread_t pthread;
    pthread_attr_t pthread_attr;

    errno = pthread_attr_init(&pthread_attr);
    if (errno != 0)
    {
      perror("pthread_attr_init");
      return 1;
    }

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set); CPU_SET(1, &cpu_set);
    errno = pthread_attr_setaffinity_np(&pthread_attr, sizeof(cpu_set), &cpu_set);
    if (errno != 0)
    {
      perror("pthread_attr_setaffinity_np");
      return 1;
    }

    errno = pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED);
    if (errno != 0)
    {
      perror("%d, pthread_attr_setschedpolicy");
      return 1;
    }

    errno = pthread_attr_setschedpolicy(&pthread_attr, SCHED_RR);
    if (errno != 0)
    {
      perror("pthread_attr_setschedpolicy");
      return 1;
    }

    sched_param sched_param
    {
      .sched_priority = 99
    };
    errno = pthread_attr_setschedparam(&pthread_attr, &sched_param);
    if (errno != 0)
    {
      perror("pthread_attr_setschedparam");
      return 1;
    }

    struct control_thread_struct loop_struct;
    loop_struct.pub = motor_states_pub;
    loop_struct.msg = msg;
    loop_struct.master = &ec_master;


    // Enter control loop
    printf("Restarting thread\n");
    errno = pthread_create(&pthread, &pthread_attr, &control_loop, &loop_struct);
    if (errno != 0)
    {
      perror("pthread_create");
      return 1;
    }

    int returnValue;

    pthread_join(pthread, (void **)&returnValue);

    if (returnValue != 2056482076)
    {
      perror("closing master 1");
      ec_master.close();
      perror("closed master 1");
    }
    
    perror("restarting thread");

    errno = pthread_attr_destroy(&pthread_attr);
    if (errno != 0)
    {
      perror("pthread_attr_destroy");
      return 1;
    }
    perror("closing master 2");
    ec_master.close();
    perror("closed master 2");
  }
  return 0;
}
