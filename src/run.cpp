// Boost
#include <boost/program_options.hpp>
// ROS
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "yaskawa_ethercat/MotorCommands.h"
// Custom header
#include "ethercat/master.h"
// Math
#include <math.h>
// Time
#include "time/time.h"

// Constants
const int BIT_RESOLUTION = pow(2, 20); // For 20-bit encoder resolution

// Structure to pass arguments to control_loop function
struct control_thread_struct {
    ros::Publisher pub;
    sensor_msgs::JointState msg;
    yaskawa::ethercat::Master* master;
};

// Helper function to calculate position in radians
float64 calculatePositionInRadians(int32 position_actual_value, float64 gear_ratio) {
    return position_actual_value * 2 * M_PI / (gear_ratio * BIT_RESOLUTION);
}

// Function to check and handle motor faults
void checkAndHandleFaults(const uint16 status_word, int slave_idx) {
    std::string binary_status = yaskawa::ethercat::toBinary(status_word);
    int str_len = binary_status.length();
    std::string fault_string = binary_status.substr(str_len - 5);

    if (binary_status[str_len - 8] == '1' || fault_string == "01000") {
        perror("ERROR OCCURRED!");
        int ret1 = 11;
        pthread_exit(&ret1);
    }
}

// Function to perform state change checks and actions
bool performStateChangeCheck(yaskawa::ethercat::Master* ec_master, const std::string& action_message, bool (yaskawa::ethercat::Master::*stateChangeFunction)()) {
    std::cout << action_message << " ... ";
    if ((ec_master->*stateChangeFunction)()) {
        std::cout << "SUCCESS" << std::endl;
        return true;
    } else {
        std::cout << "FAILURE" << std::endl;
        return false;
    }
    return true;
}

// Control loop function
void* control_loop(void* arg) {
    // Extracting control loop parameters
    struct control_thread_struct *loop_structure = (struct control_thread_struct *)arg;
    yaskawa::ethercat::Master* ec_master = loop_structure->master;
    ros::Publisher pub = loop_structure->pub;
    sensor_msgs::JointState msg = loop_structure->msg;

    // Timekeeping variables
    struct timespec t, t_1;
    clock_gettime(CLOCK_MONOTONIC, &t);

    // Initial motor position readout
    for (int i = 0; i < ec_slavecount; i++) {
        const uint16 slave_idx = 1 + i;
        int32 position_actual_value = ec_master->tx_pdo[slave_idx].position_actual_value;
        float64 position_actual_rad = calculatePositionInRadians(position_actual_value, yaskawa::ethercat::gear_ratio[i]);
        ec_master->rx_pdo[slave_idx].target_position = position_actual_value;
        printf("Motor %d Position (rad): %f\n", slave_idx, position_actual_rad);
    }
    ec_master->update();

    // 
    if (!performStateChangeCheck(ec_master, "Fault Reset", &yaskawa::ethercat::Master::fault_reset)) return nullptr;
    if (!performStateChangeCheck(ec_master, "Ready to Switch On", &yaskawa::ethercat::Master::ready_to_switch_on)) return nullptr;
    if (!performStateChangeCheck(ec_master, "Switch On", &yaskawa::ethercat::Master::switch_on)) return nullptr;
    if (!performStateChangeCheck(ec_master, "Enable Operation", &yaskawa::ethercat::Master::enable_operation)) return nullptr;

    // Main control loop
    while (true) {
        // Timing and loop management
        yaskawa::ethercat::add_timespec(&t, ec_master->t_cycle + ec_master->t_off);
        struct timespec t_left;
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, &t_left);

        // Get motor state and update message
        for (int i = 0; i < ec_slavecount; i++) {
            const uint16 slave_idx = 1 + i;
            auto& tx_pdo = ec_master->tx_pdo[slave_idx];
            uint16 status_word = tx_pdo.status_word;
            checkAndHandleFaults(status_word, slave_idx);

            int32 position_actual_value = tx_pdo.position_actual_value;
            float64 position_actual_rad = calculatePositionInRadians(position_actual_value, yaskawa::ethercat::gear_ratio[i]);
            int16 torque_actual_value = ec_master->tx_pdo[slave_idx].torque_actual_value;   // In units of 0.1% of rated torque
            float64 torque_actual_nm = torque_actual_value*yaskawa::ethercat::rated_torque[i]/(100*yaskawa::ethercat::torque_denominator);
            msg.position[i] = position_actual_rad;
            msg.effort[i] = torque_actual_nm;
            msg.header.stamp = ros::Time::now();
        }

        pub.publish(msg);
        // ROS update
        ros::spinOnce();
        ec_master->update();
        t_1 = t;
    }

    std::cout << "Finished." << std::endl;
    return nullptr;
}

int main(int argc, char* argv[]) {
    // Command-line options
    std::string ifname;
    std::vector<std::string> slaves;

    boost::program_options::options_description opt("Allowed options");
    opt.add_options()
        ("help,h", "Print this help and exit.")
        ("ifname,i", boost::program_options::value<std::string>(&ifname), "EtherCAT interface to use.")
        ("slaves,s", boost::program_options::value<std::vector<std::string>>(&slaves), "EtherCAT slaves.");

    boost::program_options::positional_options_description pos_opt;
    pos_opt.add("ifname", 1).add("slaves", -1);

    boost::program_options::variables_map var;
    boost::program_options::store(boost::program_options::command_line_parser(argc, argv).options(opt).positional(pos_opt).run(), var);
    boost::program_options::notify(var);

    if (var.count("help")) {
        std::cout << "Usage: run [options] <ifname> <slaves>...\n" << opt << std::endl;
        return EXIT_FAILURE;
    }

    // EtherCAT Master Initialization
    yaskawa::ethercat::Master ec_master(4000000U, ifname, slaves);
    if (!ec_master.init()) {
        std::cerr << "Failed to initialize EtherCAT Master" << std::endl;
        return EXIT_FAILURE;
    }

    // ROS Initialization
    ros::init(argc, argv, "motor_control"); 
    ros::NodeHandle n;

    // ROS Publisher for motor states
    ros::Publisher motor_states_pub = n.advertise<sensor_msgs::JointState>("motor_states", 10);
    sensor_msgs::JointState msg; 

    msg.name.resize(slaves.size());
    msg.position.resize(slaves.size()); 
    msg.effort.resize(slaves.size()); 

    // Assign motor names (assuming 6 motors)
    std::vector<std::string> motor_names = {"motor1", "motor2", "motor3", "motor4", "motor5", "motor6"};
    for (size_t i = 0; i < msg.name.size(); ++i) {
        if (i < motor_names.size()) {
            msg.name[i] = motor_names[i];
        }
    }

    // ROS Subscriber for motor commands
    ros::Subscriber sub = n.subscribe("motor_commands", 10, &yaskawa::ethercat::Master::commandUpdateCallback, &ec_master);

    // Start EtherCAT Master
    if (!ec_master.start()) {
        std::cerr << "Failed to start EtherCAT Master" << std::endl;
        return EXIT_FAILURE;
    }

    // POSIX Thread for Control Loop
    pthread_t pthread;
    pthread_attr_t pthread_attr;

    // Set up thread attributes
    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set); 
    CPU_SET(1, &cpu_set);
    sched_param sched_param { .sched_priority = 90 };

    if (pthread_attr_init(&pthread_attr) ||
        pthread_attr_setaffinity_np(&pthread_attr, sizeof(cpu_set), &cpu_set) ||
        pthread_attr_setinheritsched(&pthread_attr, PTHREAD_EXPLICIT_SCHED) ||
        pthread_attr_setschedpolicy(&pthread_attr, SCHED_RR) ||
        pthread_attr_setschedparam(&pthread_attr, &sched_param)) {
        perror("Thread attribute setup failed");
        return EXIT_FAILURE;
    }

    // Control Loop Structure
    struct control_thread_struct loop_struct { motor_states_pub, msg, &ec_master };

    // Run Control Loop in Thread
    if (pthread_create(&pthread, &pthread_attr, &control_loop, &loop_struct)) {
        perror("Failed to create control loop thread");
        return EXIT_FAILURE;
    }

    // Wait for Thread to Finish
    int returnValue;
    pthread_join(pthread, (void **)&returnValue);

    if (returnValue != 2056482076) {
        std::cerr << "Control loop thread terminated with unexpected value" << std::endl;
    }

    // Clean up
    ec_master.close();
    pthread_attr_destroy(&pthread_attr);

    return EXIT_SUCCESS;
}