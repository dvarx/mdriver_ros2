//ros related includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include "tnb_mns_driver/msg/tnb_mns_status.hpp"
#include "tnb_mns_driver/srv/enable.hpp"
#include "tnb_mns_driver/srv/run_regular.hpp"
#include "tnb_mns_driver/srv/run_resonant.hpp"
#include "tnb_mns_driver/srv/stop.hpp"
#include "tnb_mns_driver/srv/tnbmns_state_transition.hpp"
//#include "tnb_mns_driver/TNBMNSStateTransition.h"
#include "statemachine.h"

//ethernet / ip related includes
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <linux/tcp.h>
#include <netinet/in.h>
#include <errno.h>
#include <string.h>

#include <signal.h>

#define NO_CHANNELS 6
#define DEFAULT_RES_FREQ_MILLIHZ 10000000
//this defines the downsampling for the TNB_MNS_STATUS topics
#define TNB_MNS_STATUS_DOWNSAMPLE 10

rclcpp::Publisher<tnb_mns_driver::msg::TnbMnsStatus>::SharedPtr tnb_mns_state_publisher;
bool hardware_connected=true;


//struct for tnb_mns_messages
struct tnb_mns_msg{
    int16_t desCurrents[NO_CHANNELS];
    uint16_t desCurrentsRes[NO_CHANNELS];
    uint16_t desDuties[NO_CHANNELS];
    uint32_t desFreqs[NO_CHANNELS];
    uint16_t stp_flg_byte;
    uint16_t buck_flg_byte;
    uint16_t regen_flg_byte;
    uint16_t  resen_flg_byte;
};

//struct for receiving the state of the TNB MNS system
struct tnb_mns_msg_sysstate{
    uint16_t states[NO_CHANNELS];
    int16_t currents[NO_CHANNELS];          // [mA]
    uint16_t duties[NO_CHANNELS];
    uint32_t freqs[NO_CHANNELS];
    int16_t dclink_voltages[NO_CHANNELS];      // [mA]
};

//tcp variables
int sock_cli;                       //socket fd
struct sockaddr_in servaddr;        //address struct
#define TNB_MNS_PORT  30
#define BUFFER_SIZE 1024
uint8_t send_buffer[1024];
const float send_interval=10e-3;            //defines the timerinterval of the send timer
const float pre_filter_tau=500e-3;           //time constant of the prefilter to limit energy feedback
const unsigned int packets_per_second=(unsigned int)(1.0/send_interval);

//topic subscribers
//ros::Subscriber des_currents_reg_subs;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr des_currents_reg_subs;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr des_currents_res_subs;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr des_freqs_res;
rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr des_duties_subs;

//service handles
//ros::ServiceServer srv_enable_tnb_mns_driver;
rclcpp::Service<tnb_mns_driver::srv::Enable>::SharedPtr srv_enable_tnb_mns_driver;
rclcpp::Service<tnb_mns_driver::srv::RunRegular>::SharedPtr srv_run_regular;
rclcpp::Service<tnb_mns_driver::srv::RunResonant>::SharedPtr srv_run_resonant;
rclcpp::Service<tnb_mns_driver::srv::Stop>::SharedPtr srv_stop_tnb_mns_driver;

//state variables
bool enable_input_lowpass=true;
double m_des_currents_mA[NO_CHANNELS]={0};
double m_des_currents_mA_filtered[NO_CHANNELS]={0};
double m_des_currents_mA_prev[NO_CHANNELS]={0};
double m_des_currents_res_mA[NO_CHANNELS];
double m_des_duties[NO_CHANNELS];
uint32_t m_des_freqs_mhz[NO_CHANNELS];
bool m_stop_flags[NO_CHANNELS];
bool m_buck_flags[NO_CHANNELS];
bool m_run_reg_flags[NO_CHANNELS];
bool m_run_res_flags[NO_CHANNELS];
struct tnb_mns_msg m_next_message={0};
struct tnb_mns_msg m_last_message={0};
struct tnb_mns_msg_sysstate m_last_system_report={0};

//other constants
const double max_current=25.0;
const double max_current_res=5.0;

#include <sys/socket.h>
#include <fcntl.h>
#include <poll.h>
#include <time.h>

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Shutdown using SIGINT");
  //ROS_INFO("Shutdown using SIGINT");

  close(sock_cli);

  //ros::Duration(0.5).sleep();
  rclcpp::sleep_for(std::chrono::milliseconds(500));

  // All the default sigint handler does is call shutdown()
  //ros::shutdown();
  rclcpp::shutdown();
}

int connect_with_timeout(int sockfd, const struct sockaddr *addr, socklen_t addrlen, unsigned int timeout_ms) {
    int rc = 0;
    // Set O_NONBLOCK
    int sockfd_flags_before;
    if((sockfd_flags_before=fcntl(sockfd,F_GETFL,0)<0)) return -1;
    if(fcntl(sockfd,F_SETFL,sockfd_flags_before | O_NONBLOCK)<0) return -1;
    // Start connecting (asynchronously)
    do {
        if (connect(sockfd, addr, addrlen)<0) {
            // Did connect return an error? If so, we'll fail.
            if ((errno != EWOULDBLOCK) && (errno != EINPROGRESS)) {
                rc = -1;
            }
            // Otherwise, we'll wait for it to complete.
            else {
                // Set a deadline timestamp 'timeout' ms from now (needed b/c poll can be interrupted)
                struct timespec now;
                if(clock_gettime(CLOCK_MONOTONIC, &now)<0) { rc=-1; break; }
                struct timespec deadline = { .tv_sec = now.tv_sec,
                                             .tv_nsec = now.tv_nsec + timeout_ms*1000000l};
                // Wait for the connection to complete.
                do {
                    // Calculate how long until the deadline
                    if(clock_gettime(CLOCK_MONOTONIC, &now)<0) { rc=-1; break; }
                    int ms_until_deadline = (int)(  (deadline.tv_sec  - now.tv_sec)*1000l
                                                  + (deadline.tv_nsec - now.tv_nsec)/1000000l);
                    if(ms_until_deadline<0) { rc=0; break; }
                    // Wait for connect to complete (or for the timeout deadline)
                    struct pollfd pfds[] = { { .fd = sockfd, .events = POLLOUT } };
                    rc = poll(pfds, 1, ms_until_deadline);
                    // If poll 'succeeded', make sure it *really* succeeded
                    if(rc>0) {
                        int error = 0; socklen_t len = sizeof(error);
                        int retval = getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &error, &len);
                        if(retval==0) errno = error;
                        if(error!=0) rc=-1;
                    }
                }
                // If poll was interrupted, try again.
                while(rc==-1 && errno==EINTR);
                // Did poll timeout? If so, fail.
                if(rc==0) {
                    errno = ETIMEDOUT;
                    rc=-1;
                }
            }
        }
    } while(0);
    // Restore original O_NONBLOCK state
    if(fcntl(sockfd,F_SETFL,sockfd_flags_before)<0) return -1;
    // Success
    return rc;
}

void reset_currents(){
    //reset all currents to zero
    for(int i=0; i<NO_CHANNELS; i++){
        m_des_currents_mA_prev[i]=0.0;
        m_des_currents_mA_filtered[i]=0.0;
        m_des_currents_mA[i]=0.0;
    }
}

bool init_comm(void){
    sock_cli = socket(AF_INET,SOCK_STREAM, 0);
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(TNB_MNS_PORT);
    servaddr.sin_addr.s_addr = inet_addr("192.168.1.10");

    //the option SO_REUSEADDR needs to be set in case there is an old TCP connection in TIME_WAIT state
    const int enable=1;
    setsockopt(sock_cli,SOL_SOCKET,SO_REUSEADDR,&enable,sizeof(enable));
    int one = 1;
    setsockopt(sock_cli, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 25000;
    setsockopt(sock_cli, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    //try to establish a tcp connection to the ccard
    //ROS_INFO("Establishing TCP connection");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Establishing TCP connection");
    if(connect_with_timeout(sock_cli, (struct sockaddr *)&servaddr, sizeof(servaddr),5000)<0){
        return false;
    }
    /*
    if (connect(sock_cli, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0)
        return false;
    ROS_INFO("TCP connection established");
    */

    return true;
}

void srv_run_regular_cb(const std::shared_ptr<tnb_mns_driver::srv::RunRegular::Request> req, 
    std::shared_ptr<tnb_mns_driver::srv::RunRegular::Response> response){
    //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
    for(int i=0; i<NO_CHANNELS; i++)
        m_run_reg_flags[i]=req->enable[i];
    response->success=true;
}

// bool srv_run_regular_cb(tnb_mns_driver::TNBMNSStateTransition::Request& req,tnb_mns_driver::TNBMNSStateTransition::Response& res){
//     //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
//     for(int i=0; i<NO_CHANNELS; i++)
//         m_run_reg_flags[i]=req.enable[i];
//     res.success=true;
//     return true;
// }

void srv_run_resonant_cb(const std::shared_ptr<tnb_mns_driver::srv::RunResonant::Request> req, 
    std::shared_ptr<tnb_mns_driver::srv::RunResonant::Response> res){
    //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
    for(int i=0; i<NO_CHANNELS; i++)
        m_run_res_flags[i]=req->enable[i];
    res->success=true;
}

// bool srv_run_resonant_cb(tnb_mns_driver::TNBMNSStateTransition::Request& req,tnb_mns_driver::TNBMNSStateTransition::Response& res){
//     //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
//     for(int i=0; i<NO_CHANNELS; i++)
//         m_run_res_flags[i]=req.enable[i];
//     res.success=true;
//     return true;
// }

void srv_enable_tnb_mns_driver_cb(const std::shared_ptr<tnb_mns_driver::srv::Enable::Request> req, 
    std::shared_ptr<tnb_mns_driver::srv::Enable::Response> resp){
    //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
    for(int i=0; i<NO_CHANNELS; i++)
        m_buck_flags[i]=req->enable[i];
    resp->success=true;
}

// bool srv_enable_tnb_mns_driver_cb(tnb_mns_driver::TNBMNSStateTransition::Request& req,tnb_mns_driver::TNBMNSStateTransition::Response& res){
//     //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
//     for(int i=0; i<NO_CHANNELS; i++)
//         m_buck_flags[i]=req.enable[i];
//     res.success=true;
//     return true;
// }

void srv_stop_tnb_mns_driver_cb(const std::shared_ptr<tnb_mns_driver::srv::Stop::Request> req,
    std::shared_ptr<tnb_mns_driver::srv::Stop::Response> res){
    //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
    for(int i=0; i<NO_CHANNELS; i++)
        m_stop_flags[i]=req->enable[i];
    reset_currents();
    res->success=true;
}

// bool srv_stop_tnb_mns_driver_cb(tnb_mns_driver::TNBMNSStateTransition::Request& req,tnb_mns_driver::TNBMNSStateTransition::Response& res){
//     //set the buck en flags here to true, they will be set to false once a single Ethernet package has been sent
//     for(int i=0; i<NO_CHANNELS; i++)
//         m_stop_flags[i]=req.enable[i];
//     res.success=true;
//     reset_currents();
//     return true;
// }

void msg_des_currents_reg_cb(std_msgs::msg::Float32MultiArray::UniquePtr msg){
    //ROS_DEBUG("Received New Regular Currents");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received New Regular Currents");
    for(int i=0; i<NO_CHANNELS; i++){
        //ROS_DEBUG("Received desired current: %f",msg->data[i]);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received desired current: %f",msg->data[i]);
        if(msg->data[i]>max_current){
            m_des_currents_mA[i]=max_current;
            //ROS_DEBUG("current limited to maximum current");
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"current limited to maximum current");
        }
        else if(msg->data[i]<-max_current){
            m_des_currents_mA[i]=-max_current;
            //ROS_DEBUG("current limited to maximum current");
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"current limited to maximum current");
        }
        else
            m_des_currents_mA[i]=msg->data[i];
    }
}

// void msg_des_currents_reg_cb(const std_msgs::Float32MultiArray& msg){
//     ROS_DEBUG("Received New Regular Currents");
//     for(int i=0; i<NO_CHANNELS; i++){
//         ROS_DEBUG("Received desired current: %f",msg.data[i]);
//         if(msg.data[i]>max_current){
//             m_des_currents_mA[i]=max_current;
//             ROS_DEBUG("current limited to maximum current");
//         }
//         else if(msg.data[i]<-max_current){
//             m_des_currents_mA[i]=-max_current;
//             ROS_DEBUG("current limited to maximum current");
//         }
//         else
//             m_des_currents_mA[i]=msg.data[i];
//     }
// }

void msg_des_duties_reg_cb(std_msgs::msg::Float32MultiArray::UniquePtr msg){
    //ROS_DEBUG("Received New Duties");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received New Duties");
    for(int i=0; i<NO_CHANNELS; i++){
        if(msg->data[i]>0.9)
            m_des_duties[i]=0.9;
        else if(msg->data[i]<0.0)
            m_des_duties[i]=0.0;
        else
            m_des_duties[i]=msg->data[i];
    }
}

// void msg_des_duties_reg_cb(const std_msgs::Float32MultiArray & msg){
//     ROS_DEBUG("Received New Duties");
//     for(int i=0; i<NO_CHANNELS; i++){
//         if(msg.data[i]>0.9)
//             m_des_duties[i]=0.9;
//         else if(msg.data[i]<0.0)
//             m_des_duties[i]=0.0;
//         else
//             m_des_duties[i]=msg.data[i];
//     }
// }

void msg_des_currents_res_cb(std_msgs::msg::Float32MultiArray::UniquePtr msg){
    //ROS_DEBUG("Received New Resonant Currents");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received New Resonant Currents");
    for(int i=0; i<NO_CHANNELS; i++){
        //ROS_DEBUG("Received: %f",msg.data[i]);
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received: %f",msg->data[i]);
        if(msg->data[i]>max_current_res)
            m_des_currents_res_mA[i]=max_current_res;
        else if(msg->data[i]<-max_current_res)
            m_des_currents_res_mA[i]=-max_current_res;
        else
            m_des_currents_res_mA[i]=msg->data[i];
    }
}

// void msg_des_currents_res_cb(const std_msgs::Float32MultiArray& msg){
//     ROS_DEBUG("Received New Resonant Currents");
//     for(int i=0; i<NO_CHANNELS; i++){
//         ROS_DEBUG("Received: %f",msg.data[i]);
//         if(msg.data[i]>max_current_res)
//             m_des_currents_res_mA[i]=max_current_res;
//         else if(msg.data[i]<-max_current_res)
//             m_des_currents_res_mA[i]=-max_current_res;
//         else
//             m_des_currents_res_mA[i]=msg.data[i];
//     }
// }

void msg_des_freqs_cb(std_msgs::msg::Float32MultiArray::UniquePtr msg){
    //ROS_DEBUG("Received New Res Frequencies");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received New Res Frequencies");
    for(int i=0; i<NO_CHANNELS; i++){
        m_des_freqs_mhz[i]=msg->data[i];
    }
}

// void msg_des_freqs_cb(const std_msgs::Float32MultiArray& msg){
//     ROS_DEBUG("Received New Res Frequencies");
//     for(int i=0; i<NO_CHANNELS; i++){
//         m_des_freqs_mhz[i]=msg.data[i];
//     }
// }

unsigned int send_counter=0;
void tnb_mns_driver_timer_cb(){
    //ROS_DEBUG("Entering Timer Handler...");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Entering Timer Handler...");
    //apply the first order time constant [pre-filter]
    for(int i=0; i<NO_CHANNELS; i++){
        m_des_currents_mA_filtered[i]=pre_filter_tau/(pre_filter_tau+send_interval)*m_des_currents_mA_prev[i]+send_interval/(pre_filter_tau+send_interval)*m_des_currents_mA[i];
    }

    //assemble the tnb_mns_message and send ethernet package
    for(int i=0; i<NO_CHANNELS; i++){
        if(enable_input_lowpass)
            m_next_message.desCurrents[i]=(int16_t)(m_des_currents_mA_filtered[i]*1000);
        else
            m_next_message.desCurrents[i]=(int16_t)(m_des_currents_mA[i]*1000);
        m_next_message.desCurrentsRes[i]=(uint16_t)(m_des_currents_res_mA[i]*1000);
        m_next_message.desDuties[i]=(uint16_t)(m_des_duties[i]*UINT16_MAX);
        m_next_message.desFreqs[i]=m_des_freqs_mhz[i];
    }
    //reset fsm control bytes
    m_next_message.stp_flg_byte=0;
    m_next_message.buck_flg_byte=0;
    m_next_message.regen_flg_byte=0;
    m_next_message.resen_flg_byte=0;
    for(int i=0; i<NO_CHANNELS; i++){
        if(m_stop_flags[i]){
            m_next_message.stp_flg_byte=(m_next_message.stp_flg_byte | (1<<i) );
            m_stop_flags[i]=false;
        }
        if(m_buck_flags[i]){
            m_next_message.buck_flg_byte=(m_next_message.buck_flg_byte | (1<<i) );
            m_buck_flags[i]=false;
        }
        if(m_run_reg_flags[i]){
            m_next_message.regen_flg_byte=(m_next_message.regen_flg_byte | (1<<i) );
            m_run_reg_flags[i]=false;
        }
        if(m_run_res_flags[i]){
            m_next_message.resen_flg_byte=(m_next_message.resen_flg_byte | (1<<i) );
            m_run_res_flags[i]=false;
        }
    }
    //send the ethernet package
    //ROS_DEBUG("Sending Ethernet Package");
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Sending Ethernet Package");
    for(int i=0; i<NO_CHANNELS; i++)
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Sending des_currents[%d]=%f",i,m_des_currents_mA_filtered[i]);
        //ROS_DEBUG("Sending des_currents[%d]=%f",i,m_des_currents_mA_filtered[i]);
    if(hardware_connected){
        memcpy(send_buffer,&m_next_message,sizeof(m_next_message));
        if(send(sock_cli,send_buffer,sizeof(m_next_message),0)<0){
            //ROS_ERROR("Could not send message via TCP, aborting");
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Could not send message via TCP, aborting");
            reset_currents();
            close(sock_cli);
            rclcpp::shutdown();
            return;
        }
    }
    //check if TCP packages have been received
    //ROS_DEBUG("Reading RECV Buffer");
    // RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Reading RECV Buffer");
    // uint8_t bytes_received[2048];
    // int bytes_read=recv(sock_cli,bytes_received,2048,0);
    // if(bytes_read>0){
    //     //ROS_DEBUG("Received %d bytes...",bytes_read);
    //     RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received %d bytes...",bytes_read);
    //     if(bytes_read!=sizeof(tnb_mns_msg_sysstate)){
    //         reset_currents();
    //         RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"),"Received %d bytes : However, message size from UCU should be %d",bytes_read,sizeof(tnb_mns_msg_sysstate));
    //         //ROS_DEBUG("Received %d bytes : However, message size from UCU should be %d",bytes_read,sizeof(tnb_mns_msg_sysstate));
    //     }
    //     else{
    //         memcpy(&m_last_system_report,bytes_received,sizeof(tnb_mns_msg_sysstate));
    //         if(send_counter%TNB_MNS_STATUS_DOWNSAMPLE==0){
    //             auto tnbmns_status=tnb_mns_driver::msg::TnbMnsStatus();
    //             for(int i=0; i<NO_CHANNELS; i++){
    //                 tnbmns_status.currents_reg[i]=m_last_system_report.currents[i]*1e-3;
    //                 tnbmns_status.states[i]=m_last_system_report.states[i];
    //                 tnbmns_status.duties[i]=(double)(m_last_system_report.duties[i])/((double)(65535.0));
    //                 tnbmns_status.res_freqs[i]=m_last_system_report.freqs[i]*1e-3;
    //                 tnbmns_status.dclink_voltages[i]=m_last_system_report.dclink_voltages[i]*1e-3;
    //             }
    //             tnb_mns_state_publisher->publish(tnbmns_status);
    //         }  
    //     } 
    // }
    // if(bytes_read==-1){
    //     RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Error reading from socket: %s",strerror(errno));
    //     //ROS_ERROR("Error reading from socket: %s",strerror(errno));
    // }

   //store the sent packet for later
   for(int i=0; i<NO_CHANNELS; i++)
        m_des_currents_mA_prev[i]=m_des_currents_mA_filtered[i];

    //increment send counter
    send_counter++;
    if(send_counter%packets_per_second==0)
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sent another %d packets...",packets_per_second);
        //ROS_INFO("Sent another %d packets...",packets_per_second);
    return;
}

//main
int main(int argc,char** argv){
    //ros::init(argc, argv, "tnb_mns_driver_node",ros::init_options::NoSigintHandler);
    rclcpp::init(argc,argv);

    //ROS_INFO("startup");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"startup");
    

    signal(SIGINT, mySigintHandler);

    //ros::NodeHandle nh("~");
    auto nh=std::make_shared<rclcpp::Node>("tnb_mns_driver_node");

    //init ttycomm
    if(hardware_connected){
        bool comm_est_succeeded=init_comm();
        if(!comm_est_succeeded){
            if(errno==ETIMEDOUT)
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Could not establish TCP connection to ControlCard: Timeout");
                //ROS_ERROR("Could not establish TCP connection to ControlCard: Timeout");
            else
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Could not establish TCP connection to ControlCard: %s",strerror(errno));
                //ROS_ERROR("Could not establish TCP connection to ControlCard: %s",strerror(errno));
            rclcpp::shutdown();
            return EXIT_FAILURE;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Established Connection");
        //ROS_INFO("Established Connection");
    }

    // TODO : create a magnetic field model

    //main timer shows callback function send the driver messages
    rclcpp::TimerBase::SharedPtr timer = nh->create_wall_timer(std::chrono::milliseconds(long(1e3*send_interval)), tnb_mns_driver_timer_cb);
    //ros::Timer timer = nhcreateTimer(ros::Duration(send_interval), tnb_mns_driver_timer_cb);
    //ROS_INFO("Sending data packets at an interval of %f ms",send_interval*1e3);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sending data packets at an interval of %f ms",send_interval*1e3);

    //initialize some fields in tnb_mns_structure
    for(int i=0; i<NO_CHANNELS; i++){
        m_next_message.desFreqs[i]=DEFAULT_RES_FREQ_MILLIHZ;
        m_des_freqs_mhz[i]=DEFAULT_RES_FREQ_MILLIHZ;
    }

    //publisher node for TNB_MNS_STATUS
    tnb_mns_state_publisher=nh->create_publisher<tnb_mns_driver::msg::TnbMnsStatus>("/tnb_mns_driver/system_state", 1);
    //tnb_mns_state_publisher = nh.advertise<tnb_mns_driver::TnbMnsStatus>("/tnb_mns_driver/system_state", 1);

    //register services
    srv_enable_tnb_mns_driver=nh->create_service<tnb_mns_driver::srv::Enable>("/tnb_mns_driver/enable",&srv_enable_tnb_mns_driver_cb);
    srv_stop_tnb_mns_driver=nh->create_service<tnb_mns_driver::srv::Stop>("/tnb_mns_driver/stop",&srv_stop_tnb_mns_driver_cb);
    srv_run_regular=nh->create_service<tnb_mns_driver::srv::RunRegular>("/tnb_mns_driver/run_regular",&srv_run_regular_cb);
    srv_run_resonant=nh->create_service<tnb_mns_driver::srv::RunResonant>("/tnb_mns_driver/run_resonant",&srv_run_resonant_cb);
    // //srv_enable_tnb_mns_driver=nh.advertiseService("/tnb_mns_driver/enable",srv_enable_tnb_mns_driver_cb);
    // //srv_stop_tnb_mns_driver=nh.advertiseService("/tnb_mns_driver/stop",srv_stop_tnb_mns_driver_cb);
    // //srv_run_regular=nh.advertiseService("/tnb_mns_driver/run_regular",srv_run_regular_cb);
    // //srv_run_resonant=nh.advertiseService("/tnb_mns_driver/run_resonant",srv_run_resonant_cb);

    // //register subscribers
    des_currents_reg_subs=nh->create_subscription<std_msgs::msg::Float32MultiArray>("/tnb_mns_driver/des_currents_reg",64,msg_des_currents_reg_cb);
    des_duties_subs=nh->create_subscription<std_msgs::msg::Float32MultiArray>("/tnb_mns_driver/des_duties",64,msg_des_duties_reg_cb);
    des_currents_res_subs=nh->create_subscription<std_msgs::msg::Float32MultiArray>("/tnb_mns_driver/des_currents_res",64,msg_des_currents_res_cb);
    des_freqs_res=nh->create_subscription<std_msgs::msg::Float32MultiArray>("/tnb_mns_driver/des_freqs_res",64,msg_des_freqs_cb);
    // des_currents_reg_subs=nh.subscribe("/tnb_mns_driver/des_currents_reg",64,msg_des_currents_reg_cb);
    // des_duties_subs=nh.subscribe("/tnb_mns_driver/des_duties",64,msg_des_duties_reg_cb);
    // des_currents_res_subs=nh.subscribe("/tnb_mns_driver/des_currents_res",64,msg_des_currents_res_cb);
    // des_freqs_res=nh.subscribe("/tnb_mns_driver/des_freqs_res",64,msg_des_freqs_cb);

    //ROS_INFO("Start spinning");
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Start spinning");
    

    rclcpp::spin(nh);
}