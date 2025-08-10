#include <signal.h>
#include <array>
#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include "mdriver/srv/state_transition.hpp"

#define NO_CHANNELS 9

using namespace std::chrono_literals;

bool rectangular_current = true;
void sig_int_handler(int sig)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shutdown using SIGINT");

  // All the default sigint handler does is call shutdown()
  rclcpp::shutdown();
}

class MDriverTestNode : public rclcpp::Node
{
public:
  MDriverTestNode()
    : Node("mdriver_node_test"), delta_T_(10ms), i_amp_(1.0), freq_(1.0 * M_PI), currents_(NO_CHANNELS, 0.0), msg_()
  {
    RCLCPP_INFO(this->get_logger(), "Starting up mdriver test node");

    msg_.data.resize(NO_CHANNELS);

    this->declare_parameter("rectangular_current", false);
    this->declare_parameter("amplitude", 1.0);
    this->declare_parameter("frequency", 1.0);

    // put the driver into run_regular mode

    // put the driver into `ENABLE` mode
    RCLCPP_INFO(this->get_logger(), "Enabling driver");
    client_enable = this->create_client<mdriver::srv::StateTransition>("/mdriver/enable");
    while (!client_enable->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for enable service...");
    }
    std::array<bool, NO_CHANNELS> enablearr;
    for(int i=0; i<NO_CHANNELS; i++)
      enablearr[i]=true;
    auto request = std::make_shared<mdriver::srv::StateTransition::Request>();
    request->enable = enablearr;
    // async_send_request is non-blocking
    auto result = client_enable->async_send_request(request);
    if (!(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
          rclcpp::FutureReturnCode::SUCCESS))
    {
      RCLCPP_ERROR(this->get_logger(), "Error enabling driver");
      rclcpp::shutdown();
    }
    rclcpp::sleep_for(1s);

    // put the driver into `RUN_REGULAR_MODE`
    RCLCPP_INFO(this->get_logger(), "Putting driver into RUN_REGULAR mode");
    client_runregular = this->create_client<mdriver::srv::StateTransition>("mdriver/run_regular");
    while (!client_runregular->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for RUN_REGULAR service...");
    }
    std::array<bool, NO_CHANNELS> runregarray;
    for(int i=0; i<NO_CHANNELS; i++)
    runregarray[i]=true;
    auto runreg_request = std::make_shared<mdriver::srv::StateTransition::Request>();
    runreg_request->enable = runregarray;
    auto runreg_result = client_runregular->async_send_request(runreg_request);
    if (!(rclcpp::spin_until_future_complete(this->get_node_base_interface(), runreg_result) ==
          rclcpp::FutureReturnCode::SUCCESS))
    {
      RCLCPP_ERROR(this->get_logger(), "Error putting driver into regular mode");
      rclcpp::shutdown();
    }
    rclcpp::sleep_for(1s);

    // register a stop client
    client_stop = this->create_client<mdriver::srv::StateTransition>("/mdriver/stop");
    while (!client_stop->wait_for_service(1s))
    {
      RCLCPP_INFO(this->get_logger(), "Waiting for STOP service");
    }

    currents_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/mdriver/des_currents_reg", 256);

    timer_ = this->create_wall_timer(delta_T_, std::bind(&MDriverTestNode::mdriver_timer_cb, this));

    RCLCPP_INFO(this->get_logger(), "Start spinning");
  }

  ~MDriverTestNode()
  {
    // call the stop node before terminating
    auto req = std::make_shared<mdriver::srv::StateTransition::Request>();
    std::array<bool, NO_CHANNELS> stoparr;
    for(int i=0; i<NO_CHANNELS; i++)
      stoparr[i]=true;
    req->enable = stoparr;
    auto res = client_stop->async_send_request(req);
    if (!(rclcpp::spin_until_future_complete(this->get_node_base_interface(), res) ==
          rclcpp::FutureReturnCode::SUCCESS))
    {
      RCLCPP_ERROR(this->get_logger(), "Error calling STOP service");
    }
    rclcpp::sleep_for(1s);
    rclcpp::shutdown();
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr currents_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mdriver::srv::StateTransition>::SharedPtr client_enable;
  rclcpp::Client<mdriver::srv::StateTransition>::SharedPtr client_runregular;
  rclcpp::Client<mdriver::srv::StateTransition>::SharedPtr client_stop;
  // duration is defined in multiples of 1/1000 seconds (i.e. milliseconds)
  const std::chrono::duration<long long, std::ratio<1, 1000>> delta_T_;
  double i_amp_;
  double freq_;
  std::vector<double> currents_;
  std_msgs::msg::Float32MultiArray msg_;

  void mdriver_timer_cb()
  {
    rclcpp::Time current_time = this->get_clock()->now();
    double current;
    i_amp_ = this->get_parameter("amplitude").as_double();
    freq_ = this->get_parameter("frequency").as_double();
    if (rectangular_current)
    {
      double relt = current_time.seconds() - int(current_time.seconds());
      if (relt > 0.5)
        current = i_amp_;
      else
        current = -i_amp_;
    }
    else
      current = i_amp_ * sin(2 * M_PI * freq_ * current_time.seconds());

    for (size_t i = 0; i < msg_.data.size(); ++i)
    {
      msg_.data[i] = 0.0f;
    }

    msg_.data[0] = static_cast<float>(current);

    currents_pub_->publish(msg_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  signal(SIGINT, sig_int_handler);

  rclcpp::spin(std::make_shared<MDriverTestNode>());
  rclcpp::shutdown();
  return 0;
}