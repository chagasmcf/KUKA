#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/bool.h>

#include <cmath>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>


using namespace std;

#define pi 3.14159265358979323846

using std::placeholders::_1;

class PublishingSubscriber : public rclcpp::Node
{
public:
  PublishingSubscriber()
  : Node("publishing_subscriber")
  {
       // Publisher   
      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_command_controller_position/commands", 1);
      timer_ = this->create_wall_timer(
      1000ms, std::bind(&PublishingSubscriber::timer_callback, this));
      
      // Subscriber
      joint_states_subscription_= this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1, std::bind(&PublishingSubscriber::topic_callback_joint, this, _1));
   
  }

private:

string mensagem;

   private:
    void timer_callback()
    {
   
    
      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data.push_back(1.0);
      msg.data.push_back(-1.0);
      msg.data.push_back(1.0);
      msg.data.push_back(0.0);
      msg.data.push_back(0.0);
      msg.data.push_back(0.0);
    
      publisher_->publish(msg);
  
      
    }
    
      //declaração de timer, publisher e count
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
      size_t count_;


   void topic_callback_joint(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {
  	
  	
   // RCLCPP_INFO(this->get_logger(), "I heard JointState: '%s'", to_string(msg->position[0]));
   
   
   
   cout << "Name: " << msg->name[1] << endl;
   cout << "Position: " << msg->position[1] << endl;
   cout << "Velocity: " << msg->velocity[1] << endl;
   cout << "Effort: " << msg->effort[1] << endl;
   cout << "---------------------------------------------------" << endl;
   cout << "Name: " << msg->name[2] << endl;
   cout << "Position: " << msg->position[2] << endl;
   cout << "Velocity: " << msg->velocity[2] << endl;
   cout << "Effort: " << msg->effort[2] << endl;
   cout << "---------------------------------------------------" << endl;
   cout << "Name: " << msg->name[3] << endl;
   cout << "Position: " << msg->position[3] << endl;
   cout << "Velocity: " << msg->velocity[3] << endl;
   cout << "Effort: " << msg->effort[3] << endl;
   cout << "---------------------------------------------------" << endl;
   cout << "Name: " << msg->name[5] << endl;
   cout << "Position: " << msg->position[5] << endl;
   cout << "Velocity: " << msg->velocity[5] << endl;
   cout << "Effort: " << msg->effort[5] << endl;
   cout << "---------------------------------------------------" << endl;
   cout << "Name: " << msg->name[4] << endl;
   cout << "Position: " << msg->position[4] << endl;
   cout << "Velocity: " << msg->velocity[4] << endl;
   cout << "Effort: " << msg->effort[4] << endl;
   cout << "---------------------------------------------------" << endl;
   cout << "Name: " << msg->name[0] << endl;
   cout << "Position: " << msg->position[0] << endl;
   cout << "Velocity: " << msg->velocity[0] << endl;
   cout << "Effort: " << msg->effort[0] << endl;
   cout << "---------------------------------------------------" << endl;   

    
   
  }
  
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  
  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  //inicializa ros2
  
  rclcpp::spin(std::make_shared<PublishingSubscriber>()); //começa a processar dados do nó PublishingSubiscriber
  rclcpp::shutdown();
  return 0;
}
