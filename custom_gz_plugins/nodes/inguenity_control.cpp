#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */


int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char * argv[])
{
const char* msg = R"(
Reading from the keyboard!
---------------------------
Thrust Key:
    w    
        
    s    

Lower Blades:

a     d


Upper Blades:
    u

    j

t : up (+z)
b : down (-z)
q/e : reverse thrust
w/s : increase/decrease Thrust Key by 10
a/d : increase/decrease Lower Blade angle by 1
u/j : increase/decrease Upper Blade angle by 1
NOTE : Increasing or Decreasing will take affect live on the moving robot.
    Consider Stopping the robot before changing it.
CTRL-C to quit
)";
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("uuv_teleop");
    std::cout<<msg<<std::endl;
    auto publisher_ver = node->create_publisher<std_msgs::msg::Float64>("/lower_blade", 10);
    auto publisher_hoz = node->create_publisher<std_msgs::msg::Float64>("/upper_blade", 10);
    auto publisher_thrust = node->create_publisher<std_msgs::msg::Float64>("/thrust", 10);
    auto message_ver = std_msgs::msg::Float64();
    auto message_hoz = std_msgs::msg::Float64();
    auto message_thrust = std_msgs::msg::Float64();
    message_ver.data = 0;
    message_hoz.data = 0;
    message_thrust.data = 0;

    while (rclcpp::ok())
    {
        char key = getch();

        if(key == 'A'|| key == 'a'){
            if (message_ver.data <= -1.0){
                message_ver.data = message_ver.data;
                std::cout<<"Lower Blade Angle:- "<<message_ver.data<<std::endl;
            }
            else{       
                message_ver.data = message_ver.data - 1;
                publisher_ver->publish(message_ver);
                std::cout<<"Lower Blade Angle:- "<<message_ver.data<<std::endl;
            }
        }
        if(key == 'D'|| key == 'd'){
            if (message_ver.data >= 1.0){
                message_ver.data = message_ver.data;
                std::cout<<"Lower Blade Angle:- "<<message_ver.data<<std::endl;
            }
            else{       
                message_ver.data = message_ver.data + 1;
                publisher_ver->publish(message_ver);
                std::cout<<"Lower Blade Angle:- "<<message_ver.data<<std::endl;
            }
        }
        if(key == 'u'|| key == 'U'){
            if (message_hoz.data <= -1.0){
                message_hoz.data = message_hoz.data;
                std::cout<<"Upper Blade Angle:- "<<message_hoz.data<<std::endl;
            }
            else{       
                message_hoz.data = message_hoz.data - 1;
                publisher_hoz->publish(message_hoz);
                std::cout<<"Upper Blade Angle:- "<<message_hoz.data<<std::endl;
            }
        }
        if(key == 'j'|| key == 'J'){
            if (message_hoz.data >= 1.0){
                message_hoz.data = message_hoz.data;
                std::cout<<"Upper Blade Angle:- "<<message_hoz.data<<std::endl;
            }
            else{       
                message_hoz.data = message_hoz.data + 1;
                publisher_hoz->publish(message_hoz);
                std::cout<<"Upper Blade Angle:- "<<message_hoz.data<<std::endl;
            }
        }
        if (key == 'q' || key == 'Q'){
            if (message_thrust.data <=0){
            message_thrust.data =  message_thrust.data;
            publisher_thrust->publish(message_thrust);
            std::cout<<"Thrust Value:- "<<message_thrust.data<<std::endl;
            }
            else {
            message_thrust.data = - message_thrust.data;
            publisher_thrust->publish(message_thrust);
            std::cout<<"Thrust Value:- "<<message_thrust.data<<std::endl;
            }

        }
        if (key == 'e' || key == 'E'){
            if (message_thrust.data >=0){
            message_thrust.data =  message_thrust.data;
            publisher_thrust->publish(message_thrust);
            std::cout<<"Thrust Value:- "<<message_thrust.data<<std::endl;
            }
            else {
            message_thrust.data = - message_thrust.data;
            publisher_thrust->publish(message_thrust);
            std::cout<<"Thrust Value:- "<<message_thrust.data<<std::endl;
            }

        }
        if(key == 'w' || key == 'W'){
        message_thrust.data =  message_thrust.data + 10;
        publisher_thrust->publish(message_thrust);
        std::cout<<"Thrust Value (UP):- "<<message_thrust.data<<std::endl;
        }

        if(key == 's' || key == 'S'){
        message_thrust.data =  message_thrust.data - 10;
        publisher_thrust->publish(message_thrust);
        std::cout<<"Thrust Value (DOWN):- "<<message_thrust.data<<std::endl;
        }

        else if(key == '\x03'){
            break;
        }
    }
    

    rclcpp::shutdown();
    return 0;
}