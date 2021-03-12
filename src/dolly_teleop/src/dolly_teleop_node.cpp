// *****************************************************************************
// @toffanetto
// Dolly Teleop Key Node
// ROS2 Foxy
// Author: Gabriel Toffanetto França da Rocha - gabriel.toffanetto@unifei.edu.br
// Github: https://github.com/toffanetto
// Laboratory of Robotics, Intelligent and Complex Systems - RobSIC
// Universidade Federal de Itajubá
// Created: 2021//03/11 - 20::00 (UTF -3)
// *****************************************************************************

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <assert.h>
#include <chrono>

using namespace std::chrono_literals;

#define VEL_ANG_MAX 0
#define VEL_LIN_MAX 0


class Teleop : public rclcpp::Node{
    public: 

        Teleop():Node("teleop"){
            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            //cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", default_qos); // topic publisher

            timer_key_read = this->create_wall_timer(10ms, std::bind(&Teleop::readPubKey,this));
        }


    private:

        int getch() {
            int c=0;
            struct termios org_opts, new_opts;
            int res=0;
                //-----  store old settings -----------
            res=tcgetattr(STDIN_FILENO, &org_opts);
            assert(res==0);
                //---- set new terminal parms --------
            memcpy(&new_opts, &org_opts, sizeof(new_opts));
            new_opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOKE | ICRNL);
            tcsetattr(STDIN_FILENO, TCSANOW, &new_opts);
            c=getchar();
                //------  restore old settings ---------
            res=tcsetattr(STDIN_FILENO, TCSANOW, &org_opts);
            assert(res==0);
            return(c);
        }

        void readPubKey(){

            auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();

            int c;   

            c = getch();

            switch (c){

            case 65:
                // UP_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: UP_KEY");
                cmd_msg->linear.x = VEL_LIN_MAX;
                cmd_msg->angular.z = 0;
                break;
            
            case 66:
                // DOWN_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: DOWN_KEY");
                cmd_msg->linear.x = - VEL_LIN_MAX;
                cmd_msg->angular.z = 0;
                break;
            
            case 67:
                // RIGHT_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: RIGHT_KEY");
                cmd_msg->linear.x = 0;
                cmd_msg->angular.z = VEL_ANG_MAX;
                break;
            
            case 68:
                // LEFT_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: LEFT_KEY");
                cmd_msg->linear.x = 0;
                cmd_msg->angular.z = - VEL_ANG_MAX;
                break;

            case 119:
                // w (UP_KEY) 
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: W");
                cmd_msg->linear.x = VEL_LIN_MAX;
                cmd_msg->angular.z = 0;
                break;
            
            case 115:
                // s (DOWN_KEY)
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: S");
                cmd_msg->linear.x = - VEL_LIN_MAX;
                cmd_msg->angular.z = 0;
                break;
            
            case 100:
                // d (RIGHT_KEY)
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: D");
                cmd_msg->linear.x = 0;
                cmd_msg->angular.z = VEL_ANG_MAX;
                break;
            
            case 97:
                // a (LEFT_KEY)
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: A");
                cmd_msg->linear.x = 0;
                cmd_msg->angular.z = - VEL_ANG_MAX;
                break;
            case -1:
                exit(0);
                break;

            default:
                break;
            }
            //cmd_pub->publish(std::move(cmd_msg));
            RCLCPP_INFO(this->get_logger(), "KEY PRESSED: '%d'",c);
        
        }
    

        rclcpp::TimerBase::SharedPtr timer_key_read;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;


};


int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<Teleop>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    

    return 0;
}