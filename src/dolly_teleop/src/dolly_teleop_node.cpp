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

#define VEL_ANG_MAX 1
#define VEL_LIN_MAX 1
#define VEL_ANG_INC 0.02
#define VEL_LIN_INC 0.08

double vel_ang = 0;
double vel_lin = 0;


class Teleop : public rclcpp::Node{
    public: 

        Teleop():Node("teleop"){
            
            puts("Dolly Teleop Key | Press arrows ou WASD for move and SPACE to stop");
            puts("Play with Dolly!");

            auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

            cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/dolly/cmd_vel", default_qos); // topic publisher

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

            c = getch();

            switch (c){

            case 65:
                // UP_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: UP_KEY");
                vel_lin = (vel_lin>=VEL_LIN_MAX) ? vel_lin : VEL_LIN_INC+vel_lin;
                break;
            
            case 66:
                // DOWN_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: DOWN_KEY");
                vel_lin = (vel_lin<=(-VEL_LIN_MAX)) ? vel_lin : vel_lin-VEL_LIN_INC;
                break;
            
            case 67:
                // RIGHT_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: RIGHT_KEY");
                vel_ang = (vel_ang<=(-VEL_ANG_MAX)) ? vel_ang : vel_ang-VEL_ANG_INC;
                break;
            
            case 68:
                // LEFT_KEY
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: LEFT_KEY");
                vel_ang = (vel_ang>=VEL_ANG_MAX) ? vel_ang : VEL_ANG_INC+vel_ang;
                break;

            case 119:
                // w (UP_KEY) 
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: W");
                vel_lin = (vel_lin>=VEL_LIN_MAX) ? vel_lin : VEL_LIN_INC+vel_lin;
                break;
            
            case 115:
                // s (DOWN_KEY)
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: S");
                vel_lin = (vel_lin<=(-VEL_LIN_MAX)) ? vel_lin : vel_lin-VEL_LIN_INC;
                break;
            
            case 100:
                // d (RIGHT_KEY)
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: D");
                vel_ang = (vel_ang<=(-VEL_ANG_MAX)) ? vel_ang : vel_ang-VEL_ANG_INC;
                break;
            
            case 97:
                // a (LEFT_KEY)
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: A");
                vel_ang = (vel_ang>=VEL_ANG_MAX) ? vel_ang : VEL_ANG_INC+vel_ang;
                break;
            case -1:
                exit(0);
                break;
            case 32:
                vel_lin = 0;
                vel_ang = 0;
                RCLCPP_INFO(this->get_logger(), "KEY PRESSED: STOP_KEY");
                break;
            default:
                break;
        }

            RCLCPP_INFO(this->get_logger(), "KEY PRESSED: '%f %f'",vel_lin, vel_ang);
            
            cmd_msg->linear.x = vel_lin;
            cmd_msg->angular.z = vel_ang;
            cmd_pub->publish(std::move(cmd_msg));
            
            RCLCPP_INFO(this->get_logger(), "KEY PRESSED: '%d'",c);
        
        }
    

        rclcpp::TimerBase::SharedPtr timer_key_read;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
        
        int c=0;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);

    auto node = std::make_shared<Teleop>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    

    return 0;
}