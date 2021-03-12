#include "rclcpp/rclcpp.hpp"

class NO : public rclcpp::Node{
public:
    NO() : Node("Robsic"){
        timer_=this->create_wall_timer(std::chrono::milliseconds(200),std::bind(&NO::timercallback,this));
    }
private:
    void timercallback()
    {
    		RCLCPP_INFO(this->get_logger(),"Robsic On ROS");
    }
     rclcpp::TimerBase::SharedPtr timer_;
    };
int main(int argc, char **argv){
 	rclcpp::init(argc,argv);
	auto No=std::make_shared<NO>();
	rclcpp::spin(No);
	rclcpp::shutdown();
	return 0;
}