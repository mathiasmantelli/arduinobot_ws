#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <arduinobot_msgs/srv/euler_to_quaternion.hpp>
#include <arduinobot_msgs/srv/quaternion_to_euler.hpp>
#include <tf2/utils.h>

using namespace std::placeholders;

class AnglesConverter : public rclcpp::Node{
public:
    AnglesConverter() : Node("angles_conversion_service"){
        euler_to_quaternion_ = create_service<arduinobot_msgs::srv::EulerToQuaternion>("euler_to_quaternion", std::bind(&AnglesConverter::eulerToQuaternionCallback, this, _1, _2));
        quaternion_to_euler_ = create_service<arduinobot_msgs::srv::QuaternionToEuler>("quaternion_to_euler", std::bind(&AnglesConverter::quaternionToEulerCallback, this, _1, _2));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Angle Conversion services are ready!");
    }

private:
    rclcpp::Service<arduinobot_msgs::srv::EulerToQuaternion>::SharedPtr euler_to_quaternion_;
    rclcpp::Service<arduinobot_msgs::srv::QuaternionToEuler>::SharedPtr quaternion_to_euler_;

    void eulerToQuaternionCallback(const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Request> request, 
                         const std::shared_ptr<arduinobot_msgs::srv::EulerToQuaternion::Response> response){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New request to convert euler angles. roll: " << request->roll << "  | pitch: " << request->pitch << "  | yaw: " << request->yaw << " into a quaternion");
        tf2::Quaternion q;
        q.setRPY(request->roll, request->pitch, request->yaw);
        response->x = q.getX();
        response->y = q.getY();
        response->z = q.getZ();
        response->w = q.getW(); 
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Returning: x: " << response->x << " y: " << response->y << " z: " << response->z << " w: " << response->w);
        
    }


    void quaternionToEulerCallback(const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Request> request, 
                         const std::shared_ptr<arduinobot_msgs::srv::QuaternionToEuler::Response> response){
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "New request received. x: " << request->x << " y: " << request->y << " z: " << request->z << " w: " << request->w);
        tf2::Quaternion q(request->x, request->y, request->z, request->w);
        tf2::Matrix3x3 m(q);
        m.getRPY(response->roll, response->pitch, response->yaw);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Returning roll: " << response->roll << " pitch: " << response->pitch << " yaw: " << response->yaw);
        
    }
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AnglesConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}