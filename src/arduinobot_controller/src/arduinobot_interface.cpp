#include "arduinobot_controller/arduinobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace arduinobot_controller{
ArduinobotInterface::ArduinobotInterface(){

}

ArduinobotInterface::~ArduinobotInterface(){
    if(arduino_.IsOpen()){
        try{
            arduino_.Close();
        }catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Something went wrong while closing the connection with port " << port_);
        }
    }
}

CallbackRetrun ArduinobotInterface::on_init(const hardware_interface::HardwareInfo & hardware_info){
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);

    if(result != CallbackRetrun::SUCCESS){
        return result;
    }

    try{
        port_ = info_.hardware_parameters.at("port");
    }catch(const std::out_of_range &e){
        RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "No serial port provided. Aborting");
        return CallbackRetrun::FAILURE;
    }

    position_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    prev_position_commands_.reserve(info_.joints.size());
    
    return CallbackRetrun::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinobotInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for(size_t i = 0; i < info_.joints.size(); i++){
        state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
    }    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinobotInterface::export_command_interfaces(){
    std::vector<hardware_interface::CommandInterface> command_interfaces; 
    for(size_t i = 0; i < info_.joints.size(); i++){
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }    
    return command_interfaces;  
}

CallbackRetrun ArduinobotInterface::on_activate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Starting the robot interface...");
    position_commands_ = {0.0, 0.0, 0.0, 0.0};
    prev_position_commands_ = {0.0, 0.0, 0.0, 0.0};
    position_states_ = {0.0, 0.0, 0.0, 0.0};

    try{
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    }catch(...){
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Something went wrong while interacting with the port " << port_);
        return CallbackRetrun::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware started, ready to take commands");
    return CallbackRetrun::SUCCESS;
}

CallbackRetrun ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state){
    RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Stopping the robot hardware");

    if(arduino_.IsOpen()){
        try{
            arduino_.Close();
        }catch(...){
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Something went wrong while close the connection with port " << port_);
            return CallbackRetrun::FAILURE;                
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware stopped");
    return CallbackRetrun::SUCCESS;
    
}

hardware_interface::return_type ArduinobotInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period){
    position_states_ = position_commands_;
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinobotInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period){
    if(position_commands_ == prev_position_commands_){
        return hardware_interface::return_type::OK;
    }
    
    std::string msg; 
    int base = static_cast<int>(((position_commands_.at(0) + (M_PI/2.0)) * 180) / M_PI);
    msg.append("b");
    msg.append(std::to_string(base));
    msg.append(",");

    int shoulder = 180 - static_cast<int>(((position_commands_.at(1) + (M_PI/2.0)) * 180) / M_PI);
    msg.append("s");
    msg.append(std::to_string(shoulder));
    msg.append(",");

    int elbow = static_cast<int>(((position_commands_.at(2) + (M_PI/2.0)) * 180) / M_PI);
    msg.append("e");
    msg.append(std::to_string(elbow));
    msg.append(",");

    int gripper = static_cast<int>((-position_commands_.at(3) * 180) / (M_PI / 2.0));
    msg.append("g");
    msg.append(std::to_string(gripper));
    msg.append(",");

    try{
        arduino_.Write(msg);
    }catch(...){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"), "Something went wrong while sending the msg: " << msg);
        return hardware_interface::return_type::ERROR;
    }

    prev_position_commands_ = position_commands_;
    return hardware_interface::return_type::OK;
}


}

PLUGINLIB_EXPORT_CLASS(arduinobot_controller::ArduinobotInterface, hardware_interface::SystemInterface);