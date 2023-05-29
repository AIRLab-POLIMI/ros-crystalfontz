#ifndef BUILD_DRIVERNODE_HPP
#define BUILD_DRIVERNODE_HPP

#include "comms.h"

#include "rclcpp/rclcpp.hpp"

#include <chrono>

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#define CONTRAST_TOPIC			"/crystalfontz/contrast"
#define BACKLIGHT_POWER_TOPIC	"/crystalfontz/backlight_power"
#define KEY_ACTIVITY_TOPIC		"/crystalfontz/key_activity"

#define SERIAL_PORT "/dev/crystalfontz"
#define BAUD 115200

using std::placeholders::_1;
using namespace std::chrono_literals;

class DriverNode : public rclcpp::Node {

public:
    DriverNode() : Node("crystalfontz_driver") {

        subContrast = this->create_subscription<std_msgs::msg::Int32>(CONTRAST_TOPIC, 10, std::bind(&DriverNode::contrastCallback, this, _1));
        subBacklightPower = this->create_subscription<std_msgs::msg::Int32>(BACKLIGHT_POWER_TOPIC, 10, std::bind(&DriverNode::backlightPowerCallback, this, _1));

        subLine[0] = this->create_subscription<std_msgs::msg::String>(line_topic+"1", 10, std::bind(&DriverNode::line1Callback, this, _1));
        subLine[1] = this->create_subscription<std_msgs::msg::String>(line_topic+"2", 10, std::bind(&DriverNode::line2Callback, this, _1));
        subLine[2] = this->create_subscription<std_msgs::msg::String>(line_topic+"3", 10, std::bind(&DriverNode::line3Callback, this, _1));
        subLine[3] = this->create_subscription<std_msgs::msg::String>(line_topic+"4", 10, std::bind(&DriverNode::line4Callback, this, _1));

        subLed[0] = this->create_subscription<std_msgs::msg::ColorRGBA>(led_topic+"1", 10, std::bind(&DriverNode::led1Callback, this, _1));
        subLed[1] = this->create_subscription<std_msgs::msg::ColorRGBA>(led_topic+"2", 10, std::bind(&DriverNode::led2Callback, this, _1));
        subLed[2] = this->create_subscription<std_msgs::msg::ColorRGBA>(led_topic+"3", 10, std::bind(&DriverNode::led3Callback, this, _1));
        subLed[3] = this->create_subscription<std_msgs::msg::ColorRGBA>(led_topic+"4", 10, std::bind(&DriverNode::led4Callback, this, _1));

        pubKeyActivity = this->create_publisher<std_msgs::msg::Int8>(KEY_ACTIVITY_TOPIC, 10);

        timer = this->create_wall_timer(10ms, std::bind(&DriverNode::mainLoop, this));

        this->declare_parameter("/crystalfontz_driver/contrast", 100);
        contrast = this->get_parameter("/crystalfontz_driver/contrast").as_int();
        contrast = std::clamp(contrast, 0, 254);

        this->declare_parameter("/crystalfontz_driver/backlight_power", 50);
        backlightPower = this->get_parameter("/crystalfontz_driver/backlight_power").as_int();
        backlightPower = std::clamp(backlightPower, 0, 254);

        for (int i = 0; i < 4; ++i) {
            this->declare_parameter("/crystalfontz_driver/init_line_"+std::to_string(i+1), "");
            std::string s = this->get_parameter("/crystalfontz_driver/init_line_"+std::to_string(i+1)).as_string();

            for(std::string::size_type j = 0; j < s.size() && j < sizeof(line[0]); j++)
                if(s[j] != '\t')
                    line[0][j] = s[j];

            this->declare_parameter("/crystalfontz_driver/red_led_"+std::to_string(i+1), 0.0);
            led[i].r = this->get_parameter("/crystalfontz_driver/red_led_"+std::to_string(i+1)).as_double();
            led[i].r = std::clamp(led[i].r, 0.f, 1.f);

            this->declare_parameter("/crystalfontz_driver/green_led_"+std::to_string(i+1), 0.0);
            led[i].g = this->get_parameter("/crystalfontz_driver/green_led_"+std::to_string(i+1)).as_double();
            led[i].g = std::clamp(led[i].g, 0.f, 1.f);

        }
    }

private:
    void mainLoop() {
        sendPacket(0);
        if(!waitForAck()) {
            RCLCPP_INFO(get_logger(), "Timed out waiting for the ping response. Trying to reset serial connection...");

            Uninit_Serial();
            if (Serial_Init(SERIAL_PORT, BAUD)){
                RCLCPP_INFO(get_logger(), "Can not open port \"%s\" at \"%d\" baud.", SERIAL_PORT, BAUD);
            } else {
                RCLCPP_INFO(get_logger(), "\"%s\" opened at \"%d\" baud.\n\n", SERIAL_PORT, BAUD);
                contrastUpToDate = false;
                backlightPowerUpToDate = false;
                lineUpToDate = {false, false, false, false};
                ledUpToDate = {false, false, false, false};
            }

            return;
        }

        if( !contrastUpToDate
            || !backlightPowerUpToDate
            || !lineUpToDate[0]
            || !lineUpToDate[1]
            || !lineUpToDate[2]
            || !lineUpToDate[3]
            || !ledUpToDate[0]
            || !ledUpToDate[1]
            || !ledUpToDate[2]
            || !ledUpToDate[3]){

            // CLEAR CONSOLE
            std::system("clear");

            std::stringstream ss1, ss2, ss3, ss4;
            for(auto c : line[0]) ss1 << c;
            for(auto c : line[1]) ss2 << c;
            for(auto c : line[2]) ss3 << c;
            for(auto c : line[3]) ss4 << c;

            RCLCPP_INFO(get_logger(), "\
			\n                                 |--------------------|\
			\nled1 (r, g): %0.2f, %0.2f    line1 |%s|\
			\nled2 (r, g): %0.2f, %0.2f    line2 |%s|\
			\nled3 (r, g): %0.2f, %0.2f    line3 |%s|\
			\nled4 (r, g): %0.2f, %0.2f    line4 |%s|\
			\n                                 |--------------------|\
			\ncontrast:\t%i\
			\nbacklight:\t%i%%",
                        led[0].r,
                        led[0].g,
                        ss1.str().c_str(),
                        led[1].r,
                        led[1].g,
                        ss2.str().c_str(),
                        led[2].r,
                        led[2].g,
                        ss3.str().c_str(),
                        led[3].r,
                        led[3].g,
                        ss4.str().c_str(),
                        contrast,
                        backlightPower);
        }



        if(!contrastUpToDate){
            sendPacket(13, contrast); // contrast setting
            if(waitForAck()) contrastUpToDate = true;
            else return; // Timed out waiting for a response for LCD contrast.
        }

        if(!backlightPowerUpToDate){
            sendPacket(14, backlightPower); // backlight power setting
            if(waitForAck()) backlightPowerUpToDate = true;
            else return; // Timed out waiting for a response for LCD And Keypad Backlight.
        }

        for (int i = 0; i < 4; ++i) {
            if(!lineUpToDate[i]){
                sendPacket(31, 0, 0, line[i]);
                if(waitForAck()) lineUpToDate[i] = true;
                else return; // Timed out waiting for a response on line 1.
            }

            if(!ledUpToDate[i]){
                sendPacket(34, 12, (int)(led[i].r*100)); // red led 1
                bool timed_out1 = !waitForAck();

                sendPacket(34, 11, (int)(led[i].g*100)); // green led 1
                bool timed_out2 = !waitForAck();

                if (timed_out1 || timed_out2) return; // Timed out waiting for a response for led 1.
                else ledUpToDate[i] = true;
            }

        }


        if (check_for_packet()) {
            if(receivedKeyActivity()) {
                std_msgs::msg::Int8 keyActivity;
                keyActivity.data = getReceivedKeyActivity();
                pubKeyActivity->publish(keyActivity);
            } else {
                RCLCPP_ERROR(get_logger(), "Received non key activity packet. Type: %i", incoming_command.command);
            }
            showReceivedPacket();
        }
    }

    void contrastCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "contrastCallback: [%i]", msg->data);
        contrast = std::clamp(msg->data, 0, 254);
        contrastUpToDate = false;
    }

    void backlightPowerCallback(const std_msgs::msg::Int32::SharedPtr msg){
        RCLCPP_INFO(this->get_logger(), "backlightPowerCallback: [%i]", msg->data);
        backlightPower = std::clamp(msg->data, 0 , 100);
        backlightPowerUpToDate = false;
    }

    void line1Callback(const std_msgs::msg::String::SharedPtr msg){
//	ROS_INFO("line1Callback: [%s]", msg->data.c_str());
        std::string s = msg->data;
        s.resize(sizeof(line[0]), ' ');

        for(std::string::size_type i = 0; i < s.size() && i < sizeof(line[0]); i++)
            if(s[i] != '\t')
                line[0][i] = s[i];

        lineUpToDate[0] = false;
    }

    void line2Callback(const std_msgs::msg::String::SharedPtr msg){
//	ROS_INFO("line2Callback: [%s]", msg->data.c_str());
        std::string s = msg->data;
        s.resize(sizeof(line[1]), ' ');

        for(std::string::size_type i = 0; i < s.size() && i < sizeof(line[1]); i++)
            if(s[i] != '\t')
                line[1][i] = s[i];

        lineUpToDate[1] = false;
    }

    void line3Callback(const std_msgs::msg::String::SharedPtr msg){
//	ROS_INFO("line3Callback: [%s]", msg->data.c_str());
        std::string s = msg->data;
        s.resize(sizeof(line[2]), ' ');

        for(std::string::size_type i = 0; i < s.size() && i < sizeof(line[2]); i++)
            if(s[i] != '\t')
                line[2][i] = s[i];

        lineUpToDate[2] = false;
    }

    void line4Callback(const std_msgs::msg::String::SharedPtr msg){
//	ROS_INFO("line4Callback: [%s]", msg->data.c_str());
        std::string s = msg->data;
        s.resize(sizeof(line[3]), ' ');

        for(std::string::size_type i = 0; i < s.size() && i < sizeof(line[3]); i++)
            if(s[i] != '\t')
                line[3][i] = s[i];

        lineUpToDate[0] = false;
    }

    void led1Callback(const std_msgs::msg::ColorRGBA::SharedPtr msg){
//	ROS_INFO("led1Callback");
        led[0].r = std::clamp(msg->r, 0.f, 1.f);
        led[0].g = std::clamp(msg->g, 0.f, 1.f);
        led[0].b = std::clamp(msg->b, 0.f, 1.f);
        led[0].a = std::clamp(msg->a, 0.f, 1.f);
        ledUpToDate[0] = false;
    }

    void led2Callback(const std_msgs::msg::ColorRGBA::SharedPtr msg){
//	ROS_INFO("led2Callback");
        led[1].r = std::clamp(msg->r, 0.f, 1.f);
        led[1].g = std::clamp(msg->g, 0.f, 1.f);
        led[1].b = std::clamp(msg->b, 0.f, 1.f);
        led[1].a = std::clamp(msg->a, 0.f, 1.f);
        ledUpToDate[1] = false;
    }

    void led3Callback(const std_msgs::msg::ColorRGBA::SharedPtr msg){
//	ROS_INFO("led3Callback");
        led[2].r = std::clamp(msg->r, 0.f, 1.f);
        led[2].g = std::clamp(msg->g, 0.f, 1.f);
        led[2].b = std::clamp(msg->b, 0.f, 1.f);
        led[2].a = std::clamp(msg->a, 0.f, 1.f);
        ledUpToDate[2] = false;
    }

    void led4Callback(const std_msgs::msg::ColorRGBA::SharedPtr msg){
//	ROS_INFO("led4Callback");
        led[3].r = std::clamp(msg->r, 0.f, 1.f);
        led[3].g = std::clamp(msg->g, 0.f, 1.f);
        led[3].b = std::clamp(msg->b, 0.f, 1.f);
        led[3].a = std::clamp(msg->a, 0.f, 1.f);
        ledUpToDate[3] = false;
    }

    int contrast = 100;
    int backlightPower = 50;
    bool contrastUpToDate = false;
    bool backlightPowerUpToDate = false;

    std::array<char[20], 4> line;
    std::array<bool, 4> lineUpToDate;

    std::array<std_msgs::msg::ColorRGBA, 4> led;
    std::array<bool, 4> ledUpToDate;

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subContrast;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subBacklightPower;

    std::array<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr, 4> subLine;
    std::array<rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr, 4> subLed;

    std::string line_topic = "/crystalfontz/led_";
    std::string led_topic = "/crystalfontz/led_";

    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr pubKeyActivity;

    rclcpp::TimerBase::SharedPtr timer;
};


#endif //BUILD_DRIVERNODE_HPP
