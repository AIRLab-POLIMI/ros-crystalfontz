#ifndef BUILD_LOGICNODE_HPP
#define BUILD_LOGICNODE_HPP

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include <unistd.h>
#include <csignal>
#include <sstream>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

#include "tree.hh"
#include "tree_util.hh"

#define LINE_4_TOPIC			"/crystalfontz/line_4"
#define LED_1_TOPIC				"/crystalfontz/led_1"
#define LED_2_TOPIC				"/crystalfontz/led_2"
#define LED_3_TOPIC				"/crystalfontz/led_3"
#define LED_4_TOPIC				"/crystalfontz/led_4"
#define KEY_ACTIVITY_TOPIC		"/crystalfontz/key_activity"

enum KeyActivity {
    KEY_UP_PRESS		= 1,
    KEY_DOWN_PRESS		= 2,
    KEY_LEFT_PRESS		= 3,
    KEY_RIGHT_PRESS		= 4,
    KEY_ENTER_PRESS		= 5,
    KEY_EXIT_PRESS		= 6,
    KEY_UP_RELEASE		= 7,
    KEY_DOWN_RELEASE	= 8,
    KEY_LEFT_RELEASE	= 9,
    KEY_RIGHT_RELEASE	= 10,
    KEY_ENTER_RELEASE	= 11,
    KEY_EXIT_RELEASE	= 12
};

enum MenuState {
    DISABLED,
    ENABLED,
    RUNNING_SCRIPT
};

enum DISPLAY_CHARS {
    KEYS_CHAR 			= ' ',
    RIGHT_ARROW_CHAR 	= 16,
    LEFT_ARROW_CHAR 	= 17,
    UP_ARROW_CHAR 		= 26,
    DOWN_ARROW_CHAR 	= 27,
    ENTER_CHAR 			= 141,
    EXIT_CHAR			= 120
};

MenuState menuState = DISABLED;
pid_t currentScriptPid = -1;

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace boost::filesystem;

class LogicNode : public rclcpp::Node {

public:
    LogicNode() : Node("crystalfontz_logic") {

        pubLine4 = this->create_publisher<std_msgs::msg::String>(LINE_4_TOPIC, 10);

        pubLed1 = this->create_publisher<std_msgs::msg::ColorRGBA>(LED_1_TOPIC, 10);
        pubLed2 = this->create_publisher<std_msgs::msg::ColorRGBA>(LED_2_TOPIC, 10);
        pubLed3 = this->create_publisher<std_msgs::msg::ColorRGBA>(LED_3_TOPIC, 10);
        pubLed4 = this->create_publisher<std_msgs::msg::ColorRGBA>(LED_4_TOPIC, 10);

        subKeyActivity = this->create_subscription<std_msgs::msg::Int8>(KEY_ACTIVITY_TOPIC, 10,
                                                                        std::bind(&LogicNode::keyActivityCallback, this,
                                                                                  _1));
    }

    bool initialization() {
        std::string rootFolderPathName;

        this->declare_parameter("/crystalfontz_logic/menu_folder", "");
        rootFolderPathName = this->get_parameter("/crystalfontz_driver/contrast").as_string();

        if(rootFolderPathName.empty()) {
            RCLCPP_ERROR(get_logger(), "Missing script folder in configuration");
            return false;
        }

        path rootFolderPath(rootFolderPathName);

        if(!is_directory(rootFolderPath)){
            RCLCPP_ERROR(get_logger(), "Script folder %s does not exists or is not a directory", rootFolderPathName.c_str());
            return false;
        }

        folderRoot = folderTree.insert(folderTree.begin(), rootFolderPath);
        selectedEntry = folderRoot;

        expandFolder(folderTree, folderRoot);

        std::less<path> pathComparator;
        folderTree.sort(folderTree.begin(), folderTree.end(), pathComparator, true);

        std::cout << "Available scripts:" << std::endl;
        print_tree(folderTree, folderTree.begin(), folderTree.end());

        if(selectedEntry != folderTree.end() && tree<path>::begin(selectedEntry) != tree<path>::end(selectedEntry)) {
            selectedEntry = tree<path>::begin(selectedEntry);
        }

        sleep(2);

        std_msgs::msg::String line4;
        line4.data = "press ENTER for menu";
	    pubLine4->publish(line4);

        return true;
    }

private:
    void expandFolder(tree<path> fTree, tree<path>::iterator folderNode){

        try {
            path p(*folderNode);

            if (exists(p)) {
                if (is_directory(p)) {
                    for(auto& entry : boost::make_iterator_range(directory_iterator(p), {})){
                        path entryPath = entry.path();//.string();

                        if (is_regular_file(entry)){
                            if(entry.path().extension() == ".sh") {
                                tree<path>::iterator folderChild = fTree.append_child(folderNode, entryPath);
                            }
                        } else {
                            tree<path>::iterator folderChild = fTree.append_child(folderNode, entryPath);
                            expandFolder(fTree, folderChild);
                        }
                    }
                }
            }

        } catch (const filesystem_error& ex) {
            RCLCPP_INFO(get_logger(), "%s", ex.what());
        }

    }


    static void print_tree(const tree<path>& tr, tree<path>::pre_order_iterator it, tree<path>::pre_order_iterator end){
        if(!tr.is_valid(it)) return;
        int rootdepth=tree<path>::depth(it);
        std::cout << "-----" << std::endl;
        while(it!=end) {
            for(int i=0; i<tree<path>::depth(it)-rootdepth; ++i) std::cout << "  ";
            std::cout << (*it) << std::endl << std::flush;
            ++it;
        }
        std::cout << "-----" << std::endl;
    }

    bool has_parent(tree<path>::iterator folderNode){
        tree<path>::iterator parent = tree<path>::parent(folderNode);
        return parent != folderTree.begin();
    }

    static bool has_prev_sibling(tree<path>::iterator folderNode){
        tree<path>::sibling_iterator siblings = folderNode;
        return --siblings != siblings.end();
    }

    static bool has_next_sibling(tree<path>::iterator folderNode){
        tree<path>::sibling_iterator siblings = folderNode;
        return ++siblings != siblings.end();
    }

    bool has_children(tree<path>::iterator folderNode){
        return folderNode != folderTree.end() && folderTree.begin(folderNode) != folderTree.end(folderNode);
    }


    std::string formatLineEntry(tree<path>::iterator folderNode){
        std::string s = folderNode->stem().string();
        s.resize(20, ' ');

        s[15] = KEYS_CHAR;
        s[16] = has_parent(selectedEntry)?LEFT_ARROW_CHAR:' ';
        s[17] = has_prev_sibling(selectedEntry)?UP_ARROW_CHAR:' ';
        s[18] = has_next_sibling(selectedEntry)?DOWN_ARROW_CHAR:' ';
        s[19] = has_children(selectedEntry)?RIGHT_ARROW_CHAR:(is_regular_file(*selectedEntry)?ENTER_CHAR:' ');

        RCLCPP_INFO(get_logger(), "formatLineEntry: \t[%s]", s.c_str());

        return s;
    }



    void keyActivityCallback(const std_msgs::msg::Int8::SharedPtr msg) {
        std_msgs::msg::String line4;

        switch(menuState){

            case RUNNING_SCRIPT:

                if(msg->data == KEY_EXIT_RELEASE){
                    menuState = ENABLED;
                    RCLCPP_INFO(get_logger(), "EXIT");
                    int failedKill = kill(currentScriptPid, 15);  // Sends the SIGTERM Signal to the process, telling it to stop.

                    if(failedKill) RCLCPP_INFO(get_logger(), "Can not kill script with pid: %i; It may be already dead.", currentScriptPid);
                    else RCLCPP_INFO(get_logger(), "Sent SIGTERM to the script (pid: %i)", currentScriptPid);

                    RCLCPP_INFO(get_logger(), "Menu enabled");
                    RCLCPP_INFO(get_logger(), "current entry:\t\t%s", (*selectedEntry).c_str());
                    currentScriptPid = -1;
                    line4.data = formatLineEntry(selectedEntry);
                    pubLine4->publish(line4);
                }
                break;

            case DISABLED:

                if(msg->data == KEY_ENTER_RELEASE){
                    menuState = ENABLED;
                    RCLCPP_INFO(get_logger(), "Menu enabled");
                    RCLCPP_INFO(get_logger(), "current entry:\t\t%s", (*selectedEntry).c_str());
                    line4.data = formatLineEntry(selectedEntry);
                    pubLine4->publish(line4);
                }
                break;

            case ENABLED:

                tree<path>::iterator parent = tree<path>::parent(selectedEntry);
                tree<path>::sibling_iterator siblings = selectedEntry;

                switch(msg->data){
                    case KEY_UP_RELEASE:
                        if(--siblings != siblings.end()){
                            RCLCPP_INFO(get_logger(), "UP");
                            selectedEntry = siblings;
                            RCLCPP_INFO(get_logger(), "current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
                            line4.data = formatLineEntry(selectedEntry);
                            pubLine4->publish(line4);
                        }
                        break;

                    case KEY_DOWN_RELEASE:
                        if(++siblings != siblings.end()){
                            RCLCPP_INFO(get_logger(), "DOWN");
                            selectedEntry = siblings;
                            RCLCPP_INFO(get_logger(), "current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
                            line4.data = formatLineEntry(selectedEntry);
                            pubLine4->publish(line4);
                        }
                        break;

                    case KEY_LEFT_RELEASE:
                        if(parent != folderTree.begin()){
                            RCLCPP_INFO(get_logger(), "LEFT");
                            selectedEntry = parent;
                            RCLCPP_INFO(get_logger(), "current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
                            line4.data = formatLineEntry(selectedEntry);
                            pubLine4->publish(line4);
                        }
                        break;

                    case KEY_RIGHT_RELEASE:
                        if(selectedEntry!=folderTree.end() && folderTree.begin(selectedEntry) != folderTree.end(selectedEntry)) {
                            RCLCPP_INFO(get_logger(), "RIGHT");
                            selectedEntry = folderTree.begin(selectedEntry);
                            RCLCPP_INFO(get_logger(), "current entry:\t\t%s %s", (*selectedEntry).c_str(), has_children(selectedEntry)?">":"·");
                            line4.data = formatLineEntry(selectedEntry);
                            pubLine4->publish(line4);
                        }
                        break;

                    case KEY_ENTER_RELEASE:
                        if(is_regular_file(path(*selectedEntry))){
                            RCLCPP_INFO(get_logger(), "ENTER");

                            currentScriptPid = fork();
                            if(currentScriptPid == 0) {
                                RCLCPP_INFO(get_logger(), "Script %s started", (*selectedEntry).c_str());

                                int r = execl("/bin/bash", "/bin/bash", (*selectedEntry).c_str(), (char*) nullptr);

                                RCLCPP_INFO(get_logger(), "execl terminated with return value %i", r);
                                exit(1);
                            }

                            std::stringstream ssr;
                            ssr << "Running " << (*selectedEntry).stem().c_str() << "...";
                            line4.data = ssr.str();
                            line4.data.resize(20, ' ');
                            line4.data[18] = KEYS_CHAR;
                            line4.data[19] = EXIT_CHAR;
                            pubLine4->publish(line4);

                            menuState = RUNNING_SCRIPT;
                            RCLCPP_INFO(get_logger(), "Executing script %s; pid: %i", (*selectedEntry).c_str(), currentScriptPid);

                        }
                        break;

                    case KEY_EXIT_RELEASE:
                        RCLCPP_INFO(get_logger(), "EXIT");
                        RCLCPP_INFO(get_logger(), "Menu disabled");
                        line4.data = "press ENTER for menu";
                        pubLine4->publish(line4);
                        menuState = DISABLED;

                        break;

                }

                break;
        }
    }

    tree<path> folderTree;
    tree<path>::iterator folderRoot, selectedEntry;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubLine4;

    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pubLed1;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pubLed2;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pubLed3;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr pubLed4;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr subKeyActivity;

};

#endif //BUILD_LOGICNODE_HPP
