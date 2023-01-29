#include <iostream>
#include <termios.h>
#include "std_msgs/msg/string.hpp"
#include <sstream> 
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

#define fontColorReset "\033[0m"
#define fontColorGreenBold "\033[1m\033[32m"
#define fontColorYellowBold "\033[1m\033[33m"
#define fontColorBlueBold "\033[1m\033[34m"
#define fontColorMagentaBold "\033[1m\033[35m"
#define fontColorCyanBold "\033[1m\033[36m"

int ScanKeyboard()
{
	int in;
 
	struct termios new_settings;
	struct termios stored_settings;
    //设置终端参数
	tcgetattr(0,&stored_settings);
	new_settings = stored_settings;
	new_settings.c_lflag &= (~ICANON);
	new_settings.c_cc[VTIME] = 0;
	tcgetattr(0,&stored_settings);
	new_settings.c_cc[VMIN] = 1;
	tcsetattr(0,TCSANOW,&new_settings);
	in = getchar();
	tcsetattr(0,TCSANOW,&stored_settings);
    
	return in;
 
}

void keyboard_watching(char *char_zimu)
{
    while (rclcpp::ok())
    {
        *char_zimu = (char)ScanKeyboard();
    }
    
}

class MyNode: public rclcpp::Node{
public:
    MyNode():Node("node_name")
    {
        RCLCPP_INFO(this->get_logger(),"cmd_vel_control!");
    }

};

int main(int argc,char** argv)
{   
    setlocale(LC_ALL,"");
    rclcpp::init(argc,argv);
    geometry_msgs::msg::Twist set_moving(char keyboard_input);
    // auto ros2_node = std::make_shared<MyNode>();
    rclcpp::Node::SharedPtr ros2_node = nullptr;
    ros2_node = rclcpp::Node::make_shared("cmd_vel_control_node");

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    cmd_publisher_ = ros2_node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    char keyboard_input = 's';
    // ros::Publisher input_pub= nh.advertise<std_msgs::String>("keyboard_input", 1, true);
    std::thread xiancheng1(&keyboard_watching, &keyboard_input);


    rclcpp::Rate loop_rate(40);
    
    std::cout << fontColorCyanBold << "lsc手写控制: " << fontColorReset << std::endl;
    std::cout << fontColorCyanBold << "键入档位决定速度如: 1、2、3、4、5、6 " << fontColorReset << std::endl;
    std::cout << fontColorCyanBold << "键入q、w、e、a、s、d、z、x、c方向指行 " << fontColorReset << std::endl;
    std::cout << fontColorCyanBold << "键入j、k、l旋转车体 " << fontColorReset << std::endl;

    geometry_msgs::msg::Twist my_twist;
    
    while(rclcpp::ok())
    {   
//         std::string project_path = ${PROJECT_PATH};

//         char *path;
// if((path = getenv("MAIN_PAOTH")))
// printf("MAIN_PAOTH =%s/n",path);


        static char last_input='s';
        char current_input = keyboard_input;
        
        if( last_input != current_input )
        {   
            std::cout << "\r更新输入: " << current_input << std::endl ;//自我接收显示
            last_input = current_input;
        }
        else std::cout << "\r";//自我接收显示
        
        
        cmd_publisher_->publish( set_moving( current_input ) );

        loop_rate.sleep();
    }

    xiancheng1.detach();
    // xiancheng1.join();
    rclcpp::shutdown();
    return 0;
}

geometry_msgs::msg::Twist set_moving(char keyboard_input) // 返回计算出的Twist
{   //首次获取the_way容器里对象的索引
    static float kp=0.4;
    static float x=0;
    static float y=0;
    float angular_v=0;
    
    bool fail_key_flag=false;
    switch (keyboard_input)
    {
        //反方向抑制
        case '1':   kp=0.4;break;
        case '2':   kp=0.8;break;
        case '3':   kp=1.2;break;
        case '4':   kp=1.6;break;
        case '5':   kp=2.1;break;
        case '6':   kp=2.7;break;
        case 'q':   x=1;y=1;break;
        case 'w':   x=1;y=0;break;
        case 'e':   x=1;y=-1;break;
        case 'a':   x=0;y=1.1;break;            
        case 's':   x=0;y=0;fail_key_flag=true;break;
        case 'd':   x=0;y=-1;break;
        case 'z':   x=-1;y=1;break;
        case 'c':   x=-1;y=-1;break;
        case 'x':   x=-1;y=0;break;
        case 'j':   x=0;y=0;angular_v=0.72;break;
        case 'k':   angular_v=0;break;
        case 'l':   x=0;y=0;angular_v=-0.72;break;
        default:    std::cout << "输入无效" ;fail_key_flag=true;break;
    }
    geometry_msgs::msg::Twist my_twist;
    
    my_twist.linear.x=x*kp;
    my_twist.linear.y=y*kp;
    my_twist.linear.z=0;
    my_twist.angular.x=0;
    my_twist.angular.y=0;
    my_twist.angular.z=angular_v;//angular.z=0;//直行   =-0.5;//+逆时针转圈

    if(fail_key_flag)
    my_twist.linear.x=0,
    my_twist.linear.y=0,
    my_twist.linear.z=0,
    my_twist.angular.x=0,
    my_twist.angular.y=0,
    my_twist.angular.z=0;//angular.z=0;//直行   =-0.5;//+逆时针转圈

    return my_twist;
}