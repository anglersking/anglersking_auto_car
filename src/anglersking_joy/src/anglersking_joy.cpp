#include "anglersking_joy.h"


anglersking_joy::anglersking_joy()
{
    joystick_fd = -1;
    leftVertical    = rightVertical  = 0;
    rightHorizontal = leftHorizontal = 0;
    publicTopic = true;

    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("joystick_device", joystick_device, "/dev/input/js0");
    nh_private.param<float>("maxLinear_x", maxLinear_x, 0.4);
    nh_private.param<float>("maxLinear_y", maxLinear_y, 0.4);
    nh_private.param<float>("maxAngular_z", maxAngular_z, 1.5);

    ros::NodeHandle n;
    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);

    joystick_fd = open(joystick_device.c_str(), O_RDONLY | O_NONBLOCK); /* read write for force feedback? */
    if (joystick_fd < 0)
    {
        ROS_INFO("Open joystick device success!");
    }
}

 anglersking_joy::~anglersking_joy()
{
    close(joystick_fd);
}


void anglersking_joy::publish_joystick_event()
{
    ros::Rate rosSleep(20);
    while(ros::ok())
    {
        int bytes = read(joystick_fd, &jse, sizeof(jse));
        if(bytes != sizeof(jse) && bytes != -1)
        {
            ROS_INFO("Read joystick file sizeof error!");
        }
        jse.type &= ~JS_EVENT_INIT;
        if (jse.type == JS_EVENT_AXIS)
        {
            switch (jse.number)
            {
                case 0: this->leftHorizontal  = jse.value; break;
                case 1: this->leftVertical    = jse.value; break;
                case 2: this->rightHorizontal = jse.value; break;
                case 3: this->rightVertical   = jse.value; break;

                default: break;
            }
        }
        if (jse.type == JS_EVENT_BUTTON && jse.value == JS_EVENT_BUTTON_DOWN)
        {
            switch(jse.number)
            {
                case 4:maxLinear_x += 0.1;
                    break; 
                case 6:(maxLinear_x > 0.1)?(maxLinear_x -= 0.1):(maxLinear_x = maxLinear_x);
                    break;

                case 1:maxLinear_y += 0.1;
                    break;
                case 3:(maxLinear_y > 0.1)?(maxLinear_y -= 0.1):(maxLinear_y = maxLinear_y);
                    break;

                case 5:maxAngular_z += 0.1;
                    break;
                case 7:(maxAngular_z > 0.1)?(maxAngular_z -= 0.1):(maxAngular_z = maxAngular_z);
                    break;

                case 8: publicTopic = !publicTopic;
                        if(publicTopic)
                            {ROS_INFO("Open publish cmd_vel!");}
                        else
                            {ROS_INFO("Close publish cmd_vel!");}
                    break;
                default: break;   
            }
            if(jse.number != 8)
            {
                ROS_INFO("maxX, maxY, maxZ = [%f %f %f]",maxLinear_x, maxLinear_y, maxAngular_z);
            }
        }
        memset(&jse, 0, sizeof(jse));
        if (publicTopic)
        {
            vel_msg_.linear.y  = maxLinear_y /32767 * leftHorizontal  * -1;
            vel_msg_.linear.x  = maxLinear_x /32767 * leftVertical    * -1;
            vel_msg_.angular.z = maxAngular_z/32767 * rightHorizontal * -1;
            
            cmd_vel_pub.publish(vel_msg_);
        }


        rosSleep.sleep();
        ros::spinOnce();
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joy_controller");
    ROS_INFO("[Wanshan.pang] joy controller node start! ");

    anglersking_joy  anglersking_joystick;
    anglersking_joystick.publish_joystick_event();
}
