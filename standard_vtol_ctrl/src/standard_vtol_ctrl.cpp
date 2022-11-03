#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandVtolTransition.h>
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    ros::Time last_request;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient transition_to_fw = nh.serviceClient<mavros_msgs::CommandVtolTransition>("mavros/cmd/vtol_transition");
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose;

    // nh.getParam("target_x",pose.pose.position.x);
    // nh.getParam("target_y",pose.pose.position.y);
    // nh.getParam("target_z",pose.pose.position.z);

    ros::param::get("~target_x", pose.pose.position.x);
    ros::param::get("~target_y", pose.pose.position.y);
    ros::param::get("~target_z", pose.pose.position.z);

    // pose.type_mask = 0b000111000000;
    // pose.coordinate_frame = 1;
    // pose.velocity.x = 1
    // pose.velocity.y = 0
    // pose.velocity.z = 0
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "AUTO.TAKEOFF";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandVtolTransition vtol_transition_fw;
    vtol_transition_fw.request.state = 4;

    // wait for FCU connection
    while (ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    //控制飞机arm
    while (ros::ok())
    {
        if (!current_state.armed)
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                printf("fuel_ctl: Vehicle armed\n");
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    //控制飞机起飞
    last_request = ros::Time::now();
    while (ros::ok())
    {
        // vtol takeoff need 50s from 0m to 80m, if takeoff duration is too long, uav will takeoff twice
        //多次发送起飞命令这个问题，等待解决.
        if (ros::Time::now() - last_request < ros::Duration(10.0)) //这个是维持发送起飞命令的时间,不需要调整
        {
            if (current_state.mode != "AUTO.TAKEOFF")
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    printf("fuel_ctl: start take off\n");
                }
            }
        }
        else
        {
            // uav will hold when it takeoff to target altitude 80m(also 50s), it will hold nutil 63s reached
            //这个时间是确保飞机起飞完成并且有悬停的调整， 下面这个时间必须大于实际起飞所用的时间。因为只有剩余的时间飞机才会悬停， 如果这个时间比实际起飞所用时间小， 那么你会看到飞机不会经过悬停这一步骤。
            if (current_state.mode == "AUTO.LOITER" && (ros::Time::now() - last_request > ros::Duration(30)))
            {
                printf("fuel_ctl: auto loiter\n");
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }
    //控制飞机从旋翼转变为固定翼
    last_request = ros::Time::now();
    while (ros::ok())
    {

        if (transition_to_fw.call(vtol_transition_fw) && vtol_transition_fw.response.success)
        {
            if (ros::Time::now() - last_request > ros::Duration(30))
            {
                printf("fuel_ctl: fw enabled do next\n");
                break;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    offb_set_mode.request.custom_mode = "OFFBOARD";
    last_request = ros::Time::now();
    while (ros::ok())
    {

        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                // break;

            }
        }
        
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
