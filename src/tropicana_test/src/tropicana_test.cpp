#include "tropicana_test.h"
#include <math.h>
#include <stdlib.h>

using namespace tropicana_test;
geometry_msgs::Pose centroid_pose[10];
int centroid_pose_size = 0;
uint8_t task = 0, pre_task = 0;
std::vector<float> object_x;
std::vector<float> object_y;
std::vector<float> object_z;


enum
{
    INIT_POSITION,
    MOVE_ARM_TO_CUT,
    MOVE_ARM_TO_PLACE,
    RETURN_POSITION,
};

TROPICANA_TEST::TROPICANA_TEST()
    :node_handle_("")
{
    present_joint_angle.resize(NUM_OF_JOINT);
    present_kinematic_position.resize(3);

    initPublisher();
    initSubscriber();

    ROS_INFO("Tropicana_test initialization");
    flag = false;
}
TROPICANA_TEST::~TROPICANA_TEST()
{
    if (ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    //wait();
}

void TROPICANA_TEST::initPublisher()
{
  // service client
    goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_joint_space_path");
    goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/open_manipulator/goal_task_space_path_position_only");
    goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_tool_control");
    //set_actuator_state_client_ = n.serviceClient<open_manipulator_msgs::SetActuatorState>("/open_manipulator/set_actuator_state");
    //goal_drawing_trajectory_client_ = n.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("/open_manipulator/goal_drawing_trajectory");
}
void TROPICANA_TEST::initSubscriber()
{
  //open_manipulator_states_sub_       = n.subscribe("/open_manipulator/states", 10, &QNode::statesCallback, this);
    chain_joint_states_sub_ = node_handle_.subscribe("/open_manipulator/joint_states", 10, &TROPICANA_TEST::jointStatesCallback, this);
    chain_kinematics_pose_sub_ = node_handle_.subscribe("/open_manipulator/kinematics_pose", 10, &TROPICANA_TEST::kinematicsPoseCallback, this);

    centroid_pose_array_sub = node_handle_.subscribe("/centroid_pose_array", 10, &TROPICANA_TEST::centroidPoseArrayMsgCallback, this);
}

void TROPICANA_TEST::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
    std::vector<double> temp_angle;
    temp_angle.resize(NUM_OF_JOINT);
    for (int i = 0; i < msg->name.size(); i++)
    {
        if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
        else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
        else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
        else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    }
    present_joint_angle = temp_angle;
}

void TROPICANA_TEST::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
    std::vector<double> temp_position;
    temp_position.push_back(msg->pose.position.x);
    temp_position.push_back(msg->pose.position.y);
    temp_position.push_back(msg->pose.position.z);
    present_kinematic_position = temp_position;
    kinematics_pose_.pose = msg->pose;
}

void TROPICANA_TEST::centroidPoseArrayMsgCallback(const vision_msgs::BoundingBox3DArray &msg)
{
  //ROS_INFO("SAVE POSE OF centroidPoseArray");

    if (task == MOVE_ARM_TO_CUT)
    {
    //if( 1 == 1 ) {

    /*float x_offset = 0.07;
    float y_offset = -0.015;
    float z_offset = 0.017;*/
        const float x_offset = 0.0;     //단위 : 미터
        const float y_offset = 0.1;
        const float z_offset = 0.02;
        const float cut_height = 0.05;
        const float pi = 3.1415926;
        const float theta = 30 * pi / 180; //각도 측정 필요

        if (msg.boxes.size() == 0)
        {
            centroid_pose_size = 0;
            task = INIT_POSITION;
            //모바일 로봇 전진
            return;
        }

        for (int i = 0; i < msg.boxes.size(); i++)
        {
            if (!((msg.boxes[i].center.position.x == 0) && (msg.boxes[i].center.position.y == 0) && (msg.boxes[i].center.position.z == 0)))
            {
                centroid_pose[i] = msg.boxes[i].center;

                object_x.push_back(cos(theta) * centroid_pose[i].position.x - sin(theta) * centroid_pose[i].position.z);
                object_y.push_back(centroid_pose[i].position.y + y_offset);
                object_z.push_back(sin(theta) * centroid_pose[i].position.x + cos(theta) * centroid_pose[i].position.z + z_offset + cut_height);

                ROS_INFO("coord %d _  %.3f, %.3f, %.3f _  %.3f, %.3f, %.3f  ", i, object_x[i], object_y[i], object_z[i], centroid_pose[i].position.x, centroid_pose[i].position.y, centroid_pose[i].position.z);

                //centroid_pose_size = centroid_pose_size + 1;	
                //task = MOVE_ARM_TO_CUT;
            }
            else
            {
                task = INIT_POSITION;
            }
                // if( centroid_pose_size >= 10){
                // 	break ;
                // }
        }
    }
    else
    {

    }

}

std::vector<double> TROPICANA_TEST::getPresentJointAngle()
{
    return present_joint_angle;
}
std::vector<double> TROPICANA_TEST::getPresentKinematicsPose()
{
    return present_kinematic_position;
}

bool TROPICANA_TEST::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name = joint_name;
    srv.request.joint_position.position = joint_angle;
    srv.request.path_time = path_time;

    if (goal_joint_space_path_client_.call(srv))
    {
        return srv.response.is_planned;
    }
    return false;
}

bool TROPICANA_TEST::setToolControl(std::vector<double> joint_angle)
{
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name.push_back(node_handle_.param<std::string>("end_effector_name", "gripper"));
    srv.request.joint_position.position = joint_angle;

    if (goal_tool_control_client_.call(srv))
    {
        return srv.response.is_planned;
    }
    return false;
}

bool TROPICANA_TEST::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
{
    open_manipulator_msgs::SetKinematicsPose srv;

    srv.request.end_effector_name = "gripper";

    if (std::isnan(kinematics_pose.at(0)) || std::isnan(kinematics_pose.at(1)) || std::isnan(kinematics_pose.at(2)))
    {

        std::cerr << "Warning Error: NaN value operation.";
        return false;
    }

    srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
    srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
    srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

    srv.request.kinematics_pose.pose.orientation.w = kinematics_pose_.pose.orientation.w;
    srv.request.kinematics_pose.pose.orientation.x = kinematics_pose_.pose.orientation.x;
    srv.request.kinematics_pose.pose.orientation.y = kinematics_pose_.pose.orientation.y;
    srv.request.kinematics_pose.pose.orientation.z = kinematics_pose_.pose.orientation.z;

    ROS_INFO("-- x( %.3lf, %.3lf) y( %.3lf, %.3lf) z( %.3lf, %.3lf) \n", kinematics_pose.at(0), srv.request.kinematics_pose.pose.position.x,
            kinematics_pose.at(1), srv.request.kinematics_pose.pose.position.y, kinematics_pose.at(2), srv.request.kinematics_pose.pose.position.z);
    srv.request.path_time = path_time;

    if (goal_task_space_path_position_only_client_.call(srv))
    {
        return srv.response.is_planned;
    }
    return false;
}


void TROPICANA_TEST::cutter_open(double path_time)
{
    std::vector<double> joint_angle;
    joint_angle.push_back(0.006);

    if (!TROPICANA_TEST::setToolControl(joint_angle))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }

    ROS_INFO("Send cutter open");
    sleep(path_time);
}

void TROPICANA_TEST::cutter_close(double path_time)
{
    std::vector<double> joint_angle;
    joint_angle.push_back(0.003);

    if (!TROPICANA_TEST::setToolControl(joint_angle))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }

    ROS_INFO("Send cutter close");
    sleep(path_time);
}

// void TROPICANA_TEST::objectPublisher(void)
// {
//   std_msgs::String msg;
//   msg.data = target_object.c_str();
//   target_object_pub_.publish(msg);
//   //ROS_INFO("send objectPublisher %s ", target_object.c_str());
// }

void TROPICANA_TEST::init_pose(double path_time)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

    joint_name.push_back("joint1"); joint_angle.push_back(0.044);  //0.000
    joint_name.push_back("joint2"); joint_angle.push_back(-0.617); //-1.050
    joint_name.push_back("joint3"); joint_angle.push_back(-0.344);  //0.365
    joint_name.push_back("joint4"); joint_angle.push_back(0.600);  //0.600
    if (!setJointSpacePath(joint_name, joint_angle, path_time))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }
    ROS_INFO("Send joint angle to init pose");

    sleep(path_time);
}

void TROPICANA_TEST::place_pose(double path_time)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

    joint_name.push_back("joint1"); joint_angle.push_back(-2.605);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.695);
    joint_name.push_back("joint3"); joint_angle.push_back(0.293);
    joint_name.push_back("joint4"); joint_angle.push_back(0.199);
    if (!setJointSpacePath(joint_name, joint_angle, path_time))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }
    ROS_INFO("Send joint angle to place pose");

    sleep(path_time);
}

void TROPICANA_TEST::drop_pose(double path_time)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

    joint_name.push_back("joint1"); joint_angle.push_back(-2.605);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.695);
    joint_name.push_back("joint3"); joint_angle.push_back(0.293);
    joint_name.push_back("joint4"); joint_angle.push_back(1.049);
    if (!setJointSpacePath(joint_name, joint_angle, path_time))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }
    ROS_INFO("Send joint angle to drop pose");

    sleep(path_time);
}

void TROPICANA_TEST::process(void)
{
    switch (task)
    {
        case INIT_POSITION:
            cutter_open(1);
            init_pose(4);
            task = MOVE_ARM_TO_CUT;
            break;

        case MOVE_ARM_TO_CUT:
            //object_x = object_x - 0.16 ;
            //ROS_INFO(" move %.3f, %.3f, %.3f ", object_x, object_y, object_z );

            // goalPose.at(0) = object_x[0] ;//x
            // goalPose.at(1) = object_y[0]; //y
            // goalPose.at(2) = object_z[0] ; //z
            // if(!setTaskSpacePath(goalPose, 2))
            // {
            //     task = INIT_POSITION;
            //     break;
            // }

            goalPose.at(0) = 0.146;//x
            goalPose.at(1) = 0.129; //y
            goalPose.at(2) = 0.282; //z

            setTaskSpacePath(goalPose, 3);
            sleep(3);
            cutter_close(1);
            cutter_open(1);
            task = MOVE_ARM_TO_PLACE;
            break;

        case MOVE_ARM_TO_PLACE:
            place_pose(4);
            drop_pose(2);
            place_pose(2);
            task = RETURN_POSITION;
            break;

        case RETURN_POSITION:
            init_pose(4);
            task = MOVE_ARM_TO_CUT;
            break;

        default:
            break;
    }

}

int main(int argc, char **argv)
{
  // Init ROS node
    ros::init(argc, argv, "TROPICANA_TEST start");
    ros::NodeHandle priv_nh("~");

    TROPICANA_TEST tropicana_test_;
    tropicana_test_.goalPose = tropicana_test_.getPresentKinematicsPose();
    // tropicana_test_.init_pose();
    // sleep(2);
    // tropicana_test_.goalPose.at(0) = 0.100 ;//x
    // tropicana_test_.goalPose.at(1) = 0.000; //y
    // tropicana_test_.goalPose.at(2) = 0.270 ; //z
    // tropicana_test_.setTaskSpacePath(tropicana_test_.goalPose, 2);
    // sleep(2);
    // tropicana_test_.gripper_close();
    // sleep(2);
    // tropicana_test_.gripper_open();

    ros::Rate loop_rate(25);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        // ROS_INFO("task : %d", task);
        switch (task)
        {
            case INIT_POSITION:
                ROS_INFO("task : INIT_POSITION");
                break;
            case MOVE_ARM_TO_CUT:
                ROS_INFO("task : MOVE_ARM_TO_CUT");
                break;
            case MOVE_ARM_TO_PLACE:
                ROS_INFO("task : MOVE_ARM_TO_PLACE");
                break;
            case RETURN_POSITION:
                ROS_INFO("task : RETURN_POSITION");
                break;
        }

        tropicana_test_.process();
    }

    ROS_INFO("TROPICANA_TEST finished");
    return 0;
}