#include "tropicana.h"
#include <stdlib.h>

using namespace tropicana;
//int centroid_pose_size = 0;
uint8_t task = 0;
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

TROPICANA::TROPICANA()
    :node_handle_("")
{
    present_joint_angle.resize(NUM_OF_JOINT);
    present_kinematic_position.resize(3);

    initPublisher();
    initSubscriber();

    ROS_INFO("TROPICANA initialization");
    flag = false;
}
TROPICANA::~TROPICANA()
{
    if (ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    //wait();
}

void TROPICANA::initPublisher()
{
  // service client
    goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_joint_space_path");
    goal_task_space_path_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/open_manipulator/goal_task_space_path_position_only");
    goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_tool_control");
    //set_actuator_state_client_ = n.serviceClient<open_manipulator_msgs::SetActuatorState>("/open_manipulator/set_actuator_state");
    //goal_drawing_trajectory_client_ = n.serviceClient<open_manipulator_msgs::SetDrawingTrajectory>("/open_manipulator/goal_drawing_trajectory");
}
void TROPICANA::initSubscriber()
{
  //open_manipulator_states_sub_       = n.subscribe("/open_manipulator/states", 10, &QNode::statesCallback, this);
    chain_joint_states_sub_ = node_handle_.subscribe("/open_manipulator/joint_states", 10, &TROPICANA::jointStatesCallback, this);
    chain_kinematics_pose_sub_ = node_handle_.subscribe("/open_manipulator/kinematics_pose", 10, &TROPICANA::kinematicsPoseCallback, this);

    centroid_pose_array_sub = node_handle_.subscribe("/detection_3d/detection_3d", 1, &TROPICANA::centroidPoseArrayMsgCallback, this);
}

void TROPICANA::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
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

void TROPICANA::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
    std::vector<double> temp_position;
    temp_position.push_back(msg->pose.position.x);
    temp_position.push_back(msg->pose.position.y);
    temp_position.push_back(msg->pose.position.z);
    present_kinematic_position = temp_position;
    kinematics_pose_.pose = msg->pose;
}

void TROPICANA::centroidPoseArrayMsgCallback(const vision_msgs::Detection3DArray::ConstPtr &msg)
{
    if (task == MOVE_ARM_TO_CUT)
    {
        object_x.clear();
        object_y.clear();
        object_z.clear();

        std::vector<float> temp_object_x;
        std::vector<float> temp_object_y;
        std::vector<float> temp_object_z;
        std::vector<std::vector<float>> two_D_vector;

        geometry_msgs::Pose centroid_pose;
        ROS_INFO("const float defined");

        if (msg->detections.size() == 0)
        {
            task = INIT_POSITION;
            return;
            ROS_INFO("if box size=0");
        }
        else
        {
            for (int i = 0; i < msg->detections.size(); i++)
            {
                if (!((msg->detections.at(i).bbox.center.position.x == 0) && (msg->detections.at(i).bbox.center.position.y == 0) && (msg->detections.at(i).bbox.center.position.z == 0)))
                {
                    centroid_pose = msg->detections.at(i).bbox.center;

                    // Target point with respect to manipulator coordinate
                    temp_object_x.push_back(cos(rad_offset) * centroid_pose.position.x - sin(rad_offset) * centroid_pose.position.z + x_offset);
                    temp_object_y.push_back((centroid_pose.position.y + y_offset) * y_gain);
                    temp_object_z.push_back(sin(rad_offset) * centroid_pose.position.x + cos(rad_offset) * centroid_pose.position.z + z_offset + cutter_z_offset);

                    two_D_vector.push_back(std::vector<float>());
                    two_D_vector[i].push_back(temp_object_y[i]);
                    two_D_vector[i].push_back(temp_object_x[i]);
                    two_D_vector[i].push_back(temp_object_z[i]);
                    sort(two_D_vector.begin(), two_D_vector.end());

                    ROS_INFO("Target %d  Before transformation :  %.3f, %.3f, %.3f  ", i, centroid_pose.position.x, centroid_pose.position.y, centroid_pose.position.z);
                }
                else
                {
                    task = INIT_POSITION;
                }
            }
            object_x.push_back(two_D_vector[0][1]);
            object_y.push_back(two_D_vector[0][0]);
            object_z.push_back(two_D_vector[0][2]);

            ROS_INFO("After transformation : %.3f, %.3f, %.3f  ", object_x[0], object_y[0], object_z[0]);
        }
    }
    else
    {

    }

}

std::vector<double> TROPICANA::getPresentJointAngle()
{
    return present_joint_angle;
}
std::vector<double> TROPICANA::getPresentKinematicsPose()
{
    return present_kinematic_position;
}

bool TROPICANA::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
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

bool TROPICANA::setToolControl(std::vector<double> joint_angle)
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

bool TROPICANA::setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
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


void TROPICANA::cutter_open(double path_time)
{
    std::vector<double> joint_angle;
    joint_angle.push_back(0.0105);

    if (!TROPICANA::setToolControl(joint_angle))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }

    ROS_INFO("Send cutter open");
    sleep(path_time);
}

void TROPICANA::cutter_close(double path_time)
{
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);

    if (!TROPICANA::setToolControl(joint_angle))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }

    ROS_INFO("Send cutter close");
    sleep(path_time);
}

// void TROPICANA::objectPublisher(void)
// {
//   std_msgs::String msg;
//   msg.data = target_object.c_str();
//   target_object_pub_.publish(msg);
//   //ROS_INFO("send objectPublisher %s ", target_object.c_str());
// }

void TROPICANA::init_pose(double path_time)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

    joint_name.push_back("joint1"); joint_angle.push_back(0.000);  //0.000
    joint_name.push_back("joint2"); joint_angle.push_back(-1.295); //-1.050
    joint_name.push_back("joint3"); joint_angle.push_back(0.604);  //0.365
    joint_name.push_back("joint4"); joint_angle.push_back(0.589);  //0.600
    if (!setJointSpacePath(joint_name, joint_angle, path_time))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }
    ROS_INFO("Send joint angle to init pose");

    sleep(path_time);
}

void TROPICANA::place_pose(double path_time)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

    joint_name.push_back("joint1"); joint_angle.push_back(-2.750);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.483);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.106);
    joint_name.push_back("joint4"); joint_angle.push_back(0.422);
    if (!setJointSpacePath(joint_name, joint_angle, path_time))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }
    ROS_INFO("Send joint angle to place pose");

    sleep(path_time);
}

void TROPICANA::drop_pose(double path_time)
{
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

    joint_name.push_back("joint1"); joint_angle.push_back(-2.750);
    joint_name.push_back("joint2"); joint_angle.push_back(-0.483);
    joint_name.push_back("joint3"); joint_angle.push_back(-0.106);
    joint_name.push_back("joint4"); joint_angle.push_back(1.400);
    if (!setJointSpacePath(joint_name, joint_angle, path_time))
    {
        ROS_INFO("[ERR!!] Failed to send service");
        return;
    }
    ROS_INFO("Send joint angle to drop pose");

    sleep(path_time);
}

void TROPICANA::process(void)
{
    switch (task)
    {
        case INIT_POSITION:
            cutter_open(1);
            init_pose(2);
            task = MOVE_ARM_TO_CUT;
            break;

        case MOVE_ARM_TO_CUT:
            //object_x = object_x - 0.16 ;
            //ROS_INFO(" move %.3f, %.3f, %.3f ", object_x, object_y, object_z );

            goalPose.at(0) = object_x[0];//x
            goalPose.at(1) = object_y[0]; //y
            goalPose.at(2) = object_z[0]; //z
            if (!setTaskSpacePath(goalPose, 2))
            {
                task = INIT_POSITION;
                break;
            }
            sleep(2.2);

            // goalPose.at(0) = 0.279;//x
            // goalPose.at(1) = -0.009; //y
            // goalPose.at(2) = 0.204; //z
            // setTaskSpacePath(goalPose, 3);

            cutter_close(1);
            cutter_open(1);
            task = MOVE_ARM_TO_PLACE;
            break;

        case MOVE_ARM_TO_PLACE:
            init_pose(2);
            place_pose(2);
            drop_pose(1.5);
            place_pose(1.5);
            task = RETURN_POSITION;
            break;

        case RETURN_POSITION:
            init_pose(2);
            task = MOVE_ARM_TO_CUT;
            break;

        default:
            break;
    }

}

int main(int argc, char **argv)
{
  // Init ROS node
    ros::init(argc, argv, "TROPICANA");
    ros::NodeHandle priv_nh("~");

    TROPICANA tropicana_;
    tropicana_.goalPose = tropicana_.getPresentKinematicsPose();
    // tropicana_.init_pose();
    // sleep(2);
    // tropicana_.goalPose.at(0) = 0.100 ;//x
    // tropicana_.goalPose.at(1) = 0.000; //y
    // tropicana_.goalPose.at(2) = 0.270 ; //z
    // tropicana_.setTaskSpacePath(tropicana_.goalPose, 2);
    // sleep(2);
    // tropicana_.gripper_close();
    // sleep(2);
    // tropicana_.gripper_open();

    ros::Rate loop_rate(25);

    while (ros::ok())
    {
        ros::spinOnce();

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

        tropicana_.process();
        loop_rate.sleep();
    }

    ROS_INFO("TROPICANA finished");
    return 0;
}