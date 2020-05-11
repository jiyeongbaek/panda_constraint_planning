#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

#include <urdf_parser/urdf_parser.h>

#include <sstream>
#include <algorithm>
#include <ctype.h>
#include <fstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/shape_operations.h>

#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <cmath>
#include <ctime>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/ExecuteTrajectoryGoal.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>

#include <constraint_planner/kinematics/grasping point.h>
int main(int argc, char **argv)
{
    std::string name_ = "hiiiiiiiiiiii";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::WallDuration(1.0).sleep();
    bool robot_model_ok_;
    robot_model::RobotModelPtr robot_model;
    std::shared_ptr<planning_scene::PlanningScene> planning_scene;
    collision_detection::AllowedCollisionMatrixPtr acm_;
    moveit_msgs::PlanningScene moveit_scene;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model = robot_model_loader.getModel();
    planning_scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    acm_ = std::make_shared<collision_detection::AllowedCollisionMatrix>(planning_scene->getAllowedCollisionMatrix());

    std::vector<double> joint_values = {-1.6671076987319884, -1.4984785565179355, 1.5404192524883924, -2.388776541995507, -2.897203095390305, 3.280959665933266, -1.0941728565641882,
                                        -0.10243983084379964, 0.2659588901612104, 0.4127700947518499, -1.3902073234890953, 0.06790555501862428, 1.5908404988928444, 2.0916124777614624};
    robot_state::RobotState &current_state = planning_scene->getCurrentStateNonConst();
    current_state.setJointGroupPositions(current_state.getJointModelGroup("panda_arms"), joint_values);

    moveit_msgs::AttachedCollisionObject stefan_obj;
    stefan_obj.link_name = "panda_left_hand";
    stefan_obj.object.header.frame_id = "panda_left_hand";
    stefan_obj.object.id = "stefan"; /* The id of the object is used to identify it. */
    stefan_obj.touch_links = {"panda_left_hand", "panda_right_hand"};
    shapes::Mesh *m = shapes::createMeshFromResource("file:///home/jiyeong/catkin_ws/src/1_assembly/grasping_point/STEFAN/stl/assembly.stl");
    shape_msgs::Mesh stefan_mesh;
    shapes::ShapeMsg stefan_mesh_msg;
    shapes::constructMsgFromShape(m, stefan_mesh_msg);
    stefan_mesh = boost::get<shape_msgs::Mesh>(stefan_mesh_msg);

    geometry_msgs::Pose stefan_pose;
    grasping_point grp;
    
    Eigen::Quaterniond quat(grp.Lgrasp_obj.linear());
    stefan_pose.orientation.x = quat.x();
    stefan_pose.orientation.y = quat.y();
    stefan_pose.orientation.z = quat.z();
    stefan_pose.orientation.w = quat.w();

    stefan_pose.position.x = grp.Lgrasp_obj.translation()[0];
    stefan_pose.position.y = grp.Lgrasp_obj.translation()[1];
    stefan_pose.position.z = grp.Lgrasp_obj.translation()[2];


    stefan_obj.object.meshes.push_back(stefan_mesh);
    stefan_obj.object.mesh_poses.push_back(stefan_pose);
    stefan_obj.object.operation = stefan_obj.object.ADD;

    moveit_scene.robot_state.attached_collision_objects.push_back(stefan_obj);
    moveit_scene.is_diff = true;
    planning_scene->processAttachedCollisionObjectMsg(stefan_obj);

    acm_->setEntry("panda_left_hand", "stefan", true);
    acm_->setEntry("panda_right_hand", "stefan", true);


    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base");
    // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst());
    visual_tools.trigger();
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    moveit_msgs::PlanningScene moveit_scene2;
    planning_scene->getPlanningSceneMsg(moveit_scene2);

    std::cout << "before get subscribers" << std::endl;
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }
    std::cout << "get subscribers" << std::endl;

    planning_scene_diff_publisher.publish(moveit_scene2);
}