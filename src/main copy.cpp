
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/PathGeometric.h>

#include "ConstrainedPlanningCommon.h"
#include "KinematicChain.h"
#include "panda_model_updater.h"
#include "jacobian.h"
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

#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/ompl_interface/model_based_planning_context.h>
#include <moveit/ompl_interface/planning_context_manager.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <boost/algorithm/string.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
unsigned int links = 14;
Eigen::Affine3d left_base = Eigen::Affine3d::Identity();
Eigen::Affine3d right_base = Eigen::Affine3d::Identity();


class ConstrainedKinematicChainValidityChecker : public KinematicChainValidityChecker
{
public:
    ConstrainedKinematicChainValidityChecker(const ob::ConstrainedSpaceInformationPtr &si)
        : KinematicChainValidityChecker(si)
    {
    }

    // bool isValid(const ob::State *state) const override
    // {
    //     auto &&space = si_->getStateSpace()->as<ob::ConstrainedStateSpace>()->getSpace()->as<KinematicChainSpace>();
    //     auto &&s = state->as<ob::ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
    //     return isValidImpl(space, s);
    // }
};
class PositionConstraint : public ob::Constraint
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    PositionConstraint(unsigned int links, Eigen::VectorXd start) : ob::Constraint(links, 1, 5e-3)
    {
        for (int i = 0; i < 7; i++)
        {
            left_qinit[i] = start[i];
            right_qinit[i] = start[i + 7];
        }

        left_arm = std::make_shared<FrankaModelUpdater>(left_q);
        right_arm = std::make_shared<FrankaModelUpdater>(right_q);
        rot_jaco_ = std::make_shared<rot_jaco>();

        left_init = left_arm->getTransform(left_qinit);
        right_init = right_arm->getTransform(right_qinit);
        init = left_init.inverse() * right_init;
    }

    /*actual constraint function, state "x" from the ambient space */
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {

        Eigen::VectorXd &&temp = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);

        Eigen::Affine3d lt = left_arm->getTransform(temp);
        Eigen::Affine3d rt = right_arm->getTransform(temp2);

        Eigen::Affine3d result = lt.inverse() * rt;

        out[0] = (result.translation() - init.translation()).squaredNorm(); //only position
        std::cout << "POSITION f : " << out << std::endl;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        Eigen::VectorXd &&temp = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);

        Eigen::Matrix<double, 3, 7> Jv_l = left_arm->getJacobian(temp).block<3, 7>(0, 0);
        Eigen::Matrix<double, 3, 7> Jv_r = right_arm->getJacobian(temp2).block<3, 7>(0, 0);
        Eigen::Affine3d lt = left_arm->getTransform(temp);
        Eigen::Affine3d rt = right_arm->getTransform(temp2);
        Eigen::Affine3d result = lt.inverse() * rt;

        Eigen::Vector3d p = rt.translation() - lt.translation();
        Eigen::Matrix<double, 3, 9> tmp;
        tmp.setZero();
        tmp.block<1, 3>(0, 0) = p.transpose();
        tmp.block<1, 3>(1, 3) = p.transpose();
        tmp.block<1, 3>(2, 6) = p.transpose();

        Eigen::Matrix<double, 3, 14> j;
        j.block<3, 7>(0, 0) = tmp * rot_jaco_->jaco(temp) - lt.linear() * Jv_l;
        j.block<3, 7>(0, 7) = lt.linear().transpose() * Jv_r;

        out.row(0) = 2 * (result.translation().transpose() - init.translation().transpose()) * j;
        // std::cout << out << std::endl;
    }

private:
    Eigen::Matrix<double, 7, 1> left_q, right_q;
    Eigen::Matrix<double, 7, 1> left_qinit, right_qinit;
    Eigen::Affine3d left_init, right_init, init;

    std::shared_ptr<FrankaModelUpdater> init_left_arm, left_arm;
    std::shared_ptr<FrankaModelUpdater> right_arm, init_right_arm;
    std::shared_ptr<rot_jaco> rot_jaco_;
    Eigen::Matrix<double, 4, 4> init_;
};


class KinematicChainConstraint : public ob::Constraint
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    KinematicChainConstraint(unsigned int links, Eigen::VectorXd start) : ob::Constraint(links, 1, 1)
    {
        for (int i = 0; i < 7; i++)
        {
            left_qinit[i] = start[i];
            right_qinit[i] = start[i + 7];
        }
        left_base.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);
        right_base.translation() = Eigen::Vector3d(0.0, -0.2, 0.0);

        left_arm = std::make_shared<FrankaModelUpdater>(left_q);
        right_arm = std::make_shared<FrankaModelUpdater>(right_q);

        left_init = left_base * left_arm->getTransform(left_qinit);
        right_init = right_base * right_arm->getTransform(right_qinit);
        init = left_init.inverse() * right_init;
        init_tr = init.linear().transpose();
    }

    /*actual constraint function, state "x" from the ambient space */
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {

        Eigen::VectorXd &&temp = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);

        Eigen::Affine3d lt = left_base * left_arm->getTransform(temp);
        Eigen::Affine3d rt = right_base * right_arm->getTransform(temp2);

        Eigen::Affine3d result = lt.inverse() * rt;
        
        
        double r;
        Eigen::Quaterniond cur_q(result.linear());
        Eigen::Quaterniond ori_q(init.linear());
        r = cur_q.angularDistance(ori_q);
        
        double d;
        d = (result.translation() - init.translation()).norm();
        
        // double d_rot;
        // d_rot = (  init_tr * result.linear() ).log().norm();
        
        out[0] = d + r;
        // cout << temp.transpose() << ' ' << temp2.transpose() << endl;
        std::cout << " translation : " << d << std::endl;
        // cout << "rotation : " << r << endl;
        // cout << "rotation : " << d_rot << endl;
        cout << out << endl;
    }

    /* his is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        double h = 1e-3;
        double d, d_rot;
        double r;
        Eigen::Quaterniond ori_q(init.linear());
        for (int i = 0; i < 14; i++)
        {
            Eigen::VectorXd &&q_temp = x;
            q_temp(i) = x(i) + h;
            Eigen::Affine3d lt_1 = left_base * left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_1 = right_base * right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_1 = lt_1.inverse() * rt_1;
            
            // d_rot = ( init_tr * result_1.linear() ).log().norm();

            Eigen::Quaterniond cur1_q(result_1.linear());
            r = cur1_q.angularDistance(ori_q);
            d = (result_1.translation() - init.translation()).norm();
            double g1 = d + r;

            q_temp(i) = x(i) - h;
            Eigen::Affine3d lt_2 = left_base * left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_2 = right_base * right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
            Eigen::Quaterniond cur2_q(result_2.linear());
            r = cur2_q.angularDistance(ori_q);
            d = (result_2.translation() - init.translation()).norm();
            // d_rot = ( init_tr * result_2.linear() ).log().norm();
            
            double g2 = d + r;
            out(0, i) = (g1 - g2) / (2 * h);
        }
    }

private:
    Eigen::Matrix<double, 7, 1> left_q, right_q;
    Eigen::Matrix<double, 7, 1> left_qinit, right_qinit;
    Eigen::Affine3d left_init, right_init, init, init_tr;

    std::shared_ptr<FrankaModelUpdater> init_left_arm, left_arm;
    std::shared_ptr<FrankaModelUpdater> right_arm, init_right_arm;
    std::shared_ptr<rot_jaco> rot_jaco_;
    Eigen::Matrix<double, 4, 4> init_;
};

bool planning(ConstrainedProblem &cp, ompl::geometric::PathGeometric &path)
{
    clock_t start_time = clock();
    ob::PlannerStatus stat = cp.solveOnce(true);
    clock_t end_time = clock();

    // if (stat)
    // {
    //     path = cp.ss->getSolutionPath();
    //     path.interpolate();
    // }
    OMPL_INFORM("PLANNING TIME : %d s", (end_time - start_time) / CLOCKS_PER_SEC);
    return stat;
}

ompl::geometric::PathGeometric plannedPath(Eigen::VectorXd start, Eigen::VectorXd goal)
{
    auto ss = std::make_shared<KinematicChainSpace>(links);
    enum SPACE_TYPE space = PJ; //"PJ", "AT", "TB"
    std::vector<enum PLANNER_TYPE> planners = {RRT}; //RRTConnect

    
    std::cout << "init state   : " << start.transpose() << std::endl;
    std::cout << "target state : " << goal.transpose() << std::endl;

    auto constraint = std::make_shared<KinematicChainConstraint>(links, start);

    ConstrainedProblem cp(space, ss, constraint); // define a simple problem to solve this constrained space
    cp.setConstrainedOptions();
    cp.setAtlasOptions();
    cp.setStartAndGoalStates(start, goal);
    cp.setPlanner(planners[0]);
    
    // Get the maximum value a call to distance() can return (or an upper bound). For unbounded state spaces, this function can return infinity. More
    cout << cp.css->getMaximumExtent() << endl; // 18.4372
    // cp.ss->setStateValidityChecker(std::make_shared<ConstrainedKinematicChainValidityChecker>(cp.csi));
    ompl::geometric::PathGeometric path(cp.csi);
    if (planning(cp, path))
        return path;
    else
        OMPL_ERROR("PLANNING FAILED");
}

void execute_path(std::string path_name, moveit::planning_interface::MoveGroupInterface& move_group)
{
    moveit_msgs::RobotTrajectory robot_trajectory;
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = move_group.getJointNames();

    ifstream path_file(path_name);
    while (path_file)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        bool eof = false;
        for (int j = 0; j < 14; j++)
        {
            double data;
            if (!(path_file >> data))
            {
                cout << "wrong file: " << j << endl;
                eof = true;
                break;
            }
            traj_point.positions.push_back(data);
        }
        if (eof)
            break;
        // traj_point.time_from_start = ros::Duration();
        joint_trajectory.points.push_back(traj_point);
    }

    double total_time = 10.0;
    int n_traj = joint_trajectory.points.size();
    for (int i = 0; i < n_traj; i++)
    {
        joint_trajectory.points[i].time_from_start = ros::Duration(total_time / n_traj * i);
    }

    robot_trajectory.joint_trajectory = joint_trajectory;
    cout << "PUBLISHED" << endl;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;
    plan.planning_time_ = total_time;
    move_group.execute(plan);
  
    cout << "EXECUTE" << endl;
}

int main(int argc, char **argv)
{
    std::string name_ = "dual_arm_constraint_planning";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    /* PLANNING AND RETURN PLANNED PATH */
    // Create a shared pointer to our constraint.
    Eigen::VectorXd start(links), goal(links);
    start.setZero();
    goal.setZero();
    start <<-1.278022704113855, 0.7599595568823166, 0.8504105865120947, -2.2532133470941753, -1.1298611922879744, 2.521625636015966, 2.2355349196620695,  0.025852137525551332, -0.012156381609418291, 0.28454053715882205, -1.996989239487609, 0.1902084893722976, 1.937095545026834, 2.0326259015811394;
    goal << 1.8724599095546757, -0.8876780299370965, -1.8366765471836668, -1.9215286517090047, 0.19624468801085224, 2.110239229947917, 2.1865444462085253, 1.9679636352538856, -0.6442305802620372, -1.8145061036978387, -1.8574305808262341, 0.6783518458876869, 2.0151744366993727, 2.113935474954216;

    
    ompl::geometric::PathGeometric path_(plannedPath(start, goal));
    

    ros::WallDuration(1.0).sleep();
    const std::string PLANNING_GROUP = "panda_arms";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    robot_state::RobotStatePtr robot_state = move_group.getCurrentState(); // plan_manager->robot_state_;
    robot_model::RobotModelConstPtr robot_model = move_group.getRobotModel();

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 5, true);
    while (display_publisher.getNumSubscribers() == 0 && ros::ok())
    {
        ros::spinOnce();
    }

    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*robot_state, display_trajectory.trajectory_start);
    display_trajectory.model_id = "panda";

    robot_trajectory::RobotTrajectory trajectory_(robot_state->getRobotModel(), "panda_arms");
    trajectory_.clear();
    moveit_msgs::RobotTrajectory robot_trajectory;
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = move_group.getJointNames();
    
    ifstream path_file("/home/jiyeong/catkin_ws/projection_path.txt");
    while (path_file)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        bool eof = false;
        for (int j = 0; j < 14; j++)
        {
            double data;
            if (!(path_file >> data))
            {
                cout << "wrong file: " << j << endl;
                eof = true;
                break;
            }
            traj_point.positions.push_back(data);
        }
        if (eof)
            break;
        // traj_point.time_from_start = ros::Duration();
        joint_trajectory.points.push_back(traj_point);
    }

    double total_time = 10.0;
    int n_traj = joint_trajectory.points.size();
    for (int i = 0; i < n_traj; i++)
    {
        joint_trajectory.points[i].time_from_start = ros::Duration(total_time / n_traj * i);
    }

    robot_trajectory.joint_trajectory = joint_trajectory;
    // trajectory_.setRobotTrajectoryMsg(*robot_state, joint_trajectory);
    // trajectory_.getRobotTrajectoryMsg(robot_trajectory);
    display_trajectory.trajectory.push_back(robot_trajectory);
    // display_publisher.publish(display_trajectory);
    cout << "PUBLISHED" << endl;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;
    plan.planning_time_ = total_time;
    move_group.execute(plan);
  
    cout << "EXECUTE" << endl;
/* ==============================================================================*/

    start.setZero();
    goal.setZero();
    start << 1.8724599095546757, -0.8876780299370965, -1.8366765471836668, -1.9215286517090047, 0.19624468801085224, 2.110239229947917, 2.1865444462085253, 1.9679636352538856, -0.6442305802620372, -1.8145061036978387, -1.8574305808262341, 0.6783518458876869, 2.0151744366993727, 2.113935474954216;
    goal<< -2.6507471333602557, -1.589255627751553, 1.5684394981602237, -1.7392113847304413, 2.8184155924430048, 0.3381303947470854, 2.049881477237161,  0.2195216325634586, 0.825405588554725, -0.739912114743606, -1.7326241018909547, 1.729176902034074, 0.8970502468391156, 1.5151332948318228;
    ompl::geometric::PathGeometric path2_(plannedPath(start, goal));
    ifstream path2_file("/home/jiyeong/catkin_ws/projection_path.txt");

    trajectory_.clear();
    while (path2_file)
    {
        trajectory_msgs::JointTrajectoryPoint traj_point;
        bool eof = false;
        for (int j = 0; j < 14; j++)
        {
            double data;
            if (!(path2_file >> data))
            {
                cout << "wrong file: " << j << endl;
                eof = true;
                break;
            }
            traj_point.positions.push_back(data);
        }
        if (eof)
            break;
        joint_trajectory.points.push_back(traj_point);
    }

    double total_time = 10.0;
    int n_traj = joint_trajectory.points.size();
    for (int i = 0; i < n_traj; i++)
    {
        joint_trajectory.points[i].time_from_start = ros::Duration(total_time / n_traj * i);
    }

    robot_trajectory.joint_trajectory = joint_trajectory;
    display_trajectory.trajectory.push_back(robot_trajectory);
    // display_publisher.publish(display_trajectory);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;
    plan.planning_time_ = total_time;
    move_group.execute(plan);
  
    cout << "EXECUTE" << endl;


    return 0;
}
