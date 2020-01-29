
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

class OrientationConstraint : public ob::Constraint
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    OrientationConstraint(unsigned int links, Eigen::VectorXd start) : ob::Constraint(links, 1, 1)
    {
        for (int i = 0; i < 7; i++)
        {
            left_qinit[i] = start[i];
            right_qinit[i] = start[i + 7];
        }
        left_arm = std::make_shared<FrankaModelUpdater>(left_q);
        right_arm = std::make_shared<FrankaModelUpdater>(right_q);

        left_init = left_arm->getTransform(left_qinit);
        right_init = right_arm->getTransform(right_qinit);
        init = left_init.inverse() * right_init;

        std::cout << (init.linear().transpose() * init.linear() - Eigen::Matrix3d::Identity()).norm() << std::endl;
    }

    /*actual constraint function, state "x" from the ambient space */
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        Eigen::VectorXd &&temp = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);

        Eigen::Affine3d lt = left_arm->getTransform(temp);
        Eigen::Affine3d rt = right_arm->getTransform(temp2);

        Eigen::Affine3d result = lt.inverse() * rt;
        out[0] = (result.linear().transpose() * init.linear() - Eigen::Matrix3d::Identity()).norm();

        std::cout << "ORIENTATION f : " << out << std::endl;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        double h = 0.01;
        for (int i = 0; i < 14; i++)
        {
            Eigen::VectorXd &&q_temp = x;
            q_temp(i) = x(i) + h;
            Eigen::Affine3d lt_1 = left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_1 = right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_1 = lt_1.inverse() * rt_1;
            double g1 = (result_1.linear().transpose() * init.linear() - Eigen::Matrix3d::Identity()).norm();

            q_temp(i) = x(i) - h;
            Eigen::Affine3d lt_2 = left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_2 = right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
            double g2 = (result_2.linear().transpose() * init.linear() - Eigen::Matrix3d::Identity()).norm();
            out(0, i) = (g1 - g2) / (2 * h);
        }
    }

private:
    Eigen::Matrix<double, 7, 1> left_q, right_q;
    Eigen::Matrix<double, 7, 1> left_qinit, right_qinit;
    Eigen::Affine3d left_init, right_init, init;

    std::shared_ptr<FrankaModelUpdater> init_left_arm, left_arm;
    std::shared_ptr<FrankaModelUpdater> right_arm, init_right_arm;
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
        
        double r, d;
        Eigen::Quaterniond cur_q(result.linear());
        Eigen::Quaterniond ori_q(init.linear());
        r = cur_q.angularDistance(ori_q);
        d = (result.translation() - init.translation()).norm();
        
        // double d_rot;
        // d_rot = (  init_tr * result.linear() ).log().norm();
        
        // r = cur_q.dot(ori_q);
        // d =  (result.translation() - init.translation()).squaredNorm();

        out[0] = d + r;
        // cout << temp.transpose() << ' ' << temp2.transpose() << endl;
        // std::cout << " translation : " << d << std::endl;
        // cout << "rotation : " << r << endl;
        // cout << "rotation : " << d_rot << endl;
        // cout << "trans origin : " << init.translation().transpose() << endl
            //  << "trans current: " << result.translation().transpose() << endl;
    }

    /* his is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        double h = 1e-3;
        double r, d, d_rot;
        Eigen::Quaterniond ori_q(init.linear());
        for (int i = 0; i < 14; i++)
        {
            Eigen::VectorXd &&q_temp = x;
            q_temp(i) = x(i) + h;
            Eigen::Affine3d lt_1 = left_base * left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_1 = right_base * right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_1 = lt_1.inverse() * rt_1;
            Eigen::Quaterniond cur1_q(result_1.linear());

            // d_rot = ( init_tr * result_1.linear() ).log().norm();
            r = cur1_q.angularDistance(ori_q);
            d = (result_1.translation() - init.translation()).norm();
            // d =  (result_1.translation() - init.translation()).squaredNorm();
            double g1 = d + r;

            q_temp(i) = x(i) - h;
            Eigen::Affine3d lt_2 = left_base * left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_2 = right_base * right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
            Eigen::Quaterniond cur2_q(result_2.linear());
            r = cur2_q.angularDistance(ori_q);
            d = (result_2.translation() - init.translation()).norm();
            // d_rot = ( init_tr * result_2.linear() ).log().norm();
            
            // d =  (result_2.translation() - init.translation()).squaredNorm();
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

ompl::geometric::PathGeometric plannedPath()
{
    auto ss = std::make_shared<KinematicChainSpace>(links);
    enum SPACE_TYPE space = PJ; //"PJ", "AT", "TB"
    std::vector<enum PLANNER_TYPE> planners = {RRTConnect};

    // Create a shared pointer to our constraint.
    Eigen::VectorXd start(links), goal(links);
    start.setZero();
    goal.setZero();
    
    // start << 0.43879842840079253, 0.6382337298947351, -0.5754996808185072, -2.2044480704908374, -2.406856818012579, 3.6105131803103907, -2.0596210802552366, 2.2405497196597017, -0.8977498890476259, -1.360060919844714, -1.982898876524701, -0.7227069072897616, 1.7667101769756277, 2.89261940667946;
    // goal << -1.86502619472071, 0.42220874521268065, 1.6843138982477912, -2.5612303268037295, -0.21903199608536952, 2.2953892726156653, 1.8306450379136778, 2.1151440739365803, -1.0720649459946567, -1.2938631767371478, -1.6965945498750754, -0.5232065110791524, 1.5449860482660118, 2.6382132931217175;
    
    // start << -1.731687943523212, 1.6596897792834329, 1.6755381678608345, -2.448617084593089, -1.2336515104478012, 1.5062962092174126, 2.5407480396093587, 0.42023360149356837, 0.07578672395956589, -0.2801165758285413, -1.6979509960234709, 0.5055300131612396, 1.5519386111121753, 1.885272932257551;
    start << -1.86502619472071, 0.42220874521268065, 1.6843138982477912, -2.5612303268037295, -0.21903199608536952, 2.2953892726156653, 1.8306450379136778, 2.1151440739365803, -1.0720649459946567, -1.2938631767371478, -1.6965945498750754, -0.5232065110791524, 1.5449860482660118, 2.6382132931217175;
    goal << -1.1883966192292919, 0.47654684635278594, 1.188900403834024, -2.043110376072939, 0.22786012603711361, 2.0359730912954896, 1.9358084609099853, 2.2170307858961844, -1.2280571039695196, -1.4167808509505564, -1.4485510862730249, -0.41639268409727337, 1.6864723603999285, 2.66725585105675;

    // start << -1.1883966192292919, 0.47654684635278594, 1.188900403834024, -2.043110376072939, 0.22786012603711361, 2.0359730912954896, 1.9358084609099853, 2.2170307858961844, -1.2280571039695196, -1.4167808509505564, -1.4485510862730249, -0.41639268409727337, 1.6864723603999285, 2.66725585105675;
    // goal << -1.1839009766884918, 0.5989827756945714, 1.2419279406857917, -1.8941837370096823, 0.5124819117075965, 2.0246317808603576, 2.104747264710482, 1.9311295581266086, -1.461368971663593, -1.4059743203337784, -1.8847327577656103, -0.3470843084329271, 2.14022655344604, 2.76799537584971;

    // start<< -1.1839009766884918, 0.5989827756945714, 1.2419279406857917, -1.8941837370096823, 0.5124819117075965, 2.0246317808603576, 2.104747264710482, 1.9311295581266086, -1.461368971663593, -1.4059743203337784, -1.8847327577656103, -0.3470843084329271, 2.14022655344604, 2.76799537584971;
    // goal << -1.283366045643523, 0.7765156182977472, 1.2207072053353922, -1.8138368218093084, 0.5720248354896629, 1.864962889712384, 2.013001741822463, 1.7869467393780099, -1.7461996310469938, -1.5103788661534288, -1.9024155978863702, -0.306624896948462, 2.0792647705952243, 2.5495489263789413;

    // start << -1.283366045643523, 0.7765156182977472, 1.2207072053353922, -1.8138368218093084, 0.5720248354896629, 1.864962889712384, 2.013001741822463, 1.7869467393780099, -1.7461996310469938, -1.5103788661534288, -1.9024155978863702, -0.306624896948462, 2.0792647705952243, 2.5495489263789413;
    // goal << -1.3221287249665774, 0.9009694143728474, 1.1966835297856593, -1.7257454180007339, 0.6045753542044264, 1.740045581988698, 1.9942301147497448,  1.7329642169027442, -1.7597508080616242, -1.6463380442829965, -1.8396026654012847, -0.09169653782940333, 2.0070409700798195, 2.304969253475799;

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

int main(int argc, char **argv)
{
    std::string name_ = "dual_arm_constraint_planning";
    ros::init(argc, argv, name_);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");

    /* PLANNING AND RETURN PLANNED PATH */
    ompl::geometric::PathGeometric path_(plannedPath());
    ifstream path_file("/home/jiyeong/catkin_ws/projection_path.txt");


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

    return 0;
}
