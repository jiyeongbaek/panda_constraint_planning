
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
using namespace std;
unsigned int links = 14;
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
            out(1, i) = (g1 - g2) / (2 * h);
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

        Eigen::Quaterniond cur_q(result.linear());
        Eigen::Quaterniond ori_q(init.linear());

        double r, d;
        r = cur_q.angularDistance(ori_q);
        // d =  (result.translation() - init.translation()).norm();

        // r = cur_q.dot(ori_q);
        d =  (result.translation() - init.translation()).squaredNorm();

        out[0] = d ;
        cout << temp.transpose() << ' ' << temp2.transpose() << endl;
        std::cout <<" translation : " << d << std::endl;
        cout << "rotation : " << r << endl;
        cout << "trans origin : " << init.translation().transpose() << endl <<
        "trans current: " << result.translation().transpose() << endl;
    }

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        double h = 1e-3;
        double r, d;
        Eigen::Quaterniond ori_q(init.linear());
        for (int i = 0; i < 14; i++)
        {
            Eigen::VectorXd &&q_temp = x;
            q_temp(i) = x(i) + h;
            Eigen::Affine3d lt_1 = left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_1 = right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_1 = lt_1.inverse() * rt_1;
            Eigen::Quaterniond cur1_q(result_1.linear());

            r = cur1_q.angularDistance(ori_q);
            // d = (result_1.translation() - init.translation()).norm();
            d =  (result_1.translation() - init.translation()).squaredNorm();
            double g1 = d;

            q_temp(i) = x(i) - h;
            Eigen::Affine3d lt_2 = left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_2 = right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
            Eigen::Quaterniond cur2_q(result_2.linear());
            r = cur2_q.angularDistance(ori_q);
            // d =  (result_2.translation() - init.translation()).norm();
            d =  (result_2.translation() - init.translation()).squaredNorm();
            double g2 = d;
            out(0, i) = (g1 - g2) / (2 * h);
        }
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


bool planning(ConstrainedProblem &cp, ompl::geometric::PathGeometric &path)
{
    clock_t start_time = clock();
    ob::PlannerStatus stat = cp.solveOnce(true);
    clock_t end_time = clock();

    if (stat)
    {
        auto filename = boost::str(boost::format("kinematic_path_%i.dat") % cp.constraint->getAmbientDimension());
        OMPL_INFORM("Dumping problem information to `%s`.", filename.c_str());
        path = cp.ss->getSolutionPath();
        path.interpolate();
        std::ofstream pathfile(filename);
        path.printAsMatrix(pathfile);
        pathfile.close();
    }
    OMPL_INFORM("PLANNING TIME : %d s", (end_time - start_time)/CLOCKS_PER_SEC );
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

    // start << -0.8341104291882545, 1.2348735641071011, 1.00537157614749, -1.5289294407228247, -1.1881258342823753, 2.0753347982508044, 2.8207232078189586, 0.23372551002806874, 0.7142164410055684, 0.2943199726958308, -0.9409683977537915, -0.02818681091432607, 1.7094465432762362, 2.773245418577058;
    // goal << 2.4443141664406918, -0.7673093281937359, -2.379190146240761, -1.6439317270097529, -0.6482301722330581, 2.153191655830718, 2.614877323088198, 0.24477327245525315, 0.6224009518447352, 0.31530620113160374, -0.8941445687627276, -0.022117751989636114, 1.5761200141723637, 2.814443901055794;
   
    /* 물체 위로 5cm 들어올리기 */
    // start << -0.2039824467099883, 0.8568271883119626, 0.15046922862278453, -1.6059929079303625, -0.22539502881358264, 2.45265689629173, 2.38696627265447, 0.14880794584377596, 0.5493389741288717, 0.1728073603783121, -1.234449106233624, 0.08602683642364185, 1.823904246916223, 2.592454017382943;
    // goal << -0.1511489060513658, 0.7532571983041805, 0.09234352812635636, -1.6395574290511066, -0.13617879965165014, 2.3932660617279002, 2.3341114295753145, 0.8810467773045515, 0.858100466722242, -1.204158944056235, -1.0263559923072698, 0.9693709913828003, 1.4521482035526039, 2.286352131489894;
    
    // 1.1, -0.10 , 0.65 quaternion_from_euler(radians(10), radians(3), 0)
    // start <<2.161974362615302, -1.6211196400330166, -1.7370390401453095, -1.5145536456387945, -1.639059571337155, 1.7287910073202897, 2.8647268907512977, -0.34287270702454675, 0.7475154698699429, 0.9675946552366127, -1.2194944171770716, -0.44037976125558553, 1.7682065389509205, 2.8417982281458145;
    // goal <<2.4752881033585994, -0.852844128195467, -2.3460912587601706, -1.5495557841216463, -0.49894268575366557, 2.1031176731095025, 2.5476681567538626, -0.00903865011690246, 0.8356112965243518, 0.527152356760939, -0.5868581487969777, -0.032474656825517534, 1.424038493301814, 2.6704731926736804;
  
    // 1.1, -0.10 , 0.65 quaternion_from_euler(radians(20), radians(3), 0)
    // start <<2.4752881033585994, -0.852844128195467, -2.3460912587601706, -1.5495557841216463, -0.49894268575366557, 2.1031176731095025, 2.5476681567538626, -0.00903865011690246, 0.8356112965243518, 0.527152356760939, -0.5868581487969777, -0.032474656825517534, 1.424038493301814, 2.6704731926736804;
    // goal << 2.6554383117985405, -0.6816408244851703, -2.665377798656694, -1.5116285946954708, 0.024393693444898175, 2.058478422563071, 2.221224419650049, 0.5311219766699259, 1.1261012449220025, -1.7337105661131391, -0.7747824135537849, 1.9354537077921568, 0.9956705306532477, 2.288920589428489;

    // SEQUENCE 1
    start <<-0.2898127935814623, 0.871604960258511, 0.24968300596548199, -1.5992729965601837, -0.339007367747758, 2.4331858591580806, 2.4516225060104455, 0.3936058566580344, 0.5503626134904521, -0.1774017991083653, -1.2298923196557998, 0.27534317080631115, 1.8049455015020976, 2.502355869377598;
    // goal << 0.482352378896337, 0.6900549038955983, 0.4869100442023136, -1.638750492886491, -0.0891037418181836, 2.248044166974658, 2.2830497566297954, 0.22125341084164832, 0.48001989748524093, -0.131519874476992, -1.1682381916584446, 0.5013927287918933, 1.6757236994299995, 2.3779897883090135;
    
    // SEQUENCE 2
    // start << 0.482352378896337, 0.6900549038955983, 0.4869100442023136, -1.638750492886491, -0.0891037418181836, 2.248044166974658, 2.2830497566297954, 0.22125341084164832, 0.48001989748524093, -0.131519874476992, -1.1682381916584446, 0.5013927287918933, 1.6757236994299995, 2.3779897883090135;
    goal << -0.6864821861463876, 0.6275805016749498, 0.6350060778824758, -1.493347242007332, 0.14565167466175735, 1.9504166454346545, 2.123856677338268, -0.025493962777267544, 0.4974417218157958, -0.11566585378996927, -1.0538657580430102, 0.7555793957606083, 1.4690750295785182, 2.2367731874563015;

    // start << -0.6864821861463876, 0.6275805016749498, 0.6350060778824758, -1.493347242007332, 0.14565167466175735, 1.9504166454346545, 2.123856677338268, -0.025493962777267544, 0.4974417218157958, -0.11566585378996927, -1.0538657580430102, 0.7555793957606083, 1.4690750295785182, 2.2367731874563015;
    // goal << -0.8207871103501763, 0.7882167189851564, 0.6964016612131573, -1.1537469259134738, 0.344119178725943, 1.5870514099177153, 2.04163875371791, -0.3250191787143147, 0.6654150212893929, -0.08055724917378417, -0.9727408818425071, 1.04822819984304, 1.2646961949388844, 2.0584532029228324;
    std::cout << "init state   : " << start.transpose() << std::endl;
    std::cout << "target state : " << goal.transpose() << std::endl;

    auto constraint = std::make_shared<KinematicChainConstraint>(links, start);

    ConstrainedProblem cp(space, ss, constraint); // define a simple problem to solve this constrained space
    cp.setConstrainedOptions();
    cp.setAtlasOptions();
    cp.setStartAndGoalStates(start, goal);
    cp.setPlanner(planners[0]);
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
    
    ros::WallDuration(1.0).sleep();
    const std::string PLANNING_GROUP = "panda_arms";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        
    robot_state::RobotStatePtr robot_state = move_group.getCurrentState(); // plan_manager->robot_state_;
    robot_model::RobotModelConstPtr robot_model = move_group.getRobotModel();
  
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 5, true);
    while (display_publisher.getNumSubscribers() == 0 && ros::ok())    {ros::spinOnce();}
    moveit_msgs::DisplayTrajectory display_trajectory;
    robot_state::robotStateToRobotStateMsg(*robot_state, display_trajectory.trajectory_start);
    display_trajectory.model_id = "panda";
    
    robot_trajectory::RobotTrajectory trajectory_(robot_state->getRobotModel(), "panda_arms");
    trajectory_.clear();
    moveit_msgs::RobotTrajectory robot_trajectory;
    
    /* PLANNING AND RETURN PLANNED PATH */
    ompl::geometric::PathGeometric path_(plannedPath());
    
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = move_group.getJointNames();

    ifstream path_file("/home/jiyeong/catkin_ws/ompl_path.txt");
    char inputString[1000];
    int index = 0;

    while (path_file){
        trajectory_msgs::JointTrajectoryPoint traj_point;
        // path_file.getline(inputString, 1000);
		// char* tok1 = strtok(inputString," ");
        bool eof = false;
        for (int j = 0 ; j <14 ; j++){
            double data;
            if( ! (path_file >> data) ){ cout << "wrong file: " << j << endl; eof=true; break; }
            traj_point.positions.push_back(data);
        }
        if(eof) break;
        // traj_point.time_from_start = ros::Duration();
        joint_trajectory.points.push_back(traj_point);
        index ++;
    }
    
    double total_time = 10.0;
    int n_traj = joint_trajectory.points.size();
    for(int i=0; i<n_traj; i++)
    {
        joint_trajectory.points[i].time_from_start = ros::Duration(total_time / n_traj * i);
    }
    // while (index < 22){
    //     trajectory_msgs::JointTrajectoryPoint traj_point;
    //     path_file.getline(inputString, 1000);
        
	// 	char* tok1 = strtok(inputString," ");
    //     for (int j = 0 ; j <14 ; j++){
    //         traj_point.positions.push_back(atof(tok1));
    //         tok1 = strtok(NULL, " ");
    //     }
    //     traj_point.time_from_start = ros::Duration();
    //     joint_trajectory.points.push_back(traj_point);
    //     index ++;
    // }
    robot_trajectory.joint_trajectory = joint_trajectory;
    trajectory_.setRobotTrajectoryMsg(*robot_state, joint_trajectory);
   
    trajectory_.getRobotTrajectoryMsg(robot_trajectory);
    display_trajectory.trajectory.push_back(robot_trajectory);
    display_publisher.publish(display_trajectory);
    cout << "PUBLISHED" << endl;
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = robot_trajectory;
    plan.planning_time_ = total_time;
    move_group.execute(plan);
    // move_group.move();
    
    cout << "EXECUTE" << endl;

return 0;
}
