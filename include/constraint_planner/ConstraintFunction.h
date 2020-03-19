
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

// #include <ompl/base/Constraint.h>
#include <constraint_planner/Constraint.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <constraint_planner/ConstrainedPlanningCommon.h>
#include <constraint_planner/KinematicChain.h>
#include <constraint_planner/panda_model_updater.h>

#include <ctime>

#include <boost/algorithm/string.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;

unsigned int links = 14;
Eigen::Affine3d left_base = Eigen::Affine3d::Identity();
Eigen::Affine3d right_base = Eigen::Affine3d::Identity();

Eigen::Matrix3d skew_symmetric(Eigen::Vector3d x)
{
    Eigen::Matrix3d result;
    result.setZero();
    result(0, 1) = -x[2];
    result(0, 2) = x[1];
    result(1, 0) = x[2];
    result(1, 2) = -x[0];
    result(2, 0) = -x[1];
    result(2, 1) = x[0];

    return result;
}

class KinematicChainConstraint2 : public Constraint_new
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    KinematicChainConstraint2(unsigned int links, Eigen::VectorXd start) : Constraint_new(links, 6)
    {
        for (int i = 0; i < 7; i++)
        {
            left_qinit[i] = start[i];
            right_qinit[i] = start[i + 7];
        }
        left_base.translation() = Eigen::Vector3d(0.0, 0.3, 0.0);
        right_base.translation() = Eigen::Vector3d(0.0, -0.3, 0.0);

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

        Eigen::VectorXd p = (result.translation() - init.translation());

        Eigen::Matrix3d r_diff = init.linear().transpose() * result.linear();
        Eigen::AngleAxisd r_angleaxis(r_diff);
        Eigen::Vector3d r = r_angleaxis.axis() * r_angleaxis.angle();

        r = lt.linear().inverse() * right_init.linear() * r;
        // Eigen::VectorXd r = result.linear().eulerAngles(0, 1, 2) - init.linear().eulerAngles(0, 1, 2); // radian
        // Eigen::AngleAxisd r_current(Eigen::Quaterniond(result.linear()));
        // Eigen::AngleAxisd r_init(Eigen::Quaterniond(init.linear()));
        // Eigen::Vector3d r_current_ = r_current.axis() * r_current.angle();
        // Eigen::Vector3d r_init_ = r_init.axis() * r_init.angle();
        // Eigen::Vector3d r = r_current_ - r_init_;
        
        // std::cout << "1 : " << result.linear().eulerAngles(0, 1, 2).transpose() << std::endl;
        // std::cout << "2 : " <<init.linear().eulerAngles(0, 1, 2).transpose() << std::endl;
        // std:;cout << "3 : " <<r.transpose() << std::endl;
        // std::cout << "translation : " << p.transpose() << " // " << p.norm() << std::endl;
        // std::cout << "rotation    : " << r.transpose() << " // " << r.norm() << std::endl;
        out[0] = p[0];
        out[1] = p[1];
        out[2] = p[2];
        out[3] = r[0];
        out[4] = r[1];
        out[5] = r[2];
        // std::cout << out.segment(0, 3).squaredNorm() << " | " << out.segment(3, 3).squaredNorm() << std::endl;
    }

    /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        Eigen::VectorXd &&q_temp = x;
        Eigen::Affine3d lt = left_base * left_arm->getTransform(q_temp.segment(0, 7));
        Eigen::Affine3d rt = right_base * right_arm->getTransform(q_temp.segment(7, 7));
        Eigen::Affine3d result = lt.inverse() * rt;

        Eigen::MatrixXd jaco_l = left_arm->getJacobian(q_temp.segment(0, 7));
        Eigen::MatrixXd jaco_r = right_arm->getJacobian(q_temp.segment(7, 7));
        
        Eigen::MatrixXd left_omega = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd right_omega = Eigen::MatrixXd::Zero(6, 6);
        left_omega.block<3, 3>(0, 0) = lt.linear().inverse();
        left_omega.block<3, 3>(3, 3) = lt.linear().inverse();
        right_omega.block<3, 3>(0, 0) = result.linear() * rt.linear().inverse();
        right_omega.block<3, 3>(3, 3) = result.linear() * rt.linear().inverse();
        
        Eigen::MatrixXd wrench = Eigen::MatrixXd::Zero(6, 6);
        wrench.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        wrench.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        wrench.block<3, 3>(0, 3) = -skew_symmetric(result.translation());

        Eigen::MatrixXd jaco(6, 14);
        jaco.block<6, 7>(0, 0) = -wrench * left_omega * jaco_l;
        jaco.block<6, 7>(0, 7) = right_omega * jaco_r;
        out = jaco;
    }

private:
    Eigen::Matrix<double, 7, 1> left_q, right_q;
    Eigen::Matrix<double, 7, 1> left_qinit, right_qinit;
    Eigen::Affine3d left_init, right_init, init, init_tr;

    std::shared_ptr<FrankaModelUpdater> init_left_arm, left_arm;
    std::shared_ptr<FrankaModelUpdater> right_arm, init_right_arm;
    Eigen::Matrix<double, 4, 4> init_;
};



class KinematicChainConstraint : public Constraint_new
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    KinematicChainConstraint(unsigned int links, Eigen::VectorXd start) : Constraint_new(links, 2)
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
        
        out[0] = d;
        out[1] = r;
    }

    /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        double h = 1e-4;
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
            double g1_d = d;
            double g1_r = r;

            q_temp(i) = x(i) - h;
            Eigen::Affine3d lt_2 = left_base * left_arm->getTransform(q_temp.segment(0, 7));
            Eigen::Affine3d rt_2 = right_base * right_arm->getTransform(q_temp.segment(7, 7));
            Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
            Eigen::Quaterniond cur2_q(result_2.linear());
            r = cur2_q.angularDistance(ori_q);
            d = (result_2.translation() - init.translation()).norm();
            // d_rot = ( init_tr * result_2.linear() ).log().norm();
            
            double g2_d = d;
            double g2_r = r;
            out(0, i) = (g1_d - g2_d) / (2 * h);
            out(1, i) = (g1_r - g2_r) / (2 * h);
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