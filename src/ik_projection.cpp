#include <iostream>

#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <Eigen/Core>

#include <constraint_planner/panda_model_updater.h>

Eigen::Affine3d left_base = Eigen::Affine3d::Identity();
Eigen::Affine3d right_base = Eigen::Affine3d::Identity();

class IKConstraint
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    IKConstraint(unsigned int links)
    {
        left_base.translation() = Eigen::Vector3d(0.0, 0.3, 0.0);
        right_base.translation() = Eigen::Vector3d(0.0, -0.3, 0.0);

        left_arm = std::make_shared<FrankaModelUpdater>(left_q);
        right_arm = std::make_shared<FrankaModelUpdater>(right_q);
        joint_low << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        joint_high << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
    }

    /*actual constraint function, state "x" from the ambient space */
    void function(Eigen::VectorXd &x, Eigen::Ref<Eigen::VectorXd> out)
    {

        // Eigen::VectorXd &&temp = x;

        Eigen::Affine3d lt = left_base * left_arm->getTransform(x);
        // Eigen::Affine3d rt = right_base * right_arm->getTransform(temp2);

        Eigen::VectorXd p = lt.translation();
        Eigen::AngleAxisd r_angleaxis(lt.linear());
        Eigen::Vector3d r = r_angleaxis.axis() * r_angleaxis.angle();

        out[0] = p[0];
        out[1] = p[1];
        out[2] = p[2];
        out[3] = r[0];
        out[4] = r[1];
        out[5] = r[2];
    }

    void jacobian(Eigen::VectorXd &x, Eigen::Ref<Eigen::MatrixXd> out)
    {
        // Eigen::VectorXd &&q_temp = x;
        Eigen::MatrixXd jaco = left_arm->getJacobian(x);
        // Eigen::MatrixXd jaco_r = right_arm->getJacobian(q_temp.segment(7, 7));

        out = jaco;
    }

    void enforceBounds(Eigen::VectorXd &x)
    {
        for (unsigned int i = 0; i < 7; ++i)
        {
            if (x[i] > joint_high[i])
            {
                x[i] = joint_high[i];
            }
            else if (x[i] < joint_low[i])
            {
                x[i] = joint_low[i];
            }
        }
    }

private:
    Eigen::Matrix<double, 7, 1> left_q, right_q;
    Eigen::Matrix<double, 7, 1> left_qinit, right_qinit;
    Eigen::Affine3d left_init, right_init, init, init_tr;

    std::shared_ptr<FrankaModelUpdater> init_left_arm, left_arm;
    std::shared_ptr<FrankaModelUpdater> right_arm, init_right_arm;
    Eigen::Matrix<double, 4, 4> init_;
    Eigen::Matrix<double, 7, 1> joint_low, joint_high;
};

int main()
{
    // Newton's method
    unsigned int iter = 0;
    double norm = 0;
    double tolerance_ = 0.01;
    unsigned int maxIterations_ = 10000;
    Eigen::VectorXd f(6);
    Eigen::MatrixXd j(6, 7);
    Eigen::VectorXd x(7);
    x << 1.14669, -1.38017, -1.45742, -3.02049, -2.82783, 1.09462, 2.5943;
    const double squaredTolerance = tolerance_ * tolerance_;
    
    IKConstraint ikconstraint(7);
    ikconstraint.function(x, f);
    while ((norm = f.squaredNorm()) > squaredTolerance && iter++ < maxIterations_)
    {
        std::cout << iter << std::endl;
        ikconstraint.jacobian(x, j);
        x -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
        ikconstraint.enforceBounds(x);
        ikconstraint.function(x, f);
    }

    if (norm < squaredTolerance)
        std::cout << x.transpose() << std::endl;
}