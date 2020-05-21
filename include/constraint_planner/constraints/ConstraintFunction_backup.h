
#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <memory>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <algorithm>

#include <ompl/base/Constraint.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <constraint_planner/kinematics/KinematicChain.h>
#include <constraint_planner/kinematics/panda_model_updater.h>

#include <ctime>

#include <boost/algorithm/string.hpp>
#include <unsupported/Eigen/MatrixFunctions>

using namespace std;
class KinematicChainConstraint : public ompl::base::Constraint
{
public:
    /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
    // KinematicChainConstraint2(unsigned int links, Eigen::VectorXd start) : Constraint_new(links, 6)
    KinematicChainConstraint(unsigned int links, Eigen::VectorXd start) : ompl::base::Constraint(14, 6)
    {
        for (int i = 0; i < 7; i++)
        {
            // left_qinit[i] = start[i];
            right_qinit[i] = start[i];
            top_qinit[i] = start[i + 7];
        }

        grasping_point grp;
        base_serve = grp.base_serve;
        base_main = grp.base_main;
        base_top = grp.base_top;
        panda_model = std::make_shared<FrankaModelUpdater>();

        // left_init = base_serve * panda_model->getTransform(left_qinit);
        right_init = base_main * panda_model->getTransform(right_qinit);
        top_init = base_top * panda_model->getTransform(top_qinit);

        init = top_init.inverse() * right_init;
        init_tr = init.linear().transpose();

        lower_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        upper_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

        theta = 0.99995;
        beta1 = 0.1;
        beta2 = 0.2; // 0.25
        trust_radius_min = 5 * 10e-4;
        maxIterations = 100; //default : 50
    }

    Eigen::DiagonalMatrix<double, 14> scaling_matrix(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        Eigen::Matrix<double, 14, 1> new_j = new_jacobian(x);
        Eigen::Matrix<double, 14, 1> v;
        for (int i = 0; i < 14; i++)
        {
            if (new_j[i] < 0)
                v[i] = upper_limit[i] - x[i];
            else if (new_j[i] > 0)
                v[i] = x[i] - lower_limit[i];
            else if (new_j[i] == 0)
                v[i] = min(x[i] - lower_limit[i], upper_limit[i] - x[i]);
        }

        return v.asDiagonal();
    }

    double new_function(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        Eigen::VectorXd f(getCoDimension());
        function(x, f);
        return 0.5 * f.squaredNorm();
    }

    Eigen::Matrix<double, 14, 1> new_jacobian(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        Eigen::VectorXd f(getCoDimension());
        Eigen::MatrixXd j(getCoDimension(), n_);
        function(x, f);
        jacobian(x, j);
        return j.transpose() * f;
    }

    Eigen::Matrix<double, 14, 1> projection_onto_bound(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        Eigen::Matrix<double, 14, 1> result;
        for (int i = 0; i < 14 ; i++)
            result[i] = max(lower_limit[i], min(x[i], upper_limit[i]));
        return result;
    }

    double model_function(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::Ref<const Eigen::VectorXd> &p) const
    {
        Eigen::VectorXd f(getCoDimension());
        Eigen::MatrixXd j(getCoDimension(), n_);

        function(x, f);
        jacobian(x, j);
        // auto result = 0.5 * f.transpose() * f + f.transpose() * j * p + 0.5 * p.transpose() * j.transpose() * j * p;
        return (f + j * p).norm();
    }

    Eigen::Matrix<double, 14, 1> alpha_p(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd p) const
    {
        Eigen::Matrix<double, 14, 1> result, t;
        for (int i = 0; i < 14; i++)
        {
            if (p[i] == 0)
                t[i] = std::numeric_limits<double>::infinity();
            else
                t[i] = max((lower_limit[i] - x[i]) / p[i], (upper_limit[i] - x[i]) / p[i]);
        }
        double lambda = t.minCoeff();
        
        result = max(theta , 1-p.norm())* lambda * p;

        return result;
    }

    double cauchy_ratio(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Matrix<double, 14, 1> p_cauchy, Eigen::Matrix<double, 14, 1> p_current) const
    {
        double numerator = model_function(x, Eigen::VectorXd::Zero(14)) - model_function(x, alpha_p(x, p_current));
        double denominator = model_function(x, Eigen::VectorXd::Zero(14)) - model_function(x, alpha_p(x, p_cauchy));
        return numerator / denominator;
    }

    bool within_bounds(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        for (int i = 0; i < 14; i++)
            if (x[i] < lower_limit[i] || x[i] > upper_limit[i])
                return false;

        return true;
    }

    double LAMBDA(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd p_cauchy, Eigen::VectorXd p_newton_bar) const
    {
        Eigen::VectorXd result(14);
        for (int i = 0; i < 14; i++)
            result[i] = max( (lower_limit[i] - (x[i] + p_cauchy[i]) ) / - (p_newton_bar - p_cauchy)[i]  , 
                               (upper_limit[i] - (x[i] + p_cauchy[i]) ) / - (p_newton_bar - p_cauchy)[i] );
        return result.minCoeff();
    }


    bool project(Eigen::Ref<Eigen::VectorXd> x) const override
    {
        unsigned int iter = 0;
        int flag = 0;
        double norm1 = 0.;
        double norm2 = 0.;

        const double squaredTolerance1 = tolerance1_ * tolerance1_;
        const double squaredTolerance2 = tolerance2_ * tolerance2_;

        Eigen::VectorXd f(getCoDimension());
        Eigen::MatrixXd j(getCoDimension(), n_);

        Eigen::MatrixXd new_j(14, 1);
        Eigen::MatrixXd D(14, 14), D_inv(14, 14), G(14, 14);
        Eigen::VectorXd p_newton(14), p_cauchy(14), p(14), alpha(14), p_newton_bar(14);
        Eigen::VectorXd scaled_gradient(14);

        double cauchy_length, trust_radius, ratio;
        G.setIdentity();
    
        function(x, f);
        double new_f = new_function(x);
        new_j = new_jacobian(x);
        D = scaling_matrix(x);
        scaled_gradient = -D * new_j;
        trust_radius = (D.inverse() * new_j).norm(); // initial trust radius
        double trust_radius_temp;
        while (((norm1 = f.segment(0, 3).squaredNorm()) > squaredTolerance1 || (norm2 = f.segment(3, 3).squaredNorm()) > squaredTolerance2) && iter++ < maxIterations)
        {
            OMPL_INFORM("ITER : %d  / ftn : %f  /  norm 1 : %f  /  norm 2 : %f", iter, new_f, norm1, norm2);
            cout << "x          : " << x.transpose() << endl;
            /* STEP 1 : Compute J, F, D*/
            jacobian(x, j);
            new_f = new_function(x);
            new_j = new_jacobian(x);
            D = scaling_matrix(x);
            D_inv = D.inverse();

            /* STEP 3 */
            p_newton = -j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            std::cout << "p newton : " << p_newton.transpose() << std::endl;
            Eigen::VectorXd temp(14);
            temp = projection_onto_bound(x + p_newton) - x;
            p_newton_bar = max(theta, 1-temp.norm() ) * temp;
            /* STEP 4 & 5 */
            do
            {
                double tau_bar = min(- (f.transpose() * j * scaled_gradient)[0] / (j * scaled_gradient).squaredNorm() , trust_radius / (scaled_gradient).norm() );
                if (within_bounds(x + tau_bar * scaled_gradient))
                    p_cauchy = tau_bar * scaled_gradient;
                else
                    p_cauchy = alpha_p(x, scaled_gradient);
                
                /*dogleg method*/
                auto gamma_hat = - ( (f + j*p_cauchy).transpose()*j*(p_newton_bar - p_cauchy) ) / 
                                        (j * (p_newton_bar - p_cauchy)).squaredNorm() ;
                double gamma;
                if (gamma_hat[0] > 0)
                {
                    double gamma_plus = (p_cauchy.transpose() * G * G * (p_cauchy - p_newton_bar) + 
                                        sqrt( (p_cauchy.transpose() * G * G * (p_cauchy-p_newton_bar)).squaredNorm() - 
                                        (G * (p_cauchy - p_newton_bar)).squaredNorm() *  ((G*p_cauchy).squaredNorm() - pow(trust_radius, 2)) ) ) / 
                                        (G*(p_cauchy-p_newton_bar)).squaredNorm();
                    double gamma_bar_plus = LAMBDA(x, p_cauchy, p_newton_bar);
                    gamma = min (  min(gamma_hat[0], gamma_plus) , theta * gamma_bar_plus);
                }
                else 
                {
                    double gamma_minus = (p_cauchy.transpose() * G * G * (p_cauchy - p_newton_bar) - 
                                        sqrt( (p_cauchy.transpose() * G * G * (p_cauchy-p_newton_bar)).squaredNorm() - 
                                        (G * (p_cauchy - p_newton_bar)).squaredNorm() *  ((G*p_cauchy).squaredNorm() - pow(trust_radius, 2)) ) ) / 
                                        (G*(p_cauchy-p_newton_bar)).squaredNorm();
                    double gamma_bar_minus = -LAMBDA(x, p_cauchy, p_newton_bar);
                    gamma = min (  min(gamma_hat[0], gamma_minus) , theta * gamma_bar_minus);
                }

                p = (1 - gamma) * p_cauchy + gamma * p_newton_bar;
                
                ratio = ( f.norm() - (f + j * p).norm() ) / (f.norm() - (f + j * p_cauchy).norm() );
                if (ratio < beta2){
                    trust_radius_temp = trust_radius;
                    trust_radius = (0.25 * trust_radius, 0.5 * (G * p).norm() );
                }

            } while (ratio < beta2);

            /* STEP 6 */
            x = x + p;
            if (ratio >= 0.75)
            {
                std::cout << "ratio is bigget than 0.75 " << std::endl;
                trust_radius = max(max(trust_radius_min, trust_radius_temp), 2 * (D * p).norm()); //max ( trust_radius, 2 * (D * alpha).norm() ); // max (max(trust_radius_min, trust_radius), 2 * (D * alpha).norm() );
            }
            else
                trust_radius = max(trust_radius_min, trust_radius_temp);
            function(x, f);
            OMPL_INFORM("                                norm 1 : %f  /  norm 2 : %f", f.segment(0, 3).squaredNorm(), f.segment(3, 3).squaredNorm());
        }

        if ((norm1 < squaredTolerance1) && (norm2 < squaredTolerance2))
        {
            OMPL_INFORM("*********** PROJECTION SUCCESS");
            // cout << x.transpose() << endl;
            // cout << endl;
            // OMPL_INFORM("ITER : %d  --> PROJECTION SUCCESS", iter);
            // cout << endl;

            return 1;
        }
        else
        {
            OMPL_WARN("*********** PROJECTION FAIL");
            return 0;
        }
    }


    double quad_solve(Eigen::Matrix<double, 14, 14> D, Eigen::Matrix<double, 14, 1> p_cauchy, Eigen::Matrix<double, 14, 1> p_newton, double trust_radius) const
    {
        Eigen::Matrix<double, 14, 1> p_diff = p_newton - p_cauchy;
        // a = (p_diff).transpose() * D.transpose() * D * p_diff;
        double a = (D * p_diff).squaredNorm();
        auto b = 2 * p_diff.transpose() * D * D * (2 * p_cauchy - p_newton);
        double c = (D * (2 * p_cauchy - p_newton)).squaredNorm() - pow(trust_radius, 2);
        // b = -2 * p_diff.transpose() * D.transpose() * D * p_diff + p_cauchy.transpose() * D.transpose() * D * p_diff + (p_diff).transpose() * D.transpose() * D * p_cauchy;
        // c = (2*p_cauchy.transpose() - p_newton.transpose()) * D.transpose() * D * (2 * p_cauchy - p_newton);
        double det = b[0] * b[0] - 4 * a * c;
        if (det >= 0)
            return (-b[0] + sqrt(det)) / (2 * a);
        else
        {
            // cout << "cannot calculate" << endl;
            return 0;
        }
    }

    /*actual constraint function, state "x" from the ambient space */
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        Eigen::VectorXd &&temp = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);

        // Eigen::Affine3d lt = base_serve * panda_model->getTransform(temp);
        Eigen::Affine3d tt = base_top * panda_model->getTransform(temp2);
        Eigen::Affine3d rt = base_main * panda_model->getTransform(temp);

        Eigen::Affine3d result = tt.inverse() * rt;

        Eigen::VectorXd p = (result.translation() - init.translation());

        Eigen::Matrix3d r_diff = init.linear().transpose() * result.linear();
        Eigen::AngleAxisd r_angleaxis(r_diff);
        Eigen::Vector3d r = r_angleaxis.axis() * r_angleaxis.angle();

        r = tt.linear().inverse() * right_init.linear() * r;

        out[0] = p[0];
        out[1] = p[1];
        out[2] = p[2];
        out[3] = r[0];
        out[4] = r[1];
        out[5] = r[2];
    }

    /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    {
        Eigen::VectorXd &&q_temp = x;
        // Eigen::Affine3d lt = base_serve * panda_model->getTransform(q_temp.segment(0, 7));
        Eigen::Affine3d tt = base_top * panda_model->getTransform(q_temp.segment(7, 7));
        Eigen::Affine3d rt = base_main * panda_model->getTransform(q_temp.segment(0, 7));
        Eigen::Affine3d result = tt.inverse() * rt;
        // Eigen::Affine3d result = tt.inverse() * rt;

        // Eigen::MatrixXd jaco_l = panda_model->getJacobian(q_temp.segment(0, 7));
        Eigen::MatrixXd jaco_t = panda_model->getJacobian(q_temp.segment(7, 7));
        Eigen::MatrixXd jaco_r = panda_model->getJacobian(q_temp.segment(0, 7));

        Eigen::MatrixXd top_omega = Eigen::MatrixXd::Zero(6, 6);
        Eigen::MatrixXd right_omega = Eigen::MatrixXd::Zero(6, 6);
        top_omega.block<3, 3>(0, 0) = tt.linear().inverse();
        top_omega.block<3, 3>(3, 3) = tt.linear().inverse();
        right_omega.block<3, 3>(0, 0) = result.linear() * rt.linear().inverse();
        right_omega.block<3, 3>(3, 3) = result.linear() * rt.linear().inverse();

        Eigen::MatrixXd wrench = Eigen::MatrixXd::Zero(6, 6);
        wrench.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        wrench.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
        wrench.block<3, 3>(0, 3) = -skew_symmetric(result.translation());

        Eigen::MatrixXd jaco(6, 14);
        jaco.block<6, 7>(0, 0) = -wrench * top_omega * jaco_t;
        jaco.block<6, 7>(0, 7) = right_omega * jaco_r;
        out = jaco;
    }

    void setTolerance(const double tolerance1, const double tolerance2)
    {
        if (tolerance1 <= 0 || tolerance2 <= 0)
            throw ompl::Exception("ompl::base::Constraint::setProjectionTolerance(): "
                                  "tolerance must be positive.");
        tolerance1_ = tolerance1;
        tolerance2_ = tolerance2;
        OMPL_INFORM("Set tolerance to %f and %f", tolerance1_, tolerance2_);
    }

    bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
    {
        Eigen::VectorXd f(getCoDimension());
        function(x, f);
        return f.allFinite() && f.segment(0, 3).squaredNorm() <= tolerance1_ * tolerance1_ && f.segment(3, 3).squaredNorm() <= tolerance2_ * tolerance2_;
    }

    Eigen::Matrix3d skew_symmetric(Eigen::Vector3d x) const
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

protected:
    double tolerance1_, tolerance2_;

private:
    Eigen::Matrix<double, 7, 1> left_q, right_q, top_q;
    Eigen::Matrix<double, 7, 1> left_qinit, right_qinit, top_qinit;
    Eigen::Affine3d left_init, top_init, right_init, init, init_tr;

    std::shared_ptr<FrankaModelUpdater> panda_model;
    Eigen::Matrix<double, 4, 4> init_;

    Eigen::Matrix<double, 14, 1> upper_limit, lower_limit;
    double theta, beta1, beta2, trust_radius_min;
    int maxIterations;

    Eigen::Affine3d base_serve, base_main, base_top;
};

typedef std::shared_ptr<KinematicChainConstraint> ChainConstraintPtr;

// class KinematicChainConstraint : public Constraint_new
// {
// public:
//     /* ob::Constraint(a, b) : a is dimension of the ambient space, b is constraint ouputs*/
//     KinematicChainConstraint(unsigned int links, Eigen::VectorXd start) : Constraint_new(links, 2)
//     {
//         for (int i = 0; i < 7; i++)
//         {
//             left_qinit[i] = start[i];
//             right_qinit[i] = start[i + 7];
//         }
//         base_serve.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);
//         base_main.translation() = Eigen::Vector3d(0.0, -0.2, 0.0);

//         panda_model = std::make_shared<FrankaModelUpdater>(left_q);
//         panda_model = std::make_shared<FrankaModelUpdater>(right_q);

//         left_init = base_serve * panda_model->getTransform(left_qinit);
//         right_init = base_main * panda_model->getTransform(right_qinit);
//         init = left_init.inverse() * right_init;
//         init_tr = init.linear().transpose();
//     }

//     /*actual constraint function, state "x" from the ambient space */
//     void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
//     {

//         Eigen::VectorXd &&temp = x.segment(0, 7);
//         Eigen::VectorXd &&temp2 = x.segment(7, 7);

//         Eigen::Affine3d lt = base_serve * panda_model->getTransform(temp);
//         Eigen::Affine3d rt = base_main * panda_model->getTransform(temp2);

//         Eigen::Affine3d result = lt.inverse() * rt;

//         double r;
//         Eigen::Quaterniond cur_q(result.linear());
//         Eigen::Quaterniond ori_q(init.linear());
//         r = cur_q.angularDistance(ori_q);

//         double d;
//         d = (result.translation() - init.translation()).norm();

//         // double d_rot;
//         // d_rot = (  init_tr * result.linear() ).log().norm();

//         out[0] = d;
//         out[1] = r;
//     }

//     /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
//     void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
//     {
//         double h = 1e-4;
//         double d, d_rot;
//         double r;
//         Eigen::Quaterniond ori_q(init.linear());
//         for (int i = 0; i < 14; i++)
//         {
//             Eigen::VectorXd &&q_temp = x;
//             q_temp(i) = x(i) + h;
//             Eigen::Affine3d lt_1 = base_serve * panda_model->getTransform(q_temp.segment(0, 7));
//             Eigen::Affine3d rt_1 = base_main * panda_model->getTransform(q_temp.segment(7, 7));
//             Eigen::Affine3d result_1 = lt_1.inverse() * rt_1;

//             // d_rot = ( init_tr * result_1.linear() ).log().norm();

//             Eigen::Quaterniond cur1_q(result_1.linear());
//             r = cur1_q.angularDistance(ori_q);
//             d = (result_1.translation() - init.translation()).norm();
//             double g1_d = d;
//             double g1_r = r;

//             q_temp(i) = x(i) - h;
//             Eigen::Affine3d lt_2 = base_serve * panda_model->getTransform(q_temp.segment(0, 7));
//             Eigen::Affine3d rt_2 = base_main * panda_model->getTransform(q_temp.segment(7, 7));
//             Eigen::Affine3d result_2 = lt_2.inverse() * rt_2;
//             Eigen::Quaterniond cur2_q(result_2.linear());
//             r = cur2_q.angularDistance(ori_q);
//             d = (result_2.translation() - init.translation()).norm();
//             // d_rot = ( init_tr * result_2.linear() ).log().norm();

//             double g2_d = d;
//             double g2_r = r;
//             out(0, i) = (g1_d - g2_d) / (2 * h);
//             out(1, i) = (g1_r - g2_r) / (2 * h);
//         }
//     }

// private:
//     Eigen::Matrix<double, 7, 1> left_q, right_q;
//     Eigen::Matrix<double, 7, 1> left_qinit, right_qinit;
//     Eigen::Affine3d left_init, right_init, init, init_tr;

//     std::shared_ptr<FrankaModelUpdater> init_panda_model, panda_model;
//     std::shared_ptr<FrankaModelUpdater> panda_model, init_panda_model;
//     Eigen::Matrix<double, 4, 4> init_;
// };