
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
    KinematicChainConstraint(unsigned int links) : ompl::base::Constraint(14, 6)
    {

        grasping_point grp;
         for (int i = 0; i < 7; i++)
        {
            qinit_1st[i] = grp.start[i];
            qinit_3rd[i] = grp.start[i + 7];
        }

        base_serve = grp.base_serve;
        base_main = grp.base_main;

        offset_R = Eigen::Affine3d::Identity();
        offset_R.linear() = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitX()).toRotationMatrix();

        panda_arm = std::make_shared<FrankaModelUpdater>();
        init_1st = base_serve * panda_arm->getTransform(qinit_1st) * offset_R;
        init_3rd = base_main * panda_arm->getTransform(qinit_3rd);

        init = init_1st.inverse() * init_3rd;
        // std::cout << "init_1st " << init_1st.linear() << std::endl;
        // std::cout << "init_3rd " << init_3rd.linear() << std::endl;
        // std::cout << "relative position : " << init.translation().transpose() << std::endl;
        // std::cout << "relative position : " << init.linear().eulerAngles(0, 1, 2).transpose() << std::endl;
        lower_limit << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;
        upper_limit << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;

        theta = 0.99995;
        beta1 = 0.1;
        beta2 = 0.2; // 0.25 
        trust_radius_min = 5*10e-4;
        maxIterations = 100; //default : 50

    }

    Eigen::DiagonalMatrix<double, 14> scaling_matrix(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        Eigen::Matrix<double, 14, 1> new_j = new_jacobian(x);
        Eigen::Matrix<double, 14, 1> v;
        for (int i = 0; i < 14; i++)
        {
            if (new_j[i] < 0)
                v[i] = x[i] - upper_limit[i];
            else
                v[i] = x[i] - lower_limit[i];
        }
        for (int i = 0; i < 14; i++)
            v[i] = pow(abs(v[i]), -0.5);
        return v.asDiagonal();
    }

    Eigen::DiagonalMatrix<double, 14> scaling_matrix_inv(const Eigen::Ref<const Eigen::VectorXd> &x) const
    {
        Eigen::Matrix<double, 14, 1> new_j = new_jacobian(x);
        Eigen::Matrix<double, 14, 1> v;
        for (int i = 0; i < 14; i++)
        {
            if (new_j[i] < 0)
                v[i] = x[i] - upper_limit[i];
            else
                v[i] = x[i] - lower_limit[i];
        }
        for (int i = 0; i < 14; i++)
            v[i] = pow(abs(v[i]), 0.5);
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

    double model_function(const Eigen::Ref<const Eigen::VectorXd> &x, const Eigen::Ref<const Eigen::VectorXd> &p) const
    {
        Eigen::VectorXd f(getCoDimension());
        Eigen::MatrixXd j(getCoDimension(), n_);

        function(x, f);
        jacobian(x, j);
        auto result = 0.5 * f.transpose() * f + f.transpose() * j * p + 0.5 * p.transpose() * j.transpose() * j * p;
        return result[0];
    }

    Eigen::Matrix<double, 14, 1> alpha_p(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::VectorXd p) const
    {
        Eigen::Matrix<double, 14, 1> result, t;
        for (int i = 0; i < 14; i++)
        {
            if ( p[i] > 0 )
                t[i] = (upper_limit[i] - x[i]) / p[i];
            else
                t[i] = (lower_limit[i] - x[i]) / p[i];
        }

        // for (int i = 0; i < 14; i++)
        // {
        //     if (p[i] == 0)
        //         t[i] = std::numeric_limits<double>::infinity();
        //     else
        //         t[i] = max((lower_limit[i] - x[i]) / p[i], (upper_limit[i] - x[i]) / p[i]);
        // }
        double lambda = t.minCoeff();
        if (lambda > 1)
            result = p;
        else
            result = max(theta, 1 - p.norm()) * lambda * p;

        return result;
    }

    double cauchy_ratio(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Matrix<double, 14, 1> p_cauchy, Eigen::Matrix<double, 14, 1> p_current) const
    {
        double numerator  = model_function(x, Eigen::VectorXd::Zero(14)) - model_function(x, alpha_p(x, p_current));
        double denominator = model_function(x, Eigen::VectorXd::Zero(14)) - model_function(x, alpha_p(x, p_cauchy));
        return numerator / denominator;
    }

    // bool project(Eigen::Ref<Eigen::VectorXd> x) const override
    // {
    //     unsigned int iter = 0;
    //     int flag = 0;
    //     double norm1 = 0.;
    //     double norm2 = 0.;

    //     const double squaredTolerance1 = tolerance1_ * tolerance1_;
    //     const double squaredTolerance2 = tolerance2_ * tolerance2_;

    //     Eigen::VectorXd f(getCoDimension());
    //     Eigen::MatrixXd j(getCoDimension(), n_);

    //     Eigen::MatrixXd new_j(14, 1);
    //     Eigen::MatrixXd D(14, 14), D_inv(14, 14);
    //     Eigen::VectorXd p_newton(14), p_cauchy(14), p(14), alpha(14);

    //     double cauchy_length, trust_radius, ratio;
    //     double model_ftn;

    //     function(x, f);
    //     double new_f = new_function(x);
    //     new_j = new_jacobian(x);
    //     D = scaling_matrix(x);
    //     D_inv = scaling_matrix_inv(x);

    //     trust_radius = (D_inv * new_j).norm(); // initial trust radius
    //     while (((norm1 = f.segment(0, 3).squaredNorm()) > squaredTolerance1 || (norm2 = f.segment(3, 3).squaredNorm()) > squaredTolerance2) && iter++ < maxIterations)
    //     {
    //         // OMPL_INFORM("ITER : %d  / ftn : %f  /  norm 1 : %f  /  norm 2 : %f", iter, new_f, norm1, norm2);
    //         /* STEP 1 : Compute J, F, D*/
    //         jacobian(x, j);
    //         new_f = new_function(x);
    //         new_j = new_jacobian(x);
    //         D = scaling_matrix(x);
    //         D_inv = scaling_matrix_inv(x);

    //         /* STEP 3 */
    //         p_newton = -j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            
    //         /* STEP 4 & 5 */
    //          do {
    //             cauchy_length = min( (D_inv * new_j).squaredNorm() / (j * D_inv * D_inv * new_j).squaredNorm() ,trust_radius / (D_inv * new_j).norm()  );
    //             // cout << "trust radius : " << trust_radius << endl;
    //             p_cauchy = -cauchy_length * D_inv * D_inv * new_j;
                
    //             if ( ( (D * p_newton).norm() <= trust_radius ) && (cauchy_ratio(x, p_cauchy, p_newton) >= beta1) )
    //             {
    //                 p = p_newton;
    //             }
    //             else /*dogleg method*/
    //             {
    //                 if (  (D * p_cauchy).norm() >= trust_radius  )
    //                 {
    //                     p = - (trust_radius * D_inv * D_inv * new_j) / (D_inv * new_j).norm();
    //                 }
    //                 else
    //                 {
    //                     double u = quad_solve(D, p_cauchy, p_newton, trust_radius);
    //                     p = p_cauchy + (u-1) * (p_newton - p_cauchy);
    //                     // if (u == 0)
    //                     // {
    //                     //     break;
    //                     // }
    //                 }

    //                 if ( !(cauchy_ratio(x, p_cauchy, p) >= beta1) )
    //                 {
    //                     p = p_cauchy;
    //                 }
    //             }
    //             // cout << "p          : " <<  p.transpose() << endl;
    //             alpha = alpha_p(x, p);
    //             ratio = (new_f - new_function(x + alpha)) / ( model_function(x, Eigen::VectorXd::Zero(14)) - model_function(x, alpha));

    //             if (ratio < beta2)
    //             {
    //                 // trust_radius = min(0.25 * trust_radius, 0.5 * (D * alpha).norm());
    //                 trust_radius = 0.25 * trust_radius;
    //             }

    //             if ( (trust_radius < 10e-8) || ( (D_inv * new_j).norm() < 10e-10 ) )
    //             {
    //                 flag = 1;
    //                 break;
    //             }
    //         } while (ratio < beta2);

    //         if (flag == 1)
    //             break;
    //         /* STEP 6 */
    //         x = x + alpha;

    //         if (ratio >= 0.75)
    //         {
    //             trust_radius = max(max(trust_radius_min, trust_radius), 2 * (D * alpha).norm()); //max ( trust_radius, 2 * (D * alpha).norm() ); // max (max(trust_radius_min, trust_radius), 2 * (D * alpha).norm() );
    //         }
    //         else
    //             trust_radius = max(trust_radius_min, trust_radius);
    //         function(x, f);
    //     }

    //     if ((norm1 < squaredTolerance1) && (norm2 < squaredTolerance2))
    //     {
    //         OMPL_INFORM("*********** PROJECTION SUCCESS");
    //         // cout << x.transpose() << endl;
    //         // cout << endl;
    //         // OMPL_INFORM("ITER : %d  --> PROJECTION SUCCESS", iter);
    //         // cout << endl;

    //         return 1;
    //     }
    //     else
    //     {
    //         // OMPL_WARN("*********** PROJECTION FAIL");
    //         return 0;
    //     }
    // }

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

    bool project(Eigen::Ref<Eigen::VectorXd> x) const override
    {
        // Newton's method
        unsigned int iter = 0;
        double norm1 = 0;
        double norm2 = 0;
        Eigen::VectorXd f(getCoDimension());
        Eigen::MatrixXd j(getCoDimension(), n_);

        const double squaredTolerance1 = tolerance1_ * tolerance1_;
        const double squaredTolerance2 = tolerance2_ * tolerance2_;
        function(x, f);
        while ( ( (norm1 = f.segment(0, 3).squaredNorm()) > squaredTolerance1  || (norm2 = f.segment(3, 3).squaredNorm()) > squaredTolerance2 )
                    && iter++ < maxIterations)
        {
            // OMPL_INFORM("ITER : %d  / norm : %f  "  , iter, f.norm());
            // cout << "x          : " << x.transpose() << endl;
            jacobian(x, j);
            x -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            function(x, f);
        }
        return (norm1 < squaredTolerance1) && (norm2 < squaredTolerance2);
    }



    /*actual constraint function, state "x" from the ambient space */
    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
    {
        Eigen::VectorXd &&temp = x.segment(0, 7);
        Eigen::VectorXd &&temp2 = x.segment(7, 7);

        // Eigen::Affine3d lt = base_serve * panda_model->getTransform(temp);
        Eigen::Affine3d lt = base_serve * panda_arm->getTransform(temp) * offset_R;
        Eigen::Affine3d rt = base_main * panda_arm->getTransform(temp2);

        Eigen::Affine3d result = lt.inverse() * rt;

        Eigen::VectorXd p = (result.translation() - init.translation());
        Eigen::Matrix3d r_diff = init.linear().transpose() * result.linear();
        Eigen::Vector3d r = r_diff.eulerAngles(0, 1, 2);

        // Eigen::Vector3d r = result.linear().eulerAngles(0, 1, 2) - init.linear().eulerAngles(0, 1, 2);

        // Eigen::AngleAxisd r_angleaxis(r_diff);
        // Eigen::Vector3d r = r_angleaxis.axis() * r_angleaxis.angle();
        // r = lt.linear().inverse() * init_3rd.linear() * r;
        // 
        out[0] = p[0];
        out[1] = p[1];
        out[2] = p[2];
        // out(3) = atan2(r_diff(2, 1), r_diff(2, 2));
        // out(4) = -asin(r_diff(2, 0));
        // out(5) = atan2(r_diff(1, 0), r_diff(0, 0));
        out[3] = r[3];
        out[4] = r[4];
        out[5] = r[5];
    }

    /* this is very computationally intensive, and providing an analytic derivative is preferred. We provide a simple scrip */
    // void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
    // {
    //     Eigen::VectorXd &&q_temp = x;
    //     Eigen::Affine3d lt = base_serve * panda_arm->getTransform(q_temp.segment(0, 7)) * offset_R;
    //     Eigen::Affine3d rt = base_main * panda_arm->getTransform(q_temp.segment(7, 7));
    //     Eigen::Affine3d result = lt.inverse() * rt;
    //     Eigen::Matrix3d r_diff = init.linear().transpose() * result.linear();

    //     Eigen::Vector3d rpy = result.linear().eulerAngles(0, 1, 2);
    //     // rpy(0) = atan2(result(2, 1), result(2, 2));
    //     // rpy(1) = -asin(result(2, 0));
    //     // rpy(2) = atan2(result(1, 0), result(0, 0));
        
    //     // E_rpy(0,0) = cos(rpy(2))/cos(rpy(1));
    //     // E_rpy(0,1) = sin(rpy(2))/cos(rpy(1));
    //     // E_rpy(0,2) = 0.0;

    //     // E_rpy(1,0) = sin(rpy(2));
    //     // E_rpy(1,1) = cos(rpy(2));
    //     // E_rpy(1,2) = 0.0;

    //     // E_rpy(2,0) = -cos(rpy(2))*sin(rpy(1))/cos(rpy(1));
    //     // E_rpy(2,1) = sin(rpy(2))*sin(rpy(1))/cos(rpy(1));
    //     // E_rpy(2,2) = 1.0;

    //     Eigen::MatrixXd offset_R2 = Eigen::MatrixXd::Zero(6, 6);
    //     offset_R2.block<3, 3>(0, 0) = offset_R.linear();
    //     offset_R2.block<3, 3>(3, 3) = offset_R.linear();
        
    //     Eigen::MatrixXd jaco_l_temp = offset_R2 * panda_arm->getJacobian(q_temp.segment(0, 7));
    //     Eigen::MatrixXd jaco_r_temp = panda_arm->getJacobian(q_temp.segment(7, 7));
    //     Eigen::Matrix<double, 6, 7> jaco_l, jaco_r;
    //     jaco_l.topRows(3) = jaco_l_temp.bottomRows(3);
    //     jaco_l.bottomRows(3) = jaco_l_temp.topRows(3);
        
    //     jaco_r.topRows(3) = jaco_r_temp.bottomRows(3);
    //     jaco_r.bottomRows(3) = jaco_r_temp.topRows(3);
        
    //     Eigen::MatrixXd omega_1st = Eigen::MatrixXd::Zero(6, 6);
    //     Eigen::MatrixXd omega_3rd = Eigen::MatrixXd::Zero(6, 6);
    //     omega_1st.block<3, 3>(0, 0) = lt.linear().inverse();
    //     omega_1st.block<3, 3>(3, 3) = lt.linear().inverse();
    //     omega_3rd.block<3, 3>(0, 0) = result.linear() * (rt.linear().inverse());
    //     omega_3rd.block<3, 3>(3, 3) = result.linear() * (rt.linear().inverse());
    //     std::cout << result.linear() << std::endl;
    //     // std::cout << "omega_1st" << std::endl;
    //     // std::cout << omega_1st << std::endl;
    //     // std::cout << "omega_3rd" << std::endl;
    //     // std::cout << omega_3rd << std::endl;

    //     Eigen::MatrixXd wrench = Eigen::MatrixXd::Zero(6, 6);
    //     wrench.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    //     wrench.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
    //     wrench.block<3, 3>(0, 3) = -skew_symmetric(result.translation());
        
    //     // std::cout << "wrench" << std::endl;
    //     // std::cout << wrench << std::endl;
    //     Eigen::MatrixXd jaco(6, 14);
    //     jaco.block<6, 7>(0, 0) = -wrench * omega_1st * jaco_l;
    //     jaco.block<6, 7>(0, 7) = omega_3rd * jaco_r;

    //     Eigen::MatrixXd e_matrix = Eigen::MatrixXd::Zero(6, 6);
    //     e_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    //     Eigen::Matrix3d e_R;
    //     e_R << 1, sin(rpy[0]) * sin(rpy[1]) / cos(rpy[1]), -cos(rpy[0])*sin(rpy[1])/cos(rpy[1]), 
    //             0, cos(rpy[0]), sin(rpy[0]),
    //             0, -sin(rpy[0]) / cos(rpy[1]) , cos(rpy[0]) / cos(rpy[1]);
    //     e_matrix.block<3, 3>(3, 3) = e_R.inverse();
        
    //     // std::cout << "e_matrix" << std::endl;
    //     // std::cout << e_matrix << std::endl;
    //     // out = e_matrix * jaco;
    //     out = jaco;
    //     // out = jaco.topRows(3);
        
    // }

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
    Eigen::Matrix<double, 7, 1> q_1st, q_3rd;
    Eigen::Matrix<double, 7, 1> qinit_1st, qinit_3rd;
    Eigen::Affine3d init_1st, init_3rd, init, init_tr;

    std::shared_ptr<FrankaModelUpdater> panda_arm;
    Eigen::Matrix<double, 4, 4> init_;

    Eigen::Matrix<double, 14, 1> upper_limit, lower_limit;

    double theta, beta1, beta2, trust_radius_min;
    int maxIterations;

    Eigen::Affine3d base_serve, base_main;
    Eigen::Affine3d offset_R;
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
//             qinit_1st[i] = start[i];
//             qinit_3rd[i] = start[i + 7];
//         }
//         base_serve.translation() = Eigen::Vector3d(0.0, 0.2, 0.0);
//         base_main.translation() = Eigen::Vector3d(0.0, -0.2, 0.0);

//         panda_model = std::make_shared<FrankaModelUpdater>(q_1st);
//         panda_model = std::make_shared<FrankaModelUpdater>(q_3rd);

//         init_1st = base_serve * panda_model->getTransform(qinit_1st);
//         init_3rd = base_main * panda_model->getTransform(qinit_3rd);
//         init = init_1st.inverse() * init_3rd;
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
//     Eigen::Matrix<double, 7, 1> q_1st, q_3rd;
//     Eigen::Matrix<double, 7, 1> qinit_1st, qinit_3rd;
//     Eigen::Affine3d init_1st, init_3rd, init, init_tr;

//     std::shared_ptr<FrankaModelUpdater> init_panda_model, panda_model;
//     std::shared_ptr<FrankaModelUpdater> panda_model, init_panda_model;
//     Eigen::Matrix<double, 4, 4> init_;
// };