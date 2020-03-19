#pragma once

#include <ompl/base/Constraint.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/util/ClassForward.h>
#include <ompl/util/Exception.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <utility>
class Constraint_new : public ompl::base::Constraint
{
public:
    Constraint_new(const unsigned int ambientDim, const unsigned int coDim) : ompl::base::Constraint(ambientDim, coDim)
    {
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
        while ( ( (norm1 = f.segment(0, 3).squaredNorm()) > squaredTolerance1 || (norm2 = f.segment(3, 3).squaredNorm()) > squaredTolerance2 )
                    && iter++ < maxIterations_)
        {
            // std::cout << iter << std::endl;
            jacobian(x, j);
            x -= j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
            function(x, f);
        }

        return (norm1 < squaredTolerance1) && (norm2 < squaredTolerance2);
    }

    void setTolerance(const double tolerance1, const double tolerance2)
    {
        if (tolerance1 <= 0 || tolerance2 <= 0)
            throw ompl::Exception("ompl::base::Constraint::setProjectionTolerance(): "
                                  "tolerance must be positive.");
        tolerance1_ = tolerance1;
        tolerance2_ = tolerance2;
    }

    bool isSatisfied(const Eigen::Ref<const Eigen::VectorXd> &x) const override
    {
        Eigen::VectorXd f(getCoDimension());
        function(x, f);
        return f.allFinite() && f.segment(0, 3).squaredNorm() <= tolerance1_ * tolerance1_ && f.segment(3, 3).squaredNorm() <= tolerance2_ * tolerance2_;
    }

protected:
    double tolerance1_, tolerance2_;
};


typedef std::shared_ptr<Constraint_new> Constraint_newPtr;