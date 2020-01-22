/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Bryant Gipson, Mark Moll */

#ifndef OMPL_DEMO_KINEMATIC_CHAIN_
#define OMPL_DEMO_KINEMATIC_CHAIN_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>
#include <fstream>

// simply use a random projection
class KinematicChainProjector : public ompl::base::ProjectionEvaluator
{
public:
    KinematicChainProjector(const ompl::base::StateSpace *space) : ompl::base::ProjectionEvaluator(space)
    {
        int dimension = std::max(2, (int)ceil(log((double)space->getDimension()))); //ceil : 올림함수, dimension = 3
        projectionMatrix_.computeRandom(space->getDimension(), dimension);
    }
    unsigned int getDimension() const override
    {
        return projectionMatrix_.mat.rows();
    }
    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        std::vector<double> v(space_->getDimension());
        space_->copyToReals(v, state);
        projectionMatrix_.project(&v[0], projection);
    }

protected:
    /* A projection matrix -- it allows multiplication of
            real vectors by a specified matrix. The matrix can also be
            randomly generated. */
    ompl::base::ProjectionMatrix projectionMatrix_;
};

class KinematicChainSpace : public ompl::base::RealVectorStateSpace
{
public:
    KinematicChainSpace(unsigned int numLinks)
      : ompl::base::RealVectorStateSpace(numLinks)
    {
        ompl::base::RealVectorBounds bounds(numLinks);
        
        for (int i = 0; i < 2; i++)
        {
            bounds.setLow(0 + i * 7, -2.8973);
            bounds.setHigh(0 + i * 7, 2.8973);

            bounds.setLow(1 + i * 7, -1.7628);
            bounds.setHigh(1 + i * 7, 1.7628);

            bounds.setLow(2 + i * 7, -2.8973);
            bounds.setHigh(2 + i * 7, 2.8973);

            bounds.setLow(3 + i * 7, -3.0718);
            bounds.setHigh(3 + i * 7, -0.0698);

            bounds.setLow(4 + i * 7, -2.8973);
            bounds.setHigh(4 + i * 7, 2.8973);

            bounds.setLow(5 + i * 7, -0.0175);
            bounds.setHigh(5 + i * 7, 3.7525);

            bounds.setLow(6 + i * 7, -2.8973);
            bounds.setHigh(6 + i * 7, 2.8973);
        }
        setBounds(bounds);
        std::cout << "  - min: ";
        for (unsigned int i = 0; i < dimension_; ++i)
            std::cout << bounds.low[i] << " ";
        std::cout << std::endl;
        std::cout << "  - max: ";
        for (unsigned int i = 0; i < dimension_; ++i)
            std::cout << bounds.high[i] << "  ";
        std::cout << std::endl;
        
        type_ = ompl::base::STATE_SPACE_SO2;
    }

    // void registerProjections() override
    // {
    //     registerDefaultProjection(std::make_shared<KinematicChainProjector>(this));
    // }


    void enforceBounds(ompl::base::State *state) const override
    {
        auto *statet = state->as<StateType>();
        for (int i = 0; i < 2; i++)
        {
            double v0 = fmod(statet->values[0 + i*7], 2.0 * 2.8973);
            double v1 = fmod(statet->values[1 + i*7], 2.0 * 1.7628);
            double v2 = fmod(statet->values[2 + i*7], 2.0 * 2.8973);
            double v3 = fmod(statet->values[3 + i*7], -0.0698 -3.0718);
            double v4 = fmod(statet->values[4 + i*7], 2.0 * 2.8973);
            double v5 = fmod(statet->values[5 + i*7], 3.7525-0.0175);
            double v6 = fmod(statet->values[6 + i*7], 2.0 * 2.8973);

            if (v0 < -2.8973)
                v0 += 2.0 * 2.8973;
            else if (v0 > 2.8973)
                v0 -= 2.0 * 2.8973;
            statet->values[0 + i*7] = v0;
            
            if (v1 < -1.7628)
                v1 += 2.0 * 1.7628;
            else if (v1 > 1.7628)
                v1 -= 2.0 * 1.7628;    
            statet->values[1 + i*7] = v1;

            if (v2 < -2.8973)
                v2 += 2.0 * 2.8973;
            else if (v2 > 2.8973)
                v2 -= 2.0 * 2.8973;
            statet->values[2 + i*7] = v2;

            if (v3 < -3.0718)
                v3 += -0.0698 -3.0718;
            if (v3 > -0.0698)
                v3 -= -0.0698 -3.0718;
            statet->values[3 + i*7] = v3;

            if (v4 < -2.8973)
                v4 += 2.0 * 2.8973;
            else if (v4 > -2.8973)
                v4 -= 2.0 * 2.8973;    
            statet->values[4 + i*7] = v4;

            if (v5 < -0.0175)
                v5 += 3.7525-0.0175;
            else if (v5 > 3.7525)
                v5 -= 3.7525-0.0175;
            statet->values[5 + i*7] = v5;

            if (v6 < -2.8973)
                v6 += 2.0 * 2.8973;
            if (v6 > -2.8973)
                v6 -= 2.0 * 2.8973;
            statet->values[6 + i*7] = v6;
        }
    }

    // double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override
    // {
    //     const auto *cstate1 = state1->as<StateType>();
    //     const auto *cstate2 = state2->as<StateType>();
    // }
    bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override
    {
        // bool flag = true;
        // const auto *cstate1 = state1->as<StateType>();
        // const auto *cstate2 = state2->as<StateType>();

        // for (unsigned int i = 0; i < dimension_ && flag; ++i)
        //     flag &= fabs(cstate1->values[i] - cstate2->values[i]) < 1e-4;

        // return flag;

        const double *s1 = static_cast<const StateType *>(state1)->values;
        const double *s2 = static_cast<const StateType *>(state2)->values;
        for (unsigned int i = 0; i < dimension_; ++i)
        {
            double diff = (*s1++) - (*s2++);
            if (fabs(diff) > 1e-20)
                return false;
        }
        return true;
    }

    /* 	Computes the state that lies at time t in [0, 1] on the segment that connects from state to to state. 
    The memory location of state is not required to be different from the memory of either from or to. */

    // void interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
    //                  ompl::base::State *state) const override
    // {
    //     const auto *fromt = from->as<StateType>();
    //     const auto *tot = to->as<StateType>();
    //     auto *statet = state->as<StateType>();

    //     for (unsigned int i = 0; i < dimension_; ++i)
    //     {
    //         double diff = tot->values[i] - fromt->values[i];
    //         if (fabs(diff) <= boost::math::constants::pi<double>())
    //             statet->values[i] = fromt->values[i] + diff * t;
    //         else
    //         {
    //             if (diff > 0.0)
    //                 diff = 2.0 * boost::math::constants::pi<double>() - diff;
    //             else
    //                 diff = -2.0 * boost::math::constants::pi<double>() - diff;

    //             statet->values[i] = fromt->values[i] - diff * t;
    //             if (statet->values[i] > boost::math::constants::pi<double>())
    //                 statet->values[i] -= 2.0 * boost::math::constants::pi<double>();
    //             else if (statet->values[i] < -boost::math::constants::pi<double>())
    //                 statet->values[i] += 2.0 * boost::math::constants::pi<double>();
    //         }
    //     }
    // }

protected:
   
};

class KinematicChainValidityChecker : public ompl::base::StateValidityChecker // to find valid state space configurations
{
public:
    KinematicChainValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    // bool isValid(const ompl::base::State *state) const override
    // {
    // }

protected:
    
};

#endif
