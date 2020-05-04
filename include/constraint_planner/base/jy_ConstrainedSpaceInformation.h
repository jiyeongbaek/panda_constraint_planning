#pragma once
#include <utility>

#include "ompl/base/SpaceInformation.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/base/spaces/constraint/TangentBundleStateSpace.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <constraint_planner/KinematicChain.h>
namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(jy_ConstrainedSpaceInformation); // make jy_ConostrainedSpaceInformationPtr
        class jy_ConstrainedValidStateSampler : public ValidStateSampler
        {
        public:
            jy_ConstrainedValidStateSampler(const SpaceInformation *si)
              : ValidStateSampler(si)
              , sampler_(si->getStateSpace()->allocStateSampler())
              , constraint_(si->getStateSpace()->as<ompl::base::ConstrainedStateSpace>()->getConstraint())
            {
                start_state << -1.6377232882241266, -1.315323930182948, 1.8320045929628053, -2.7664737781390967, 1.0296301925737725, 3.4689343789323694, 1.432766630340054, -0.10243983084379964, 0.2659588901612104, 0.4127700947518499, -1.3902073234890953, 0.06790555501862428, 1.5908404988928444, 2.0916124777614624;
                goal_state << 2.28836, -1.292, -0.684402, -0.825018, -2.05174, 0.744845, 2.19023, -0.238986, 0.655853, 0.0894607, -1.77037, 1.42673, 1.39508, 1.47327;
            }

            bool sample(State *state) override
            {
                // Rejection sample for at most attempts_ tries
                unsigned int tries = 0;
                bool valid;
                do
                    sampler_->sampleUniform(state);
                while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < 50);
                return valid;
            }
            
            // original ftn of ompl
            bool sampleNear(State *state, const State *near, double distance) override
            {
                // Rejection sample for at most attempts_ tries.
                unsigned int tries = 0;
                bool valid;
                do
                    sampler_->sampleUniformNear(state, near, distance);
                while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < 50);
                return valid;
            }            
            
            // jy ftn 
            // bool sampleNear(State *state, const State *near, double distance) override
            // {
            //     unsigned int tries = 0;
            //     std::cout << "test " << std::endl;
            //     bool valid;
            //     do{
            //         sampleUniformNear(state, near, distance);
            //     } while (!(valid = si_->isValid(state) && constraint_->isSatisfied(state)) && ++tries < 50);
            //     return valid;
            // }     

            void sampleUniformNear(State *state, const State *near, const double distance)
            {
                // auto *rstate = static_cast<RealVectorStateSpace::StateType *>(state);
                auto *rstate = state->as<ConstrainedStateSpace::StateType>()->getState()->as<KinematicChainSpace::StateType>();
                for (unsigned int i = 0; i < 14; ++i)
                {
                    rstate->values[i] = rng_.uniformReal(start_state[i], goal_state[i]);
                }
            }

            void setStartAndGoalStates(const Eigen::Ref<const Eigen::VectorXd> &start, const Eigen::Ref<const Eigen::VectorXd> &goal)
            {
                start_state = start;
                goal_state = goal;
            }

        private:
            StateSamplerPtr sampler_;
            int attempts;
            const ConstraintPtr constraint_;
            RNG rng_;
            Eigen::Matrix<double, 14, 1> start_state, goal_state;
        }; 

        class jy_ConstrainedSpaceInformation : public SpaceInformation
        {
        public:
            jy_ConstrainedSpaceInformation(StateSpacePtr space) : SpaceInformation(std::move(space))
            {
                stateSpace_->as<ConstrainedStateSpace>()->setSpaceInformation(this);
                setValidStateSamplerAllocator([](const SpaceInformation *si) -> std::shared_ptr<ValidStateSampler> {
                    return std::make_shared<jy_ConstrainedValidStateSampler>(si);
                });
            }

            unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                                         unsigned int count, bool endpoints, bool alloc) const override
            {
                bool success = stateSpace_->as<ConstrainedStateSpace>()->discreteGeodesic(s1, s2, true, &states);

                if (endpoints)
                {
                    if (!success && states.size() == 0)
                        states.push_back(cloneState(s1));

                    if (success)
                        states.push_back(cloneState(s2));
                }

                return states.size();
            }
        };

        class jy_TangentBundleSpaceInformation : public jy_ConstrainedSpaceInformation
        {
        public:
            /** \brief Constructor. Sets the instance of the state space to plan with. */
            jy_TangentBundleSpaceInformation(StateSpacePtr space) : jy_ConstrainedSpaceInformation(std::move(space))
            {
            }

            unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                                         unsigned int count, bool endpoints, bool alloc) const override
            {
                auto &&atlas = stateSpace_->as<TangentBundleStateSpace>();

                std::vector<State *> temp;
                bool success = atlas->discreteGeodesic(s1, s2, true, &temp);

                if (!success && temp.size() == 0)
                    temp.push_back(cloneState(s1));

                auto it = temp.begin();
                for (; it != temp.end(); ++it)
                {
                    auto astate = (*it)->as<AtlasStateSpace::StateType>();
                    if (!atlas->project(astate))
                        break;

                    states.push_back(astate);
                }

                while (it != temp.end())
                    freeState(*it++);

                return states.size();
            }

            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override
            {
                auto &&atlas = stateSpace_->as<TangentBundleStateSpace>();
                bool valid = motionValidator_->checkMotion(s1, s2, lastValid);

                if (lastValid.first)
                {
                    auto astate = lastValid.first->as<AtlasStateSpace::StateType>();
                    if (!atlas->project(astate))
                        valid = false;
                }

                return valid;
            }
        };
    }
}

