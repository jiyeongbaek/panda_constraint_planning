#pragma once

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <constraint_planner/base/jy_SpaceInformation.h>
class jy_UniformValidStateSampler : public ompl::base::UniformValidStateSampler
{
    public:
    jy_UniformValidStateSampler(const ompl::base::SpaceInformation *si) : ompl::base::UniformValidStateSampler(si)
    {

    }

    bool sample(ompl::base::State *state) override
    {
        unsigned int attempts = 0;
        bool valid = false;
        do
        {
            sampler_->sampleUniform(state);
            valid = si_->isValid(state);
            ++attempts;
        } while (!valid && attempts < attempts_);
        return valid;
    }
};