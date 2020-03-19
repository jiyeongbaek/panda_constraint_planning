#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/StateSampler.h>
#include <ompl/base/ValidStateSampler.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

#include <Eigen/Core>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::NoRandomSampleSpace */
        OMPL_CLASS_FORWARD(NoRandomSampleSpace);
        /// @endcond

        /** \brief StateSampler for use for a projection-based state space. */
        class NoRandomSampler : public WrapperStateSampler
        {
        public:
            /** \brief Constructor. */
            NoRandomSampler(const NoRandomSampleSpace *space, StateSamplerPtr sampler);

            /** \brief Sample a state uniformly in ambient space and project to
             * the manifold. Return sample in \a state. */
            void sampleUniform(State *state) override;

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance in ambient space and project to the
             * manifold. Return sample in \a state. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev in ambient space and project to the
                manifold. Return sample in \a state. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            /** \brief Constraint. */
            const ConstraintPtr constraint_;
        };


        /** \brief ConstrainedStateSpace encapsulating a projection-based
         * methodology for planning with constraints. */
        class NoRandomSampleSpace : public ConstrainedStateSpace
        {
        public:
            /** \brief Construct an atlas with the specified dimensions. */
            NoRandomSampleSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint)
              : ConstrainedStateSpace(ambientSpace, constraint)
            {
                setName("Projected" + space_->getName());
            }

            /** \brief Destructor. */
            ~NoRandomSampleSpace() override = default;

            /** \brief Allocate the default state sampler for this space. */
            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<NoRandomSampler>(this, space_->allocDefaultStateSampler());
            }

            /** \brief Allocate the previously set state sampler for this space. */
            StateSamplerPtr allocStateSampler() const override
            {
                return std::make_shared<NoRandomSampler>(this, space_->allocStateSampler());
            }

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a geodesic
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a geodesic.*/
            
            bool discreteGeodesic(const State *from, const State *to, bool interpolate = false,
                                  std::vector<State *> *geodesic = nullptr) const override;
        };
    }
}
