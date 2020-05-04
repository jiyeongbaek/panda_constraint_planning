#pragma once

#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/geometric/planners/PlannerIncludes.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <mutex>
#include <utility>
#include <vector>
#include <map>

#include <constraint_planner/panda_model_updater.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <constraint_planner/KinematicChain.h>

#include <ompl/base/goals/GoalState.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {
        class newPRM : public base::Planner
        {
        public:
            struct vertex_state_t
            {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_total_connection_attempts_t
            {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_successful_connection_attempts_t
            {
                typedef boost::vertex_property_tag kind;
            };

            typedef boost::adjacency_list<
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property<
                    vertex_state_t, base::State *,
                    boost::property<
                        vertex_total_connection_attempts_t, unsigned long int,
                        boost::property<vertex_successful_connection_attempts_t, unsigned long int,
                                        boost::property<boost::vertex_predecessor_t, unsigned long int,
                                                        boost::property<boost::vertex_rank_t, unsigned long int>>>>>,
                boost::property<boost::edge_weight_t, base::Cost>>
                Graph;

            /* The type for a vertex in the roadmap. */
            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
            /* The type for an edge in the roadmap. */
            typedef boost::graph_traits<Graph>::edge_descriptor Edge;

            /*  A nearest neighbors data structure for roadmap vertices. */
            typedef std::shared_ptr<NearestNeighbors<Vertex>> RoadmapNeighbors;

            /*  A function returning the milestones that should be
             * attempted to connect to. */
            typedef std::function<const std::vector<Vertex> &(const Vertex)> ConnectionStrategy;

            /*  A function that can reject connections.

             This is called after previous connections from the neighbor list
             have been added to the roadmap.
             */
            typedef std::function<bool(const Vertex &, const Vertex &)> ConnectionFilter;

            /** \brief Constructor */
            newPRM(const base::SpaceInformationPtr &si, bool starStrategy = false);

            ~newPRM() override;

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be make to for a
             given milestone.

             \par The behavior and performance of newPRM can be changed drastically
             by varying the number and properties if the milestones that are
             connected to each other.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }
            /** \brief Convenience function that sets the connection strategy to the
             default one with k nearest neighbors.
             */
            void setMaxNearestNeighbors(unsigned int k);
            void setConnectionFilter(const ConnectionFilter &connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief While the termination condition allows, this function will construct the roadmap (using
               growRoadmap() and expandRoadmap(),
                maintaining a 2:1 ratio for growing/expansion of roadmap) */

            void constructRoadmap(const base::PlannerTerminationCondition &ptc, base::State *mid, double distance);
            
            /** If the user desires, the roadmap can be
                improved for the given time (seconds). The solve()
                method will also improve the roadmap, as needed.*/
            void growRoadmap(double growTime);

            /** \brief If the user desires, the roadmap can be
                improved until a given condition is true. The solve()
                method will also improve the roadmap, as needed.*/
            void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *mid, double distance);

            /** \brief Attempt to connect disjoint components in the roadmap
                using random bouncing motions (the newPRM expansion step) for the
                given time (seconds). */
            void expandRoadmap(double expandTime);

            /** \brief Attempt to connect disjoint components in the roadmap
                using random bouncing motions (the newPRM expansion step) until the
                given condition evaluates true. */
            void expandRoadmap(const base::PlannerTerminationCondition &ptc);

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for newPRM. */
            void clearQuery();

            void clear() override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() == 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Vertex>>();
                if (!userSetConnectionStrategy_)
                    connectionStrategy_ = ConnectionStrategy();
                if (isSetup())
                    setup();
            }

            void setup() override;

            const Graph &getRoadmap() const
            {
                return g_;
            }

            /** \brief Return the number of milestones currently in the graph */
            unsigned long int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Return the number of edges currently in the graph */
            unsigned long int edgeCount() const
            {
                return boost::num_edges(g_);
            }

            const RoadmapNeighbors &getNearestNeighbors()
            {
                return nn_;
            }

        protected:
            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data
               structure
                and then connect it to the roadmap in accordance to the connection strategy. */
            Vertex addMilestone(base::State *state);

            /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with
             * fewer elements will get the id of the component with more elements. */
            void uniteComponents(Vertex m1, Vertex m2);

            /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a
             * const function since we use incremental connected components from boost */
            bool sameComponent(Vertex m1, Vertex m2);

            /** \brief Randomly sample the state space, add and connect milestones
                 in the roadmap. Stop this process when the termination condition
                 \e ptc returns true.  Use \e workState as temporary memory. */
            void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState, base::State *mid, double distance);
            void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState, base::State *start_state, base::State *goal_state);

            /** \brief Attempt to connect disjoint components in the
                roadmap using random bounding motions (the newPRM
                expansion step) */
            void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State *> &workStates);

            /** Thread that checks for solution */
            void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is
             * in \e start and the second is in \e goal, and the two milestones are in the same connected component. If
             * a solution is found, it is constructed in the \e solution argument. */
            bool maybeConstructSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals,
                                        base::PathPtr &solution);

            /** \brief (Assuming that there is always an approximate solution), finds an
             * approximate solution. */
            ompl::base::Cost constructApproximateSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution);

            /** \brief Returns the value of the addedNewSolution_ member. */
            bool addedNewSolution() const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set
             * it as the solution */
            base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
                This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(Vertex u, Vertex v) const;

            /** \brief Compute distance between two milestones (this is simply distance between the states of the
             * milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const
            {
                return std::to_string(iterations_);
            }
            std::string getBestCost() const
            {
                return std::to_string(bestCost_.value());
            }
            std::string getMilestoneCountString() const
            {
                return std::to_string(milestoneCount());
            }
            std::string getEdgeCountString() const
            {
                return std::to_string(edgeCount());
            }

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool starStrategy_;

            /** \brief Sampler user for generating valid samples in the state space */

            base::ValidStateSamplerPtr sampler_;
            // base::jy_ConstrainedValidStateSamplerPtr sampler_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr simpleSampler_;

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors nn_;

            /** \brief Connectivity graph */
            Graph g_;

            /** \brief Array of start milestones */
            std::vector<Vertex> startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex> goalM_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type stateProperty_;

            /** \brief Access to the number of total connection attempts for a vertex */
            boost::property_map<Graph, vertex_total_connection_attempts_t>::type totalConnectionAttemptsProperty_;

            /** \brief Access to the number of successful connection attempts for a vertex */
            boost::property_map<Graph, vertex_successful_connection_attempts_t>::type
                successfulConnectionAttemptsProperty_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Data structure that maintains the connected components */
            boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                                 boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy connectionStrategy_;

            /** \brief Function that can reject a milestone connection */
            ConnectionFilter connectionFilter_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are
             * assumed) */
            bool userSetConnectionStrategy_{false};

            /** \brief Random number generator */
            RNG rng_;

            /** \brief A flag indicating that a solution has been added during solve() */
            bool addedNewSolution_{false};

            /** \brief Mutex to guard access to the Graph member (g_) */
            mutable std::mutex graphMutex_;

            /** \brief Objective cost function for newPRM graph edges */
            base::OptimizationObjectivePtr opt_;

            //////////////////////////////
            // Planner progress properties
            /** \brief Number of iterations the algorithm performed */
            unsigned long int iterations_{0};
            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};


            std::shared_ptr<FrankaModelUpdater> panda_arm;
            Eigen::Matrix<double, 7, 1> q_temp;
            Eigen::VectorXd u_eigen, v_eigen;
            // base::jy_ConstrainedSpaceInformationPtr si_;


        };
    }
}

