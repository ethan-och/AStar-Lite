#pragma once

#include <functional>
#include <vector>

namespace astar_lite {

    /**
     * @brief Holds a node in the open set.
     * @tparam S State type.
     * @tparam C Cost type.
     */
    template<
        typename S,
        typename C
    >
    struct NodeRecord {
        // The state of the node.
        S state;
        // The cost to reach the node from the start.
        C g;
        // The estimated cost to reach the goal from the node.
        C f;
    };

    /**
     * @brief Options for the A* search algorithm.
     * @tparam S State type.
     * @tparam C Cost type.
     * @tparam H Hash function type.
     * @tparam E Equality function type.
     */
    template<
        typename S,
        typename C,
        typename H = std::hash<S>,
        typename E = std::equal_to<S>
    >
    struct AStarOptions {
        // Heuristic function as a function from state to cost.
        std::function<C(const S&)> heuristic;
        // Fills a vector with state-cost pairs for a neighbor.
        std::function<
            void(const S&, std::vector<std::pair<S, C>>&)
        > neighbors;
        // Maximum number of expansions before giving up.
        size_t max_expansions = std::numeric_limits<size_t>::max();
        // Whether to clear the neighbor vector before each call.
        bool clear_neighbors = true;
    };

    /**
     * @brief Compares two node records for the open set.
     * @tparam S State type.
     * @tparam C Cost type.
     */
    template<
        typename S,
        typename C
    >
    struct CompareNodeRecord {
        bool operator()(const NodeRecord<S, C>& lhs,
                        const NodeRecord<S, C>& rhs) const noexcept {
            return lhs.f > rhs.f;
        }
    };

} // namespace astar_lite
