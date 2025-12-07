#pragma once

#include <vector>

namespace astar_lite {

    /**
     * @brief The result of an A* search.
     * @tparam S State type.
     * @tparam C Cost type.
     */
    template<
        typename S,
        typename C
    >
    struct AStarResult {
        // Whether the search was successful.
        bool success = false;
        // The path from the start to the goal.
        std::vector<S> path;
        // The cost of the path.
        C cost{};
        // The number of expansions required to reach the goal.
        size_t expansions = 0;

        explicit operator bool() const noexcept {
            return success;
        }
    };

} // namespace astar_lite
