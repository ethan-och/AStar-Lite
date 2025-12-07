#pragma once

#include <chrono>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>

#include "AStar-Lite/result.hpp"
#include "AStar-Lite/types.hpp"
#include "AStar-Lite/utility.hpp"

namespace astar_lite {

    /**
     * @brief Performs an A* search from the start to the goal state.
     * @tparam S State type.
     * @tparam C Cost type.
     * @tparam H Hash function type.
     * @tparam E Equality function type.
     * @param start The start state.
     * @param goal The goal state.
     * @param options The options for the A* search.
     * @return The result of the A* search.
     */
    template<
        typename S,
        typename C,
        typename H = std::hash<S>,
        typename E = std::equal_to<S>
    >
    AStarResult<S, C> astar_search(
            const S& start,
            const S& goal,
            const AStarOptions<S, C, H, E>& options) {
        using node_t = NodeRecord<S, C>;
        using compare_t = CompareNodeRecord<S, C>;
        using parent_map_t = std::unordered_map<S, S, H, E>;
        using goal_map_t = std::unordered_map<S, C, H, E>;

        AStarResult<S, C> result{};
        if ((!options.heuristic) || (!options.neighbors)) {
            return result;
        }

        if (start == goal) {
            result.success = true;
            result.path = {start};
            result.cost = C{};
            result.expansions = 0;
            return result;
        }

        auto cmp = [](const node_t& lhs,
                      const node_t& rhs) noexcept {
            return lhs.f > rhs.f;
        };
        std::priority_queue<
            node_t, std::vector<node_t>, decltype(cmp)
        > open(cmp);

        parent_map_t parent_map{};
        goal_map_t goal_map{};

        goal_map[start] = C{};
        C start_h = options.heuristic(start);
        open.push(node_t{start, C{}, start_h});

        size_t expansions = 0;
        size_t peak_open_size = 1;

        std::vector<std::pair<S, C>> neighbors{};

        while (!open.empty()) {
            peak_open_size = std::max(peak_open_size, open.size());
            node_t current = open.top();
            open.pop();

            auto it_goal = goal_map.find(current.state);
            if (it_goal == goal_map.end()) {
                continue;
            }
            if (current.g > it_goal->second) {
                continue;
            }

            if (current.state == goal) {
                result.success = true;
                result.expansions = expansions;
                result.cost = current.g;
                result.path = reconstruct_path<S, H, E>(goal, parent_map);
                return result;
            }

            if (++expansions > options.max_expansions) {
                break;
            }

            if (options.clear_neighbors) {
                neighbors.clear();
            }
            options.neighbors(current.state, neighbors);

            for (auto& [num_state, step_cost] : neighbors) {
                C tentative_g = current.g + step_cost;

                auto goal_it = goal_map.find(num_state);
                bool better = (goal_it == goal_map.end()) ||
                              (tentative_g < goal_it->second);

                if (better) {
                    parent_map[num_state] = current.state;
                    goal_map[num_state] = tentative_g;
                    C f = tentative_g + options.heuristic(num_state);
                    open.push(node_t{num_state, tentative_g, f});
                }
            }
        }

        result.success = false;
        result.expansions = expansions;
        return result;
    }

} // namespace astar_lite
