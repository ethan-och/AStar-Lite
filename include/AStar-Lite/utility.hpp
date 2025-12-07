#pragma once

#include <unordered_map>
#include <vector>

namespace astar_lite {

    /**
     * @brief Reconstructs the path from the start to the goal.
     * @tparam S State type.
     * @tparam H Hash function type.
     * @tparam E Equality function type.
     * @param goal The goal state.
     * @param parent The parent map.
     * @return The path from the start to the goal.
     */
    template<
        typename S,
        typename H,
        typename E
    >
    std::vector<S> reconstruct_path(
            const S& goal,
            const std::unordered_map<S, S, H, E>& parent) {
        std::vector<S> path;
        path.push_back(goal);
        auto it = parent.find(goal);

        while (it != parent.end()) {
            const S& prev = it->second;
            path.push_back(prev);
            it = parent.find(prev);
        }

        std::reverse(path.begin(), path.end());
        return path;
    }

    /**
     * @brief Retrieve a value from a map, otherwise return default.
     * @tparam K Key type.
     * @tparam V Value type.
     * @tparam H Hash function type.
     * @tparam E Equality function type.
     * @param map The map.
     * @param key The key.
     * @param default_value The default value.
     * @return The value, or the default value if not found.
     */
    template<
        typename K,
        typename V,
        typename H,
        typename E
    >
    V get_or_default(
            const std::unordered_map<K, V, H, E>& map,
            const K& key,
            const V& default_value) {
        auto it = map.find(key);
        return (it == map.end()) ? default_value : it->second;
    }

} // namespace astar_lite
