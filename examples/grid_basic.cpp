#include <iostream>
#include <unordered_set>
#include <vector>

#include "AStar-Lite.hpp"

/**
 * @brief A 2D vector.
 */
struct Vec2D {
    int x;
    int y;

    bool operator==(const Vec2D& other) const noexcept {
        return x == other.x && y == other.y;
    }
};

/**
 * @brief A hashing representation for 2D vectors.
 */
struct Vec2DHash {
    size_t operator()(const Vec2D& v) const noexcept {
        return (std::hash<int>()(v.x) << 1) ^
                std::hash<int>()(v.y);
    }
};

/**
 * @brief The Manhattan distance between two 2D vectors.
 * @param a The first vector.
 * @param b The second vector.
 * @return The Manhattan distance between the two vectors.
 */
int manhattan_distance(const Vec2D& a, const Vec2D& b) {
    return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

/**
 * @brief The main function that demonstrates the basic grid example.
 * @return The exit code.
 */
int main() {
    const int width = 10;
    const int height = 10;

    // Set of obstacles.
    std::unordered_set<Vec2D, Vec2DHash> obstacles = {
        {3, 4}, {3, 5}, {3, 6}, {4, 6}, {5, 6}, {6, 6}
    };

    Vec2D start = {0, 0};
    Vec2D goal = {8, 8};

    astar_lite::AStarOptions<Vec2D, int, Vec2DHash> options;
    options.heuristic = [&](const Vec2D& state) {
        return manhattan_distance(state, goal);
    };

    // Neighbors for the grid.
    options.neighbors = [&](const Vec2D& state,
                            std::vector<std::pair<Vec2D, int>>& out) {
        // 4 cardinal directions.
        static const std::vector<Vec2D> dirs = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}
        };

        for (auto& dir : dirs) {
            Vec2D neighbor = {state.x + dir.x, state.y + dir.y};
            if ((neighbor.x < 0) || (neighbor.x >= width) ||
                (neighbor.y < 0) || (neighbor.y >= height)) {
                // Out of bounds.
                continue;
            }
            if (obstacles.contains(neighbor)) {
                // Obstacle.
                continue;
            }
            out.emplace_back(neighbor, 1);
        }
    };

    auto result = astar_lite::astar_search(start, goal, options);
    if (result) {
        std::cout << "Found path of length "
                  << result.cost << std::endl;

        for (auto& state : result.path) {
            std::cout << "(" << state.x << ", " << state.y << ")"
                      << std::endl;
        }

    } else {
        std::cout << "No path found." << std::endl;
    }

    return 0;
}
