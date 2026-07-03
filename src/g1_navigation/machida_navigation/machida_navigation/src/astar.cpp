#include "machida_navigation/astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <queue>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace machida_navigation
{

static constexpr float SQRT2 = 1.41421356f;

std::optional<Path> astar(
  const std::vector<int8_t> & grid_data,
  int width, int height,
  int start_gx, int start_gy,
  int goal_gx, int goal_gy,
  int obstacle_threshold,
  ProgressCb progress_cb,
  int log_interval,
  float obstacle_cost_weight)
{
  auto in_bounds = [&](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
  };
  auto is_obstacle = [&](int x, int y) {
    int8_t v = grid_data[y * width + x];
    return v < 0 || static_cast<int>(v) >= obstacle_threshold;
  };
  auto traversal_multiplier = [&](int x, int y) {
    int8_t v = grid_data[y * width + x];
    if (v <= 0 || obstacle_cost_weight <= 0.0f) return 1.0f;

    float normalized_cost = static_cast<float>(v) /
      static_cast<float>(std::max(obstacle_threshold - 1, 1));
    normalized_cost = std::clamp(normalized_cost, 0.0f, 1.0f);
    return 1.0f + obstacle_cost_weight * normalized_cost;
  };
  auto h = [&](int x, int y) {
    float dx = static_cast<float>(x - goal_gx);
    float dy = static_cast<float>(y - goal_gy);
    return std::sqrt(dx * dx + dy * dy);
  };

  if (is_obstacle(start_gx, start_gy) || is_obstacle(goal_gx, goal_gy)) {
    return std::nullopt;
  }

  static constexpr std::array<std::tuple<int, int, float>, 8> MOVES = {{
    {-1, -1, SQRT2}, {-1, 0, 1.0f}, {-1, 1, SQRT2},
    { 0, -1, 1.0f},                  { 0, 1, 1.0f},
    { 1, -1, SQRT2}, { 1, 0, 1.0f}, { 1, 1, SQRT2},
  }};

  struct Node {
    float f, g;
    int counter, idx;
    bool operator>(const Node & o) const
    {
      return f > o.f || (f == o.f && counter > o.counter);
    }
  };

  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
  std::unordered_map<int, float> g_score;
  std::unordered_map<int, int> came_from;

  int start_idx = start_gy * width + start_gx;
  int goal_idx  = goal_gy  * width + goal_gx;

  g_score[start_idx] = 0.0f;
  int counter = 0, expanded = 0;
  open_set.push({h(start_gx, start_gy), 0.0f, counter++, start_idx});

  while (!open_set.empty()) {
    auto [f_cur, g_pushed, _cnt, cur_idx] = open_set.top();
    open_set.pop();

    auto git = g_score.find(cur_idx);
    if (git != g_score.end() && git->second < g_pushed - 1e-6f) continue;

    ++expanded;

    if (progress_cb && log_interval > 0 && expanded % log_interval == 0) {
      int cx = cur_idx % width, cy = cur_idx / width;
      std::ostringstream oss;
      oss << "expanded=" << expanded
          << ", open=" << open_set.size()
          << ", current=(" << cx << "," << cy << ")"
          << std::fixed << std::setprecision(1)
          << ", g=" << g_pushed
          << ", h=" << h(cx, cy)
          << ", f=" << f_cur;
      progress_cb(oss.str());
    }

    if (cur_idx == goal_idx) {
      Path path;
      int idx = cur_idx;
      while (came_from.count(idx)) {
        path.emplace_back(idx % width, idx / width);
        idx = came_from[idx];
      }
      path.emplace_back(start_gx, start_gy);
      std::reverse(path.begin(), path.end());
      return path;
    }

    int cx = cur_idx % width, cy = cur_idx / width;
    float g_cur = g_score.count(cur_idx) ?
      g_score[cur_idx] : std::numeric_limits<float>::infinity();

    for (auto & [dx, dy, move_cost] : MOVES) {
      int nx = cx + dx, ny = cy + dy;
      if (!in_bounds(nx, ny) || is_obstacle(nx, ny)) continue;

      int neighbor_idx = ny * width + nx;
      float traversal_cost = 0.5f * (
        traversal_multiplier(cx, cy) + traversal_multiplier(nx, ny));
      float tentative_g = g_cur + move_cost * traversal_cost;

      auto it = g_score.find(neighbor_idx);
      if (it == g_score.end() || tentative_g < it->second - 1e-6f) {
        came_from[neighbor_idx] = cur_idx;
        g_score[neighbor_idx]   = tentative_g;
        float f = tentative_g + h(nx, ny);
        open_set.push({f, tentative_g, counter++, neighbor_idx});
      }
    }
  }

  return std::nullopt;
}

std::vector<std::pair<float, float>> smooth_path(
  const Path & path,
  float weight_data,
  float weight_smooth,
  float tolerance)
{
  if (path.size() <= 2) {
    std::vector<std::pair<float, float>> result;
    for (auto & [x, y] : path) result.emplace_back(static_cast<float>(x), static_cast<float>(y));
    return result;
  }

  std::vector<std::pair<float, float>> new_path;
  for (auto & [x, y] : path) new_path.emplace_back(static_cast<float>(x), static_cast<float>(y));

  float change = tolerance + 1.0f;
  while (change > tolerance) {
    change = 0.0f;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
      for (int j = 0; j < 2; ++j) {
        float orig = (j == 0) ?
          static_cast<float>(path[i].first) : static_cast<float>(path[i].second);
        float & cur  = (j == 0) ? new_path[i].first  : new_path[i].second;
        float prev   = (j == 0) ? new_path[i - 1].first : new_path[i - 1].second;
        float next   = (j == 0) ? new_path[i + 1].first : new_path[i + 1].second;

        float old_val = cur;
        cur += weight_data   * (orig - cur);
        cur += weight_smooth * (prev + next - 2.0f * cur);
        change += std::abs(old_val - cur);
      }
    }
  }

  return new_path;
}

}  // namespace machida_navigation
