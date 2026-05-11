#pragma once

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace machida_navigation
{

using Point = std::pair<int, int>;
using Path  = std::vector<Point>;
using ProgressCb = std::function<void(const std::string &)>;

std::optional<Path> astar(
  const std::vector<int8_t> & grid_data,
  int width, int height,
  int start_gx, int start_gy,
  int goal_gx, int goal_gy,
  int obstacle_threshold     = 50,
  ProgressCb progress_cb     = nullptr,
  int log_interval           = 500,
  float obstacle_cost_weight = 0.0f);

std::vector<std::pair<float, float>> smooth_path(
  const Path & path,
  float weight_data   = 0.5f,
  float weight_smooth = 0.1f,
  float tolerance     = 1e-4f);

}  // namespace machida_navigation
