#include "machida_navigation/costmap.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <tuple>
#include <vector>

namespace machida_navigation
{

static constexpr float SQRT2 = 1.41421356f;

static std::vector<float> parse_float_list(const std::string & s)
{
  std::string stripped;
  for (char c : s) {
    if (c == ' ' || c == '[' || c == ']') continue;
    stripped += (c == ';') ? ',' : c;
  }

  std::vector<float> vals;
  if (stripped.empty() || stripped == "none") return vals;

  std::stringstream ss(stripped);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (token.empty()) continue;
    vals.push_back(std::stof(token));
  }
  return vals;
}

float circumscribed_radius(const std::vector<std::pair<float, float>> & points)
{
  float max_r = 0.0f;
  for (auto & [x, y] : points) {
    max_r = std::max(max_r, std::sqrt(x * x + y * y));
  }
  return max_r;
}

float footprint_radius(const std::string & s, float default_radius)
{
  std::vector<float> vals;
  try {
    vals = parse_float_list(s);
  } catch (...) {
    return default_radius;
  }

  if (vals.empty()) return default_radius;
  if (vals.size() == 1) return std::abs(vals[0]);
  if (vals.size() == 2) {
    float half_length = std::abs(vals[0]) * 0.5f;
    float half_width  = std::abs(vals[1]) * 0.5f;
    return std::sqrt(half_length * half_length + half_width * half_width);
  }

  auto footprint = parse_footprint(s);
  return footprint.empty() ? default_radius : circumscribed_radius(footprint);
}

std::vector<std::pair<float, float>> parse_footprint(const std::string & s)
{
  std::vector<float> vals;
  try {
    vals = parse_float_list(s);
  } catch (...) {
    return {};
  }

  if (vals.size() == 2) {
    float half_length = vals[0] * 0.5f;
    float half_width  = vals[1] * 0.5f;
    return {
      { half_length,  half_width},
      { half_length, -half_width},
      {-half_length, -half_width},
      {-half_length,  half_width},
    };
  }

  if (vals.size() < 6 || vals.size() % 2 != 0) return {};

  std::vector<std::pair<float, float>> result;
  for (size_t i = 0; i < vals.size(); i += 2) {
    result.emplace_back(vals[i], vals[i + 1]);
  }
  return result;
}

static std::vector<std::pair<int, int>> make_circle_offsets(int r)
{
  std::vector<std::pair<int, int>> offsets;
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (dx * dx + dy * dy <= r * r) {
        offsets.emplace_back(dx, dy);
      }
    }
  }
  return offsets;
}

std::vector<int8_t> inflate_grid(
  const std::vector<int8_t> & grid_data,
  int width, int height,
  int footprint_cells, int padding_cells,
  int obstacle_threshold,
  int8_t footprint_cost,
  int8_t padding_cost)
{
  int total_cells = footprint_cells + padding_cells;
  if (total_cells <= 0) return grid_data;

  auto footprint_offsets = (footprint_cells > 0) ?
    make_circle_offsets(footprint_cells) :
    std::vector<std::pair<int, int>>{};
  auto total_offsets = make_circle_offsets(total_cells);

  int dim = 2 * total_cells + 1;
  std::vector<bool> is_footprint(dim * dim, false);
  for (auto & [dx, dy] : footprint_offsets) {
    is_footprint[(dx + total_cells) * dim + (dy + total_cells)] = true;
  }

  std::vector<int8_t> result = grid_data;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int8_t v = grid_data[y * width + x];
      if (v >= 0 && static_cast<int>(v) < obstacle_threshold) continue;

      for (auto & [dx, dy] : total_offsets) {
        int nx = x + dx, ny = y + dy;
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

        int idx = ny * width + nx;
        int8_t orig = grid_data[idx];
        if (orig < 0 || static_cast<int>(orig) >= obstacle_threshold) continue;

        int8_t cost = is_footprint[(dx + total_cells) * dim + (dy + total_cells)] ?
          footprint_cost : padding_cost;
        if (result[idx] < cost) result[idx] = cost;
      }
    }
  }

  return result;
}

std::vector<int8_t> distance_transform_grid(
  const std::vector<int8_t> & grid_data,
  int width, int height,
  int footprint_cells, int padding_cells,
  int obstacle_threshold,
  int8_t obstacle_cost,
  int8_t footprint_cost,
  int8_t max_padding_cost)
{
  const int cell_count = width * height;
  if (width <= 0 || height <= 0 || static_cast<int>(grid_data.size()) != cell_count) {
    return {};
  }

  static constexpr float INF = std::numeric_limits<float>::infinity();
  std::vector<float> distance_to_obstacle(cell_count, INF);
  std::vector<int8_t> result(cell_count, 0);

  struct QueueNode {
    float distance;
    int idx;
    bool operator>(const QueueNode & other) const
    {
      return distance > other.distance;
    }
  };

  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open_set;

  for (int idx = 0; idx < cell_count; ++idx) {
    int8_t v = grid_data[idx];
    if (v < 0 || static_cast<int>(v) >= obstacle_threshold) {
      distance_to_obstacle[idx] = 0.0f;
      result[idx] = obstacle_cost;
      open_set.push({0.0f, idx});
    } else if (v > 0) {
      result[idx] = v;
    }
  }

  static constexpr std::array<std::tuple<int, int, float>, 8> MOVES = {{
    {-1, -1, SQRT2}, {-1, 0, 1.0f}, {-1, 1, SQRT2},
    { 0, -1, 1.0f},                  { 0, 1, 1.0f},
    { 1, -1, SQRT2}, { 1, 0, 1.0f}, { 1, 1, SQRT2},
  }};

  auto in_bounds = [&](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
  };

  while (!open_set.empty()) {
    auto [cur_dist, cur_idx] = open_set.top();
    open_set.pop();

    if (cur_dist > distance_to_obstacle[cur_idx] + 1e-6f) continue;

    int cx = cur_idx % width;
    int cy = cur_idx / width;
    for (auto & [dx, dy, move_cost] : MOVES) {
      int nx = cx + dx;
      int ny = cy + dy;
      if (!in_bounds(nx, ny)) continue;

      int next_idx = ny * width + nx;
      float next_dist = cur_dist + move_cost;
      if (next_dist + 1e-6f < distance_to_obstacle[next_idx]) {
        distance_to_obstacle[next_idx] = next_dist;
        open_set.push({next_dist, next_idx});
      }
    }
  }

  const float footprint = static_cast<float>(std::max(0, footprint_cells));
  const float padding   = static_cast<float>(std::max(0, padding_cells));
  const float total     = footprint + padding;

  for (int idx = 0; idx < cell_count; ++idx) {
    if (result[idx] == obstacle_cost) continue;

    float d = distance_to_obstacle[idx];
    if (d <= footprint + 1e-6f) {
      result[idx] = std::max(result[idx], footprint_cost);
      continue;
    }

    if (padding <= 0.0f || d > total) continue;

    float ratio = (total - d) / padding;
    ratio = std::clamp(ratio, 0.0f, 1.0f);
    int cost = static_cast<int>(std::ceil(ratio * static_cast<float>(max_padding_cost)));
    result[idx] = std::max(result[idx], static_cast<int8_t>(cost));
  }

  return result;
}

std::vector<int8_t> build_obstacle_grid(
  int width, int height,
  float resolution,
  float origin_x,
  float origin_y,
  const std::vector<std::pair<float, float>> & obstacle_points,
  int8_t obstacle_value)
{
  std::vector<int8_t> grid(width * height, 0);
  for (auto & [px, py] : obstacle_points) {
    int cx = static_cast<int>((px - origin_x) / resolution);
    int cy = static_cast<int>((py - origin_y) / resolution);
    if (cx >= 0 && cx < width && cy >= 0 && cy < height) {
      grid[cy * width + cx] = obstacle_value;
    }
  }
  return grid;
}

}  // namespace machida_navigation
