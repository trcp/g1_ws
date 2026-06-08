#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace machida_navigation
{

float circumscribed_radius(const std::vector<std::pair<float, float>> & points);

float footprint_radius(const std::string & s, float default_radius = 0.2f);

std::vector<std::pair<float, float>> parse_footprint(const std::string & s);

std::vector<int8_t> inflate_grid(
  const std::vector<int8_t> & grid_data,
  int width, int height,
  int footprint_cells, int padding_cells,
  int obstacle_threshold = 50,
  int8_t footprint_cost  = 99,
  int8_t padding_cost    = 75);

std::vector<int8_t> distance_transform_grid(
  const std::vector<int8_t> & grid_data,
  int width, int height,
  int footprint_cells, int padding_cells,
  int obstacle_threshold  = 50,
  int8_t obstacle_cost    = 100,
  int8_t footprint_cost   = 99,
  int8_t max_padding_cost = 98,
  float free_space_weight = 0.0f,
  int8_t unknown_cost     = 0);

// Build a raw 2D occupancy grid by marking obstacle points.
// origin_x/origin_y: world coordinates of the grid's bottom-left corner.
// obstacle_points: (x, y) positions in the same world frame as the grid origin.
std::vector<int8_t> build_obstacle_grid(
  int width, int height,
  float resolution,
  float origin_x,
  float origin_y,
  const std::vector<std::pair<float, float>> & obstacle_points,
  int8_t obstacle_value = 100);

}  // namespace machida_navigation
