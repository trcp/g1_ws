#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace erasers_g1_common
{
constexpr std::size_t kG1JointCount = 29;
constexpr std::size_t kG1UpperBodyJointCount = 17;

struct G1JointDescriptor { const char * name; std::size_t motor_index; };
struct G1JointControlLimit
{
  std::string name;
  std::size_t motor_index;
  double lower;
  double upper;
  double velocity;
  bool active{true};
};

// 順序は SDK の 0～28 番を維持し、URDF にない軸を詰めない。
const std::array<G1JointDescriptor, kG1JointCount> & g1JointDescriptors();
bool loadG1JointLimits(
  const std::string & urdf, std::vector<G1JointControlLimit> & limits,
  std::string & error);
bool loadAndValidateG1UpperBodyLimits(
  const std::string & urdf, std::vector<G1JointControlLimit> & limits,
  std::string & error);
}  // namespace erasers_g1_common
