#include <iostream>
#include <gtest/gtest.h>
#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include "booster_vision/base/misc_utils.hpp"

namespace booster_vision {
TEST(ConfigUtilsTest, MergeConfig) {
  YAML::Node template_node = YAML::Load("{\
    a: 1,          \
    b: {           \
      c: 2,        \
      d: 3         \
    }              \
  }");
  YAML::Node input_node = YAML::Load("{\
    b: {           \
      c: 4         \
    },             \
    e: 5           \
  }");

  auto merge_node = YAML::Clone(template_node);
  MergeYAML(merge_node, input_node);
  
  
  std::cout << "template node: " << template_node << std::endl;
  std::cout << "input node: " << input_node << std::endl;
  std::cout << "merged node: " << merge_node << std::endl;
}

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}