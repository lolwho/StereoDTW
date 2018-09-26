#pragma once

#include <vector>
#include <memory>
#include <string>

#include <opencv2/core/core.hpp>


namespace stereodtw {

struct StereoDTWSettings
{
  uint wind_wing = 1;
  uint tonel_expand_size = 2;
  uint restriction;
  std::vector<float> dilatation_cost;

  StereoDTWSettings()
  {}

  StereoDTWSettings(uint wind_wing,
           uint tonel_expand_size,
           uint restriction,
           std::vector<float> dilatation_cost)
      : wind_wing(wind_wing),
        tonel_expand_size(tonel_expand_size),
        restriction(restriction),
        dilatation_cost(dilatation_cost)
  {}

  StereoDTWSettings(const std::string &filename);
};

class StereoDTW
{
public:
  StereoDTW(const StereoDTW&) = delete;
  const StereoDTW& operator=(const StereoDTW&) = delete;

  static std::shared_ptr<StereoDTW> create(StereoDTWSettings const &set) {
    return create(set.wind_wing,set.tonel_expand_size, set.restriction, set.dilatation_cost);
  }

  static std::shared_ptr<StereoDTW> create(uint wind_wing,
                                           uint tonel_expand_size,
                                           uint restriction,
                                           std::vector<float> const &dilatation_cost) {
    return std::shared_ptr<StereoDTW>(new StereoDTW(wind_wing,tonel_expand_size, restriction, dilatation_cost));
  }

  void compute(cv::Mat const &left_img,
               cv::Mat const &right_img,
               cv::Mat *disparity);

private:
  uint wind_wing_;
  uint tonel_expand_size_;
  uint restriction_;
  std::vector<float> dilation_cost_;

  StereoDTW(uint wind_wing,
            uint tonel_expand_size,
            uint restriction,
            std::vector<float> const &dilataion_cost)
      : wind_wing_(wind_wing),
        tonel_expand_size_(tonel_expand_size),
        restriction_(restriction),
        dilation_cost_(dilataion_cost)
  {}
};

}  // stereodtw
