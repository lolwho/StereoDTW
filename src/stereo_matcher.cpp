#include <fstream>

#include <json-cpp/json.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <stereo_matcher.h>
#include <dtw.h>


namespace stereodtw{

StereoDTWSettings::StereoDTWSettings(const std::string &filename)
{
  Json::Value cfgdata;
  Json::Reader reader;
  std::ifstream cfgfile(filename);
  if (!reader.parse(cfgfile, cfgdata)) {
    throw std::runtime_error("Failed to parse dtw settings from file " + filename);
  }
  wind_wing = cfgdata["wind_wing"].asUInt();
  tonel_expand_size = cfgdata["tonel_expand_size"].asUInt();
  restriction = cfgdata["restriction"].asUInt();
  for (auto cost_json : cfgdata["dilatation_cost"]) {
    dilatation_cost.push_back(cost_json.asDouble());
  }
}

void StereoDTW::compute(cv::Mat const &left_img,
                        cv::Mat const &right_img,
                        cv::Mat *disparity)
{
  cv::Mat left_img_gray;
  cv::Mat right_img_gray;

  cv::cvtColor(left_img, left_img_gray, CV_BGR2GRAY);
  cv::cvtColor(right_img, right_img_gray, CV_BGR2GRAY);


  dtw(disparity, left_img_gray, right_img_gray, wind_wing_, tonel_expand_size_, restriction_, dilation_cost_);
}

} // stereodtw
