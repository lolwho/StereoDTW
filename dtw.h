#pragma once

#include <opencv/cv.h>
#include <vector>


namespace stereodtw {

enum ErrorCode {
  NO_ERRORS =  0,
  BAD_ARGS  = -1
};

int dtw(cv::Mat *disparity_map,
        cv::Mat const &left_image,
        cv::Mat const &right_image,
        int wind_wing,
        int tonel_expand_size,
        int restriction,
        std::vector<float> const & dilataion_cost);

} // stereodtw
