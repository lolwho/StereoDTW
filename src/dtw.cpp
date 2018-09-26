#include <utility>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dtw.h>


namespace stereodtw {

const uint8_t BACKWARD_PASS_DIAG    = 0;
const uint8_t BACKWARD_PASS_LEFT    = 125;
const uint8_t BACKWARD_PASS_UP      = 225;
const uint8_t BACKWARD_PASS_NODIR   = 200;
const uint8_t BORDER_EXTD           = 1;
const uint32_t COST_MAT_MAX_VALUE   = 1073741824; // less than 2^31 to avoid overfill

using std::vector;
using std::pair;


int img_extd(const cv::Mat &img,
  cv::Mat *img_extd,
  int wind_wing_size)
{
  int width = img.cols;
  int height = img.rows;

  cv::Mat img_extd_roi = (*img_extd)(cv::Rect(wind_wing_size, wind_wing_size, width, height));
  img.copyTo(img_extd_roi);

  cv::Mat src_roi_img = (*img_extd)(cv::Rect(wind_wing_size, 0, wind_wing_size, height + wind_wing_size));
  cv::Mat dst_roi_img = (*img_extd)(cv::Rect(0, 0, wind_wing_size, height + wind_wing_size));
  src_roi_img.copyTo(dst_roi_img);

  src_roi_img = (*img_extd)(cv::Rect(width, 0, wind_wing_size, height + wind_wing_size));
  dst_roi_img = (*img_extd)(cv::Rect(width + wind_wing_size, 0, wind_wing_size, height + wind_wing_size));
  src_roi_img.copyTo(dst_roi_img);

  src_roi_img = (*img_extd)(cv::Rect(0, wind_wing_size, width + 2 * wind_wing_size, wind_wing_size));
  dst_roi_img = (*img_extd)(cv::Rect(0, 0, width + 2 * wind_wing_size, wind_wing_size));
  src_roi_img.copyTo(dst_roi_img);

  src_roi_img = (*img_extd)(cv::Rect(0, height, width + 2 * wind_wing_size, wind_wing_size));
  dst_roi_img = (*img_extd)(cv::Rect(0, height + wind_wing_size, width + 2 * wind_wing_size, wind_wing_size));
  src_roi_img.copyTo(dst_roi_img);

  cv::imwrite("ext.png", *img_extd);

  return NO_ERRORS;
}

/* Algorythm function: builds pyramid of scaled images.
   Each side image twice smaller for current layer than for previous.
   Height of pyramid is equal to set.scale + 1
*/
int pyramid_build(
  cv::Mat const &img,
  const int scale_factor,
  vector<cv::Mat> &left_img_layers)
{
  if (scale_factor < 1) {
    return BAD_ARGS;
  }

  int m = img.cols;
  int n = img.rows;

  left_img_layers[0] = img;

  for (int i = 1; i < scale_factor; i++) {
    cv::resize(left_img_layers[i - 1], left_img_layers[i], cv::Size((m + 1) / 2, (n + 1) / 2));

    m = left_img_layers[i].cols;
    n = left_img_layers[i].rows;
  };

  return NO_ERRORS;
}

/* Algorythm function: calculate disparity value for corresponding lines of
   two images.
*/
int disparity_generation(vector<int> *disparities,
    const vector<cv::Mat> &left_img_layers_extd,
    const vector<cv::Mat> &right_img_layers_extd,
    int wind_wing,
    int tonel_expand_size,
    int restriction,
    std::vector<float> const & dilataion_cost,
    int line_num)
{
  int m = 0;

// Declaration:
  int layers_extd_size = dilataion_cost.size();
  vector<cv::Mat> cost_mat_vec(layers_extd_size);
  vector<cv::Mat> path_mat_vec(layers_extd_size);
  vector<pair<int,int>> tonel_patch (left_img_layers_extd[0].cols - 2 * wind_wing + BORDER_EXTD);

// Allocation:
  for(int scale = 0; scale < layers_extd_size; scale++) {      // the larger scale the smaller image
    m = left_img_layers_extd[scale].cols - 2 * wind_wing + BORDER_EXTD;

    cost_mat_vec[scale] = cv::Mat(m, m, CV_32SC1, cv::Scalar(COST_MAT_MAX_VALUE));
    path_mat_vec[scale] = cv::Mat(m, m, CV_8UC1, cv::Scalar(BACKWARD_PASS_NODIR));
  }

// Definition:
  tonel_patch[0].first = 0;
  tonel_patch[0].second = 0;
  for (int i = 1; i < m; ++i) {
    tonel_patch[i].first  = BORDER_EXTD;
    tonel_patch[i].second = i;
  }

// Disparity computation:
  for(int scale = layers_extd_size - 1; scale > -1; --scale)
  {
     if(scale != 0)
     {
       // cost_mat_computation()
       // backward_pass()
       // tonel_patch_expansion()
     }

    if (scale == 0) {
      // cost_mat_computation()
      // backward_pass_final()
    }
  }

  return 0;
}

/* Algorythm function: calculate disparity map using two rectified and undistored images.
*/
int dtw(cv::Mat *disparity_map,
        cv::Mat const &left_image,
        cv::Mat const &right_image,
        int wind_wing,
        int tonel_expand_size,
        int restriction,
        std::vector<float> const & dilataion_cost)
{
  for (size_t i = 0; i < dilataion_cost.size(); ++i) {
    if (dilataion_cost[i] < 0) {
      return BAD_ARGS;
    }
  }

  int img_width = left_image.cols;
  int img_height  =  left_image.rows;

  int layers_num = dilataion_cost.size();

  vector<cv::Mat> left_img_layers(layers_num);
  pyramid_build(left_image, dilataion_cost.size(), left_img_layers);

  vector<cv::Mat> right_img_layers(layers_num);
  pyramid_build(right_image, dilataion_cost.size(), right_img_layers);

  vector<cv::Mat> left_img_layers_extd(layers_num);
  vector<cv::Mat> right_img_layers_extd(layers_num);

  for (int scale = 0; scale < layers_num; scale++) { // the larger scale the smaller image
    int n, m;
    m = left_img_layers[scale].cols;
    n = left_img_layers[scale].rows;

    left_img_layers_extd[scale] = cv::Mat::zeros(cv::Size(m + 2 * wind_wing, n + 2 * wind_wing), CV_8UC1);
    right_img_layers_extd[scale] = cv::Mat::zeros(cv::Size(m + 2 * wind_wing, n + 2 * wind_wing), CV_8UC1);

    img_extd(left_img_layers[scale], &left_img_layers_extd[scale], wind_wing);
    img_extd(right_img_layers[scale], &right_img_layers_extd[scale], wind_wing);
  }

  *disparity_map = cv::Mat::zeros(left_image.size(), CV_8UC1);

  for (int line = 0; line < img_height; line++) {
    vector<int> disparities;
    disparity_generation(&disparities,
                         left_img_layers_extd,
                         right_img_layers_extd,
                         wind_wing,
                         tonel_expand_size,
                         restriction,
                         dilataion_cost,
                         line);

    int disparity_size = static_cast<int>(disparities.size());
    for (int j = 0; j < disparity_size; j++) {
      disparity_map->at<uint8_t>(line, j) = static_cast<uint8_t>(disparities[j]);
    }

    if (img_width - disparity_size + 1 < 0) {
      throw std::runtime_error("Error: too big disparity (greater than width of image)");
    }

    disparities.clear();
  }

  return 0;
}

} // stereodtw
