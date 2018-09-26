#include <iostream>
#include <exception>
#include <string>
#include <ctime>

#include <stereo_matcher.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace stereodtw;

int main( int argc, char* argv[] )
{
  try {

    if (argc != 4 && argc != 5) {
      throw std::runtime_error("Input format: Img1, Img2, config, [output]\n");
    }

    std::string out_path = (argc == 5) ? argv[4] : "dtw_result.png";

    cv::Mat left_img;
    cv::Mat right_img;

    left_img = cv::imread(argv[1]);
    right_img = cv::imread(argv[2]);

    if (!left_img.data || !right_img.data) {
      throw std::runtime_error("Failed to load images");
    }

    stereodtw::StereoDTWSettings set(argv[3]);

    cv::Mat depth_map;

    auto stereo = stereodtw::StereoDTW::create(set);

    const clock_t begin_time = clock();
    stereo->compute(left_img, right_img, &depth_map);
    std::cout << "Time: " << float( clock () - begin_time ) /  CLOCKS_PER_SEC << "s" << std::endl;

    cv::imwrite(out_path.c_str(), depth_map);

    return 0;
  }
  catch (std::exception const& e) {
    std::cerr << "Exception: " << e.what() << std::endl;
    return 1;
  }
}
