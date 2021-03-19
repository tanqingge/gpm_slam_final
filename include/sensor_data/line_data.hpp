#ifndef GPM_SLAM_SENSOR_DATA_LINE_DATA_HPP_
#define GPM_SLAM_SENSOR_DATA_LINE_DATA_HPP_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <boost/make_shared.hpp>

namespace gpm_slam {
class LineData {
  public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;
    struct LineSeg{
      POINT start_point;
      POINT end_point;
      bool isnew_;//whether a new line for map
      int count_;// the number of calclate as valid segments
      //double A_,B_,C_;//AX+BY+C=0;
      double k_,b_,theta_;//y=kx+b;
    };

    using LINE = std::vector <LineSeg>;
    //using LINE_PTR LINE*;
    public:
      LineData()
        :line_ptr(new LINE()) {
      }

    public:
      double time = 0.0;
      //LINE_PTR line_ptr;
      LINE* line_ptr;
      
};
}

#endif