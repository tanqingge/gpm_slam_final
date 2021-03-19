#ifndef GPM_SLAM_GRIDMAP_GRIDMAP_HPP_
#define GPM_SLAM_GRIDMAP_GRIDMAP_HPP_

#include <eigen3/Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>

#include "sensor_data/line_data.hpp"


namespace gpm_slam
{
    /* code */
    class GridMap
    {
    public:
        GridMap(const double& resolution,const int& map_width,const int& map_hight);
        //virtual ~GridMap();

    public:
        void setGridBel(int idx,int idy, int val);
        int getGridBel(int idx,int idy);
        void MapInit(LineData::LINE* line_ptr);
        void MapUpdate(LineData::LINE* line_ptr);
        Eigen::Vector2i GetGridId();
        void SetlineinMap();
        int BresenhaminMap(LineData::LINE* line_ptr);
    public:
        /* data */
        double resolution_;
        int map_width_,map_height_;//the size of map
        int init_x_,init_y_;//the left border of map
        int size_x_,size_y_;
        Eigen::MatrixXd Map_bel_;

    };
    
}


#endif