//根据groundtruth生成global line共线地图:

#include <ros/ros.h>
#include <pcl/common/transforms.h>  
#include "sensor_data/line_data.hpp"
#include "subscriber/point_subscriber.hpp"
#include "subscriber/newline_subscriber.hpp"
#include "tf_listener/newtf_listener.hpp"
#include "publisher/odometry_publisher.hpp"
#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <fstream>

#define PI 3.1415926
#define theta_dis_threhold 5.0
#define dis_opoint 0.3
#define dis_midtoanother 0.3
#define dis_pq 0.2

using namespace gpm_slam;
using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "tflisteneralgorithm_node");
    ros::NodeHandle nh;

    std::shared_ptr<PointSubscriber> point_sub_ptr = std::make_shared<PointSubscriber>(nh, "scan", 1000);
    
    std::shared_ptr<NewTFListener> tf_to_odom_ptr = std::make_shared<NewTFListener>(nh, "map", "base_link");
    
    std::deque<PointSet> point_data_buff;
	
    int cnt=0;
    ros::Rate rate(0.1);
    while (ros::ok()) 
    {
        ros::spinOnce();
        point_sub_ptr->ParseData(point_data_buff);
        //tf_to_odom_ptr->LookupData(tf_to_odom);
        double posx,posy,postheta;
        posx=posy=postheta=0;
        tf_to_odom_ptr->LookupData(posx,posy,postheta); 
        std::ofstream est_lab_;
        while(point_data_buff.size() > 0)
        {
            std::cout<<"Frame: "<<cnt<<std::endl;
            std::cout<<"x: "<<posx<<" y: "<<posy<<" theta "<<postheta;
            cnt++;
        }
       
        rate.sleep();
    }
    return 0;
}