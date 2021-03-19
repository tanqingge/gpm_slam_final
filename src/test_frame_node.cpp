#include <ros/ros.h>
#include <pcl/common/transforms.h>

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/line_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "publisher/line_publisher.hpp"
#include "publisher/lineonrviz_publisher.hpp"
#include <time.h>

using namespace gpm_slam;

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "scan", 100000);
    std::shared_ptr<LineSubscriber> line_sub_ptr = std::make_shared<LineSubscriber>(nh, "current_scan", 100);
    //std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/oxts/gps/fix", 1000000);*/
    std::shared_ptr<TFListener> tf_to_odom_ptr = std::make_shared<TFListener>(nh, "odom", "base_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    std::shared_ptr<LinePublisher> line_pub_str=std::make_shared<LinePublisher>(nh,"line_scan",200,"map");
    std::shared_ptr<LineOnRvizPublisher> lineonviz_pub_ptr=std::make_shared<LineOnRvizPublisher>(nh,"line_segments",200,"map");

    std::deque<CloudData> cloud_data_buff;
    std::deque<LineData> line_data_buff;
    /*std::deque<IMUData> imu_data_buff;
    std::deque<GNSSData> gnss_data_buff;*/
    Eigen::Matrix4f tf_to_odom = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool line_subscribe_received=false;
    //bool gnss_origin_position_inited = false;
    

    //two flag for test
    static int cnt_1=0;
    static int cnt_2=0;
    ros::Rate rate(100);
    while (ros::ok()) 
    {
        ros::spinOnce();
        clock_t start,finish;
        double totaltime;
        start=clock();
        cloud_sub_ptr->ParseData(cloud_data_buff);
        tf_to_odom_ptr->LookupData(tf_to_odom);
        /*imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);*/

        /*if (!transform_received) 
        {
            if (tf_to_odom_ptr->LookupData(tf_to_odom))
            {
                transform_received = true;
                // LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
            }
        } 
        else {*/
            while (cloud_data_buff.size() > 0) 
            {
                CloudData cloud_data = cloud_data_buff.front();
                /*IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();*/

                /*double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();*/
                cloud_data_buff.pop_front();
                Eigen::Matrix4f odometry_matrix =tf_to_odom;

                    /*if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    gnss_data.UpdateXYZ();
                    odometry_matrix(0,3) = gnss_data.local_E;
                    odometry_matrix(1,3) = gnss_data.local_N;
                    odometry_matrix(2,3) = gnss_data.local_U;
                    odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();*/
                   
                    pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix);
                    
                    cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
                    odom_pub_ptr->Publish(odometry_matrix);
                    std::cout<<"hello world"<<std::endl;
                    line_sub_ptr->ParseData(line_data_buff);
                    finish=clock();
                    totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
                    std::cout<<"run time of transform"<<totaltime<<"s"<<std::endl;
                    
                    /*std::ofstream outfile_1;
                    outfile_1.open("/home/tanqingge/catkin_ws/src/gpm_slam/exp_data/cloud_data_buff.txt", std::ios::out|std::ios::app);
                    outfile_1<<"cloud count"<<cnt_1<<"\n";
                    for(int i=0;i<(*cloud_data.cloud_ptr).size();i++)
                    {
                       
                        outfile_1<<(*cloud_data.cloud_ptr).points[i].x<<" "<<(*cloud_data.cloud_ptr).points[i].y<<"\n";
                    }
                    outfile_1.close();
                    cnt_1++;*/
                
            }
            while (line_data_buff.size() > 0) 
            {
                        LineData line_data = line_data_buff.front();
                        line_data_buff.pop_front();
                        line_pub_str->Publish(line_data);
                        // the test in txt
                        Eigen::Matrix4f odometry_matrix=tf_to_odom;
                        lineonviz_pub_ptr->Publish(line_data,odometry_matrix);
             };
           
            
        

        rate.sleep();
    }

    return 0;
}