#include <ros/ros.h>
#include <pcl/common/transforms.h>

#include "front_end/front_end.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/line_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "publisher/line_publisher.hpp"
#include "publisher/lineonrviz_publisher.hpp"
#include <time.h>

using namespace gpm_slam;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh_, "scan", 100000);
    std::shared_ptr<LineSubscriber> line_sub_ptr = std::make_shared<LineSubscriber>(nh_, "current_scan", 100);
    std::shared_ptr<TFListener> tf_to_odom_ptr = std::make_shared<TFListener>(nh_, "odom", "base_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh_, "current_scan", 100, "map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh_, "lidar_odom", "map", "lidar", 100);
    std::shared_ptr<LinePublisher> line_pub_str=std::make_shared<LinePublisher>(nh_,"line_scan",200,"map");
    std::shared_ptr<LineOnRvizPublisher> lineonviz_pub_ptr=std::make_shared<LineOnRvizPublisher>(nh_,"line_segments",200,"map");
    //std::shared_ptr<OdometryPublisher> tf_pub_ptr = std::make_shared<OdometryPublisher>(nh_,"tf_odom","map","raw",100);

    ros::Publisher pub = nh_.advertise<nav_msgs::OccupancyGrid>("/gridMap", 1);
    nav_msgs::OccupancyGrid map;
    std::shared_ptr<FrontEnd> front_end_ptr =std::make_shared<FrontEnd>(0.1,40,40);
    std::deque<CloudData> cloud_data_buff;
    std::deque<LineData> line_data_buff;

    Eigen::Matrix4f tf_to_odom = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f last_pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f now_pose = Eigen::Matrix4f::Identity();
    
    bool transform_received = false;
    bool line_subscribe_received=false;
    bool front_end_inited=false;


    static int count_tf=0;

    ros::Rate rate(100);
    while (ros::ok())
    {
        /* code for loop body */
        ros::spinOnce();
        //subscribe raw pointcloud data
        cloud_sub_ptr->ParseData(cloud_data_buff);
        //get the initial pose(use tf raw data)
        if(!transform_received)
        {
            if(tf_to_odom_ptr->LookupData(tf_to_odom))
            {
                transform_received=true;
                tf_to_odom_ptr->LookupData(tf_to_odom);
                last_pose=tf_to_odom;
                now_pose=tf_to_odom;
            }
        }
        //else
        //{
            while (cloud_data_buff.size() > 0) 
            {
                CloudData cloud_data = cloud_data_buff.front();
                cloud_data_buff.pop_front();
                Eigen::Matrix4f odometry_matrix =tf_to_odom;
                pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, odometry_matrix);
                cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
                std::cout<<"hello world"<<std::endl;
                line_sub_ptr->ParseData(line_data_buff);
                while(line_data_buff.size()>0)
                {

                        LineData line_data = line_data_buff.front();
                        line_data_buff.pop_front();
                        tf_to_odom_ptr->LookupData(tf_to_odom);
                        now_pose=tf_to_odom;
                        //tf_pub_ptr->Publish(tf_to_odom);
                        if(!front_end_inited)
                        {
                            front_end_inited=true;
                            front_end_ptr->SetInitPose(now_pose);
                        }
                        front_end_ptr->SetPredictPose(last_pose,now_pose);
                        front_end_ptr->PublishPredictpose();
                        front_end_ptr->GetCurrentScan(line_data);
                        Eigen::Matrix4f current_pose=front_end_ptr->Update(line_data);
                        std::cout<<"current_pose\n"<<current_pose<<std::endl;
                        //line_pub_str->Publish(current_scan_ptr);
                        odom_pub_ptr->Publish(current_pose);
                        FrontEnd::Frame ShowFrame(0.1,40,40);
                        front_end_ptr->CopyFrame(ShowFrame);
                        //publish occupied map: 
                        {
                            map.header.frame_id="grid";
                            map.header.stamp = ros::Time::now(); 
                            map.info.resolution = ShowFrame.grid_map.resolution_;
                            map.info.width = ShowFrame.grid_map.size_x_;           // uint32
                            map.info.height = ShowFrame.grid_map.size_y_;  
                            map.info.origin.position.x = current_pose(0,3);
                            map.info.origin.position.y = current_pose(1,3);
                            map.info.origin.position.z = current_pose(2,3);
                            map.info.origin.orientation.x = 0.0;
                            map.info.origin.orientation.y = 0.0;
                            map.info.origin.orientation.z = 0.0;
                            map.info.origin.orientation.w = 1.0;
                            map.data.resize(map.info.width * map.info.height);
                            for (int i = 0; i < map.info.width;i++)
                            {
                                for (int j = 0; j < map.info.width;j++)
                                {
                                    map.data[i*map.info.width+j] = ShowFrame.grid_map.Map_bel_(i,j);
                                }
                            }
                            pub.publish(map);
                        }
                        lineonviz_pub_ptr->Publish(line_data,current_pose);
                        //tf_pub_ptr->Publish(now_pose);
                        /*if (run_time > 460.0 && !has_global_map_published) 
                        {
                            if (front_end_ptr->GetNewGlobalMap(global_map_ptr))
                            {
                                global_map_pub_ptr->Publish(global_map_ptr);
                                has_global_map_published = true;
                            }
                        }*/
                        
                        
                        last_pose=now_pose;

                }



            //}
                
            
        }
    }
    return 0;
}