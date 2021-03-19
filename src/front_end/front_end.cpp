#include "front_end/front_end.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace gpm_slam
{
    FrontEnd::FrontEnd(const double& resolution,const int& map_width,const int& map_hight)
    :
    current_frame_(resolution, map_width, map_hight)
    {
       is_currentframe_new=0;
    }

    Eigen::Matrix4f FrontEnd::Update(const LineData line_in_now_) 
    {
        std::cout<<"is_currentframe_new: "<<is_currentframe_new<<std::endl;
        if ( is_currentframe_new == 0) {
            /*current_frame_.pose = init_pose_;
            current_frame_.LineData=line_in_now_;
            current_frame_.grid_map.MapInit(line_in_now_.line_ptr);*/
            UpdateNewFrame(current_frame_);

            is_currentframe_new = 1;
            return current_frame_.pose;
        }

    // 不是第一帧，就正常匹配
    double resolution_temp=current_frame_.grid_map.resolution_;
    int map_width_temp=current_frame_.grid_map.map_width_;
    int map_hight_temp=current_frame_.grid_map.map_height_;
    GridMap gridmap_temp(resolution_temp,map_width_temp,map_hight_temp);
    //gridmap_temp=current_frame_.grid_map;
    int r=gridmap_temp.Map_bel_.rows();
    int c=gridmap_temp.Map_bel_.cols();


    //std::cout<<"r= "<<r<<" c= "<<c<<std::endl;
    
    /*cv::Mat Cv_Map_Origin;
    cv::eigen2cv(gridmap_temp.Map_bel_,Cv_Map_Origin);
    Cv_Map_Origin.convertTo(Cv_Map_Origin,CV_8UC1);
    cv::imshow("figure_Origin",Cv_Map_Origin);
    cv::waitKey(10000);
    gridmap_temp.Map_bel_=Eigen::MatrixXd::Constant(r,c,1);*/
    //std::cout<<"INIT GRIDMAPTEMP\n"<<gridmap_temp.Map_bel_<<std::endl;
    //std::cout<<"Map_bel init"<<std::endl;
    float x_min=-0.5+predict_pose_(0,3);
    float x_max=0.5+predict_pose_(0,3);
    float y_min=-0.5+predict_pose_(1,3);
    float y_max=0.5+predict_pose_(1,3);
    std::cout<<"x_min= "<<x_min<<"x_max= "<<x_max<<"y_min= "<<y_min<<"y_max= "<<y_max<<std::endl;
    float x_last,y_last,theta_last;
    //std::cout<<"start set score"<<std::endl;
    float last_score=0;
    float now_score=0;
    Eigen::Matrix4f guess_pose=Eigen::Matrix4f::Identity();
    Eigen::Matrix4f transform_pose=Eigen::Matrix4f::Identity();
    float theta_this_frame,x_this_frame,y_this_frame;
    //std::cout<<"start csm "<<std::endl;
    //将每个可能位姿的点数和打分计入txt
        //std::ofstream outfile;
        std::cout<<"let score record!\n";
        /*outfile.open("/home/tanqingge/catkin_ws/src/gpm_slam/exp_data/score.txt",std::ios::out|std::ios::app);
        if(!outfile.is_open())
        {
            std::cout << "can not open file ";
		    exit(EXIT_FAILURE);
        }*/
        static int round=0;
        //outfile<<"round: "<<round<<'\n';      
        
    for(float theta_i=-PI/2;theta_i<PI/2;theta_i=theta_i+0.157)
    {
        for(float x=x_min;x<x_max;x=x+0.1)
        {
            for(float y=y_min;y<y_max;y=y+0.1)
            {
                //std::cout<<"start now_score= "<<now_score<<std::endl;
                // 更新相邻两帧的相对运动
                //std::cout<<"theta_i= "<<theta_i<<"x= "<<x<<"y= "<<y<<std::endl;

                //outfile<<"x: "<<x<<"y: "<<y<<" theta: "<<theta_i<<"\n";


                Eigen::AngleAxisf t_V(theta_i, Eigen::Vector3f(0, 0, 1));
                Eigen::Matrix3f t_R=t_V.matrix();
                guess_pose.block<3,3>(0,0)=t_R;
                guess_pose.block<3,1>(0,3)<<x,y,0;
                transform_pose=current_frame_.pose.inverse()*guess_pose;
                LineData::LINE* temp_line_ptr=new (LineData::LINE);
                LineData::LineSeg line_tmp;
                //std::cout<<"guess pose=:\n "<<guess_pose<<std::endl;
                //std::cout<<"line number is"<<line_in_now_.line_ptr->size()<<std::endl;*/
                for(int i=0;i<line_in_now_.line_ptr->size();i++)
                {
                    Eigen::Vector4f start_p;
                    start_p<<(*line_in_now_.line_ptr)[i].start_point.x,(*line_in_now_.line_ptr)[i].start_point.y,(*line_in_now_.line_ptr)[i].start_point.z,1;
                    Eigen::Vector4f end_p;
                    end_p<<(*line_in_now_.line_ptr)[i].end_point.x,(*line_in_now_.line_ptr)[i].end_point.y,(*line_in_now_.line_ptr)[i].end_point.z,1;
                    start_p = guess_pose*start_p;
                    end_p = guess_pose * end_p;
                    /*std::cout<<"start_p\n"<<start_p<<std::endl;
                    std::cout<<"end_p\n"<<end_p<<std::endl;*/
                    line_tmp.start_point.x=start_p(0);
                    line_tmp.start_point.y=start_p(1);
                    line_tmp.start_point.z=start_p(2);
                    line_tmp.end_point.x=end_p(0);
                    line_tmp.end_point.y=end_p(1);
                    line_tmp.end_point.z=end_p(2);
                    /*std::cout<<"i= "<<i<<"\n";
                    std::cout<<"start_point"<<start_p(0)<<" "<<start_p(1)<<" "<<start_p(2)<<std::endl; 
                    std::cout<<"end_point"<<end_p(0)<<" "<<end_p(1)<<" "<<end_p(2)<<std::endl;   */
                    temp_line_ptr->push_back(line_tmp);   
                               
                }
                gridmap_temp.BresenhaminMap(temp_line_ptr);
                //caculate score
                for(int i =0;i<r;i++)
                {
                    for(int j=0;j<c;j++)
                    {
                        if(gridmap_temp.Map_bel_(i,j)==0)
                        {
                            now_score = now_score + current_frame_.grid_map.Map_bel_(i,j);
                            //std::cout<<"score now = "<<now_score<<std::endl;
                            //outfile<<"i= "<<i<<"j= "<<j<<"score=: "<<now_score;
                        }
                        
                    }
                }
                if (now_score>last_score)
                {
                    theta_this_frame=theta_i;
                    x_this_frame=x;
                    y_this_frame=y;
                    last_score=now_score;
                }

                //if(x<0.0001&&x>-0.0001&&y<0.0001&&y>-0.0001&&theta_i>-0.001&&theta_i<0.001)
                //{
                    /*cv::Mat Cv_Map_;
                    cv::eigen2cv(gridmap_temp.Map_bel_,Cv_Map_);
                    Cv_Map_.convertTo(Cv_Map_,CV_8UC1);
                    Cv_Map_ = 100 * Cv_Map_;
                    cv::imshow("figure_000",Cv_Map_);
                    cv::waitKey(10);*/
                //}
                /*outfile<<"x: "<<x<<"y: "<<y<<" theta: "<<theta_i<<'\n';
                outfile<<"final now_score= "<<now_score<<'\n';*/
                /*std::cout<<"x: "<<x_this_frame<<"y: "<<y_this_frame<<" theta: "<<theta_this_frame<<std::endl;
                std::cout<<"final now_score= "<<now_score<<std::endl;*/
                
                now_score=0;
                gridmap_temp.Map_bel_=Eigen::MatrixXd::Constant(r,c,1);
            }
        }
    }
    round++;
    std::cout<<"last_score=: "<<last_score<<std::endl;
    std::cout<<"final pose is "<< "x: "<<x_this_frame<<"y: "<<y_this_frame<<" theta: "<<theta_this_frame<<std::endl;
    Eigen::AngleAxisf t_final_V(theta_this_frame, Eigen::Vector3f(0, 0, 1));
    Eigen::Matrix3f t_final_R=t_final_V.matrix();
    last_pose_.block<3,3>(0,0)=t_final_R;
    last_pose_.block<3,1>(0,3)<<x_this_frame,y_this_frame,0;
    std::cout<<"last_pose:\n "<<last_pose_<<std::endl;
    //如果在一个局部子地图中，则更新gridmap:
    LineData::LINE * update_line_ptr=new(LineData::LINE);
    LineData::LineSeg line_tmp;
    for(int i=0;i<line_in_now_.line_ptr->size();i++)
    {
        Eigen::Vector4f start_p;
        start_p<<(*line_in_now_.line_ptr)[i].start_point.x,(*line_in_now_.line_ptr)[i].start_point.y,(*line_in_now_.line_ptr)[i].start_point.z,1;
        Eigen::Vector4f end_p;
        end_p<<(*line_in_now_.line_ptr)[i].end_point.x,(*line_in_now_.line_ptr)[i].end_point.y,(*line_in_now_.line_ptr)[i].end_point.z,1;
        start_p = last_pose_*start_p;
        end_p = last_pose_ * end_p;
        line_tmp.start_point.x=start_p(0);
        line_tmp.start_point.y=start_p(1);
        line_tmp.start_point.z=start_p(2);
        line_tmp.end_point.x=end_p(0);
        line_tmp.end_point.y=end_p(1);
        line_tmp.end_point.z=end_p(2);
        update_line_ptr->push_back(line_tmp); 
    }
    current_frame_.grid_map.MapUpdate(update_line_ptr);
    
   

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(x_this_frame - current_frame_.pose(0,3)) + 
        fabs(y_this_frame - current_frame_.pose(1,3))  > 2.0) 
    {
        UpdateNewFrame(current_frame_);
        
    }

    //return current_frame_.pose;
    return last_pose_;
}


   /* void FrontEnd::PublishLocalMap(av_msgs::OccupancyGrid map)
    {
        map.header.frame_id="grid";
        map.header.stamp = ros::Time::now(); 
        map.info.resolution = current_frame_.grid_map.resolution_;
        map.info.width = current_frame_.grid_map.size_x_;           // uint32
        map.info.height = current_frame_.grid_map.size_y_;  
        int p[map.info.width*map.info.height] = {-1};   // [0,100]
        p[10] = 100;
        std::vector<signed char> a(p, p+400);
        map.data = a;
      pub.publish(map);
  }*/



    bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose)
    {
        init_pose_ = init_pose;
        return true;
    }

    void FrontEnd::SetPredictPose(const Eigen::Matrix4f& last_pose,const Eigen::Matrix4f& now_pose)
    {
        // the last_pose and now_pose are from tf raw data, carculate the trasition and add the change into the predict
        Eigen::Vector3f T_last=last_pose.block<3,1>(0,3);
        Eigen::Vector3f T_now=now_pose.block<3,1>(0,3);
        Eigen::Matrix3f R_last=last_pose.block<3,3>(0,0);
        Eigen::Matrix3f R_now=now_pose.block<3,3>(0,0);
        Eigen::Vector3f delta_t=T_now-T_last;
        Eigen::Matrix3f delta_r=R_last.inverse()*R_now;

        //the predict pose with csm algorithm
        Eigen::Matrix3f predictlast_r=last_pose_.block<3,3>(0,0);
        Eigen::Matrix3f predict_r=delta_r*predictlast_r;
        Eigen::Vector3f predictlast_t=last_pose_.block<3,1>(0,3);
        Eigen::Vector3f predict_t=delta_t+predictlast_t;
        predict_pose_.block<3,3>(0,0)=predict_r;
        predict_pose_.block<3,1>(0,3)=predict_t;
    }

    void FrontEnd::UpdateNewFrame(Frame& current_frame_)
    {
        if(is_currentframe_new == 0)
        {
            current_frame_.pose = init_pose_;
            current_frame_.line_in_frame_=line_in_now_;
            current_frame_.grid_map.MapInit(line_in_now_.line_ptr);
            local_map_frames_.push_back(current_frame_);

            cv::Mat Cv_Map_CurrentFrame;
            cv::eigen2cv(current_frame_.grid_map.Map_bel_,Cv_Map_CurrentFrame);
            Cv_Map_CurrentFrame.convertTo(Cv_Map_CurrentFrame,CV_8UC1);
            /*cv::imshow("figure_Origin",Cv_Map_CurrentFrame);
            cv::waitKey(1000);*/
        }
        else
        {
            current_frame_.pose = last_pose_;
            current_frame_.line_in_frame_=line_in_now_;
            current_frame_.grid_map.MapInit(line_in_now_.line_ptr);
            local_map_frames_.push_back(current_frame_);
            local_map_frames_.pop_front();
        
        }

    // 更新ndt匹配的目标点云
        /*if (local_map_frames_.size() ==0) 
        {
            local_map_frames_.push_back(new_key_frame);
        } 
        else 
        {
            CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
            local_map_filter_.setInputCloud(local_map_ptr_);
            local_map_filter_.filter(*filtered_local_map_ptr);
            ndt_ptr_->setInputTarget(filtered_local_map_ptr);
        }*/

    // 更新全局地图
        global_map_frames_.push_back(current_frame_);
        /*if (global_map_frames_.size() % 100 != 0) 
        {
            return;
        } 
        else 
        {
            global_map_ptr_.reset(new CloudData::CLOUD());
            for (size_t i = 0; i < global_map_frames_.size(); ++i) 
            {
                pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.cloud_ptr, 
                                    *transformed_cloud_ptr, 
                                    global_map_frames_.at(i).pose);
                *global_map_ptr_ += *transformed_cloud_ptr;
            }
            has_new_global_map_ = true;
        }*/
    };

    void FrontEnd::GetCurrentScan(LineData current_scan)
    {
        line_in_now_=current_scan;
        std::cout<<"GetCurrentScan succeed"<<std::endl;
    };

    void FrontEnd::SetlineinMap(LineData::LINE* line_ptr)
    {

    };

    void FrontEnd::PublishPredictpose()
    {   
        std::cout<<"The Predict pose is \n"<<predict_pose_<<std::endl;
    };

    FrontEnd::Frame FrontEnd::CopyFrame(FrontEnd::Frame &TempFrame)
    {
        
        TempFrame=current_frame_;
    }

}


