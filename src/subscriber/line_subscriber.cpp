#include "subscriber/line_subscriber.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <time.h>

namespace gpm_slam{
    LineSubscriber::LineSubscriber(ros::NodeHandle& nh, std::string topic_name,size_t buff_size)
    :nh_(nh),origincloud_for_line_ptr(new LineData::CLOUD){
        subscriber_=nh_.subscribe(topic_name,buff_size,&LineSubscriber::msg_callback,this);
        
    };
    
    void LineSubscriber::ParseData(std::deque<LineData>& line_data_buff)
    {
       // clock_t startTime,endTime;
      //  startTime = clock();
        std::cout<<"new line_data size"<<new_line_data_.size()<<std::endl;
        if(new_line_data_.size()>0)
        {
            line_data_buff.insert(line_data_buff.end(),new_line_data_.begin(),new_line_data_.end());
            new_line_data_.clear();
        }
        /*endTime = clock();//计时结束
        std::cout << "The run time of parse data is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;*/
    };

    void LineSubscriber::NewLineDataInit()
    {
        /*clock_t startTime,endTime;
        startTime = clock();*/
        for(int i=0;i<MAXSEGMENTS;i++)
        {
            init_segs_[i].count_=0;
            init_segs_[i].grad_=0;
            init_segs_[i].invgrad_=0;
            init_segs_[i].x_=0;
            init_segs_[i].y_=0;
            init_segs_[i].xx_=0;
            init_segs_[i].xy_=0;
            init_segs_[i].yy_=0;
            init_segs_[i].x_start_=0;
            init_segs_[i].y_start_=0;
            init_segs_[i].x_end_=0;
            init_segs_[i].y_end_=0;
            init_segs_[i].xMean_=0;
            init_segs_[i].yMean_=0;
        }
        //endTime = clock();//计时结束
        //std::cout << "The run time of init segment data is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    };
    void LineSubscriber::FitLineWithPoint()
    {
           //clock_t startTime,endTime;
           //startTime = clock();
           int line_num_=0;
           int point_size=origincloud_for_line_ptr->points.size();
           static int count=0;
           std::cout<<"points size"<<point_size<<std::endl;
           for (int i=0;i<point_size;i++)
           {
               int seg_state_=0;
               float x_temp_=origincloud_for_line_ptr->points[i].x;
               float y_temp_=origincloud_for_line_ptr->points[i].y;
               for (int j=0; j<line_num_; j++)
               {
                   double d_,err_;
                   if (init_segs_[j].grad_<__DBL_MAX__)
                   {
                       d_=init_segs_[j].grad_*x_temp_-y_temp_+init_segs_[j].yMean_-init_segs_[j].grad_*init_segs_[j].xMean_;
                       err_=abs(d_)/sqrt(init_segs_[j].grad_*init_segs_[j].grad_+1);
                   }
                   else
                   {
                       d_=init_segs_[j].invgrad_*y_temp_-x_temp_+init_segs_[j].xMean_-init_segs_[j].invgrad_*init_segs_[j].yMean_;
                       err_=abs(d_)/sqrt(init_segs_[j].invgrad_*init_segs_[j].invgrad_+1);
                   }
                   double d1_=sqrt(pow(x_temp_-init_segs_[j].x_end_,2)+pow(y_temp_-init_segs_[j].y_end_,2));
                   double d2_=sqrt(pow(x_temp_-init_segs_[j].x_start_,2)+pow(y_temp_-init_segs_[j].y_start_,2));
                   if ((err_<connect_th_) && ((d1_<min_distance_)||(d2_<min_distance_)))
                   {
                       init_segs_[j].xMean_=(init_segs_[j].count_*init_segs_[j].xMean_+x_temp_)/(init_segs_[j].count_+1);
                       init_segs_[j].yMean_=(init_segs_[j].count_*init_segs_[j].yMean_+y_temp_)/(init_segs_[j].count_+1);
                       init_segs_[j].x_ += x_temp_;
                       init_segs_[j].y_ += y_temp_;
                       init_segs_[j].xx_ += pow(x_temp_,2);
                       init_segs_[j].yy_ += pow(y_temp_,2);
                       init_segs_[j].xy_ += x_temp_*y_temp_;
                       init_segs_[j].count_ += 1;
                       seg_state_ +=1;
                       init_segs_[j].grad_=(init_segs_[j].xy_-init_segs_[j].x_*init_segs_[j].y_/init_segs_[j].count_)/(init_segs_[j].xx_-init_segs_[j].x_*init_segs_[j].x_/init_segs_[j].count_);
                       if (init_segs_[j].grad_==__DBL_MAX__)
                            init_segs_[j].invgrad_=(init_segs_[j].xy_-init_segs_[j].x_*init_segs_[j].y_/init_segs_[j].count_)/(init_segs_[j].yy_-init_segs_[j].y_*init_segs_[j].y_/init_segs_[j].count_);
                       init_segs_[j].x_end_=x_temp_;
                       init_segs_[j].y_end_=y_temp_;
                   }
                 }
                 if ((seg_state_==0) && (line_num_<MAXSEGMENTS))
                 {
                     init_segs_[line_num_].x_start_=x_temp_;
                     init_segs_[line_num_].y_start_=y_temp_;
                     init_segs_[line_num_].x_end_=x_temp_;
                     init_segs_[line_num_].y_end_=y_temp_;
                     init_segs_[line_num_].xMean_=x_temp_;
                     init_segs_[line_num_].yMean_=y_temp_;
                     init_segs_[line_num_].xx_=pow(x_temp_,2);
                     init_segs_[line_num_].yy_=pow(y_temp_,2);
                     init_segs_[line_num_].xy_=x_temp_*y_temp_;
                     init_segs_[line_num_].count_=1;
                     init_segs_[line_num_].x_=x_temp_;
                     init_segs_[line_num_].y_=y_temp_;
                     line_num_ += 1;
                 }
            }
            count++;
            //endTime = clock();//计时结束
            //std::cout << "The run time of Fitline is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
            if(count==1500)
            {
                std::ofstream outfile;
                outfile.open("/home/tanqingge/catkin_ws/src/gpm_slam/exp_data/lidat.txt", std::ios::out|std::ios::app);
                for(int i=0;i<point_size;i++){
                    outfile<<origincloud_for_line_ptr->points[i].x<<" "<<origincloud_for_line_ptr->points[i].y<<"\n";
                }
                outfile.close();
            };
    }; 

    void LineSubscriber::WhetherSegmentIsValid(LineData line_data_)
    {
        //clock_t startTime,endTime;
        //startTime = clock();
        
        LineData::LineSeg segs_temp_;
        static int num__=0;
        for (int i=0;i<MAXSEGMENTS;i++)
        {
            double length_=sqrt(pow(init_segs_[i].x_end_-init_segs_[i].x_start_,2)+pow(init_segs_[i].y_end_-init_segs_[i].y_start_,2));
            //std::cout<< "i:"<<i<<"Length"<<length_<<std::endl;
            if((init_segs_[i].count_>count_threhold_)&&(length_>=length_limit_))
            {
                if(init_segs_[i].grad_!=__DBL_MAX__)
                {
                    segs_temp_.start_point.x=init_segs_[i].xMean_-length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));
                    segs_temp_.start_point.y=init_segs_[i].yMean_-init_segs_[i].grad_*length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));
                    segs_temp_.end_point.x=init_segs_[i].xMean_+length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));
                    segs_temp_.end_point.y=init_segs_[i].yMean_+init_segs_[i].grad_*length_/(2*sqrt(pow(init_segs_[i].grad_,2)+1));
                }
                else if ((init_segs_[i].grad_==__DBL_MAX__)&&(init_segs_[i].invgrad_!=0))
                {
                    segs_temp_.start_point.x=init_segs_[i].xMean_-init_segs_[i].invgrad_*length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
                    segs_temp_.start_point.y=init_segs_[i].yMean_-length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
                    segs_temp_.end_point.x=init_segs_[i].xMean_+init_segs_[i].invgrad_*length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
                    segs_temp_.end_point.y=init_segs_[i].yMean_+length_/(2*sqrt(pow(init_segs_[i].invgrad_,2)+1));
                }
            //new_line_data_.push_back(segs_temp_);
            std::cout<<"x_start "<<segs_temp_.start_point.x<<std::endl;
            std::cout<<"y_start "<<segs_temp_.start_point.y<<std::endl;
            std::cout<<"x_end "<<segs_temp_.end_point.x<<std::endl;
            std::cout<<"y_end "<<segs_temp_.end_point.y<<std::endl;
            line_data_.line_ptr->push_back(segs_temp_);
           // num__+=1;
            //std::cout<<num__<<" "<<length_<<std::endl;
            }

        }
        num__++;
        if(num__==1500)
        {
            std::ofstream outfile;
            outfile.open("/home/tanqingge/catkin_ws/src/gpm_slam/exp_data/line.txt", std::ios::out|std::ios::app);
            for(int i=0;i<line_data_.line_ptr->size();i++){
                outfile<<(*line_data_.line_ptr)[i].start_point.x<<" "<<(*line_data_.line_ptr)[i].start_point.y<<" "<<(*line_data_.line_ptr)[i].end_point.x<<" "<<(*line_data_.line_ptr)[i].end_point.y<<"\n";
                }
                outfile.close();
            };
        std::cout<<"line number in a frame "<<line_data_.line_ptr->size()<<std::endl;
        //endTime = clock();//计时结束
        //std::cout << "The run time of WhetherValid is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

        
    };
    void LineSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
    {
        LineData line_data_;
        std::cout<<"subscribe current_scan successfully"<<std::endl;
        line_data_.time=cloud_msg_ptr->header.stamp.toSec();
        pcl::fromROSMsg(*cloud_msg_ptr,*origincloud_for_line_ptr);
        static int cnt_=0;
        std::ofstream outfile_2;
        outfile_2.open("/home/tanqingge/catkin_ws/src/gpm_slam/exp_data/line_data_buff.txt", std::ios::out|std::ios::app);
        outfile_2<<"cloud count"<<cnt_<<"\n";
        for(int i=0;i<origincloud_for_line_ptr->points.size();i++)
        {
            outfile_2<<origincloud_for_line_ptr->points[i].x<<" "<<origincloud_for_line_ptr->points[i].y<<"\n";
        }
        outfile_2.close();
        cnt_++;
        NewLineDataInit();
        FitLineWithPoint();
        WhetherSegmentIsValid(line_data_);
        clock_t startTime,endTime;
        startTime = clock();
        new_line_data_.push_back(line_data_);
        endTime = clock();//计时结束
        //std::cout << "The run time of Pushdata in new data is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

    };
    } 
    