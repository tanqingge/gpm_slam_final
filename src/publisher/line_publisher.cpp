#include "publisher/line_publisher.hpp"
#include <time.h>
namespace gpm_slam {

        LinePublisher::LinePublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   size_t buff_size,
                   std::string frame_id)
         :nh_(nh),frame_id_(frame_id){
             publisher_=nh_.advertise<gpm_slam::Line_Segment>(topic_name,buff_size);
                   };
        void LinePublisher::Publish(LineData line_data){
            //clock_t startTime,endTime;
            //startTime = clock();//计时开始
            new_line_msgs_.header.stamp = ros::Time::now();
            new_line_msgs_.header.seq = 1;
            new_line_msgs_.header.frame_id = frame_id_;
            int line_number_=line_data.line_ptr->size();
            new_line_msgs_.num_lines=line_number_;
            new_line_msgs_.start.resize(line_number_);
            new_line_msgs_.end.resize(line_number_);
            for(int i=0;i<line_number_;i++){
                new_line_msgs_.start[i].x=(*line_data.line_ptr)[i].start_point.x;
                new_line_msgs_.start[i].y=(*line_data.line_ptr)[i].start_point.y;
                new_line_msgs_.end[i].x=(*line_data.line_ptr)[i].end_point.x;
                new_line_msgs_.end[i].y=(*line_data.line_ptr)[i].end_point.y;
            }
            publisher_.publish(new_line_msgs_);
            //endTime = clock();
            //std::cout << "The run time of publish is: " <<(double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
        };

}