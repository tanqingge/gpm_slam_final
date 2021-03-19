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
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#define PI 3.1415926
#define theta_dis_threhold 10.0
#define dis_opoint 0.03
#define dis_midtoanother 0.05
#define dis_pq 0.2

using namespace gpm_slam;
using namespace std;
using namespace cv;

ros::Subscriber g_odom_suber;
Eigen::Vector3f pose_robot;

void odometryCallback ( const nav_msgs::OdometryConstPtr& odom)
{
    /* 获取机器人姿态 */
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    double theta = tf::getYaw ( odom->pose.pose.orientation );
    //g_robot_pose = Pose2d ( x, y, theta );
    pose_robot[0]=x;
    pose_robot[1]=y;
    pose_robot[2]=theta;
}

LineData::LineSeg LineRotation(double x, double y, double theta, LineData::LineSeg lineseg) 
{
	LineData::LineSeg AfterRSeg;
	AfterRSeg.start_point.x = lineseg.start_point.x * cos(theta) - lineseg.start_point.y * sin(theta) + x;
	AfterRSeg.start_point.y = lineseg.start_point.y * cos(theta) + lineseg.start_point.x * sin(theta) + y;
	AfterRSeg.end_point.x = lineseg.end_point.x * cos(theta) - lineseg.end_point.y * sin(theta) + x;
	AfterRSeg.end_point.y = lineseg.end_point.y * cos(theta) + lineseg.end_point.x * sin(theta) + y;
	return AfterRSeg;
};

void generateGrad(LineData::LineSeg & lineseg)
{
	double x_dis = lineseg.end_point.x - lineseg.start_point.x;
	double y_dis = lineseg.end_point.y - lineseg.start_point.y;
	if (x_dis == 0)
	{
		lineseg.k_ = DBL_MAX;
		lineseg.theta_ = PI / 2;
		lineseg.b_ = lineseg.end_point.x;
	}
	else 
	{
		lineseg.k_ = y_dis / x_dis;
		lineseg.theta_ = atan(lineseg.k_);
		lineseg.b_ = lineseg.end_point.y - lineseg.k_*lineseg.end_point.x;

	}

}


void SetLines(double xs,double ys,double xe,double ye, vector <LineData::LineSeg> * p)
{
	LineData::LineSeg lineseg;
	lineseg.start_point.x = xs;
	lineseg.start_point.y = ys;
	lineseg.end_point.x = xe;
	lineseg.end_point.y = ye;
	p->push_back(lineseg);
}

void SplitLineToPoints(LineData::LineSeg lineseg, int number, vector <cv::Point2f> *p)
{
	//number是
	double length = sqrt(pow((lineseg.start_point.x - lineseg.end_point.x), 2) + pow((lineseg.start_point.y - lineseg.end_point.y), 2));
	double x_step = (lineseg.end_point.x - lineseg.start_point.x)/(number-1);
	double y_step = (lineseg.end_point.y - lineseg.start_point.y)/(number-1);
	for (int i = 1; i < (number - 1); i++)
	{
		cv::Point2f point_temp;
		point_temp.x = lineseg.start_point.x + i * x_step;
		point_temp.y = lineseg.start_point.y + i * y_step;
		p->push_back(point_temp);
	}
}

bool WhetherTwoBelongOne(LineData::LineSeg p, LineData::LineSeg q)
{
	int flag1 = 0;
	int flag2 = 0;
	int flag3 = 0;
	int flag4 = 0;
	double theta_dis = abs((p.theta_ - q.theta_) / PI * 180);
	if (theta_dis < theta_dis_threhold)
		flag1 = 1;
	double dis_op = abs(p.b_) / sqrt(pow(p.k_, 2) + 1);
	double dis_oq = abs(q.b_) / sqrt(pow(q.k_, 2) + 1);
	if (abs(dis_op - dis_oq) < dis_opoint)
		flag2 = 1;
	double p_xmid = (p.start_point.x + p.end_point.x) / 2;
	double p_ymid = (p.end_point.y + p.start_point.y) / 2;
	double q_xmid = (q.start_point.x + q.end_point.x) / 2;
	double q_ymid = (q.start_point.y + q.end_point.y) / 2;
	double length_pmidtoq = abs(q.k_*p_xmid - p_ymid + q.b_) / sqrt(pow(q.k_, 2) + 1);
	if (length_pmidtoq <= dis_midtoanother)
		flag3 = 1;
	double xmid_dis = p_xmid - q_xmid;
	double ymid_dis = p_ymid - q_ymid;
	double length1 = sqrt(pow(xmid_dis, 2) + pow(ymid_dis, 2));
	double length_p = sqrt(pow(p.start_point.x - p.end_point.x, 2) + pow(p.end_point.y - p.start_point.y, 2));
	double length_q = sqrt(pow(q.end_point.x - q.start_point.x, 2) + pow(q.end_point.y - q.start_point.y, 2));
	double length2 = (length_p + length_q) / 2;
	if (length1 <= length2)
		flag4 = 1;
	int flag = flag1 + flag2 + flag3 + flag4;
	if (flag == 4)
		return true;
	return false;

};

LineData::LineSeg Merge(LineData::LineSeg p, LineData::LineSeg q)
{
	int splitnumber = 10;
	vector <cv::Point2f> p_pointset;
	vector <cv::Point2f> q_pointset;
	vector <cv::Point2f> *p_ptr;
	vector <cv::Point2f> *q_ptr;
	p_ptr = &p_pointset;
	q_ptr = &q_pointset;
	SplitLineToPoints(p, splitnumber, p_ptr);
	SplitLineToPoints(q, splitnumber, q_ptr);
	p_pointset.insert(p_pointset.end(), q_pointset.begin(), q_pointset.end());
	cv::Vec4f line_para;
	cv::fitLine(p_pointset, line_para, cv::DIST_L2, 0, 1e-2, 1e-2);
	std::cout << line_para << std::endl;
	//计算拟合直线方程：y=kx+b(点斜式）,点为point0
	cv::Point2f point0;
	point0.x = line_para[2];
	point0.y = line_para[3];
	double k= line_para[1] / line_para[0];
	double b = point0.y - k * point0.x;
	//计算原始两条线端点分别在线上的投影
	double p_x1 = (k*(p.start_point.y-b)+p.start_point.x) / (pow(k, 2) + 1);
	double p_y1 = k * p_x1 + b;
	double p_x2 = (k*(p.end_point.y - b) + p.end_point.x) / (pow(k, 2) + 1);
	double p_y2 = k * p_x2 + b;
	double q_x1 = (k*(q.start_point.y - b) + q.start_point.x) / (pow(k, 2) + 1);
	double q_y1 = k * q_x1 + b;
	double q_x2 = (k*(q.end_point.y - b) +q.end_point.x) / (pow(k, 2) + 1);
	double q_y2 = k * q_x2 + b;
	double arrayx[4] = { p_x1,p_x2,q_x1,q_x2 };
	double arrayy[4] = { p_y1,p_y2,q_y1,q_y2 };
	//排序，取x最大和最小的为新直线的两端点
	int maxPosition = max_element(arrayx, arrayx + 4) - arrayx;
	int minPosition = min_element(arrayx, arrayx + 4) - arrayx;
	LineData::LineSeg FitLine;
	FitLine.start_point.x = arrayx[minPosition];
	FitLine.start_point.y = arrayy[minPosition];
	FitLine.end_point.x = arrayx[maxPosition];
	FitLine.end_point.y = arrayy[maxPosition];
	return FitLine;
}

//用来判断前后帧哪些直线是相同的
void WhetherLineIsSame(vector <LineData::LineSeg> *p, vector <LineData::LineSeg> *q)
{
	std::vector <int> judgePisMerge;
	for(int i =0;i<p->size();i++)
	{
		judgePisMerge.push_back(0);
	}
	//p:new_frame line q:old_frame line
	for (int i = 0; i < p->size(); i++)
	{
		LineData::LineSeg NewLine = (*p)[i];
		for (int j = 0; j < q->size(); j++)
		{
			LineData::LineSeg OldLine = (*q)[j];
			bool judgement = WhetherTwoBelongOne(OldLine, NewLine);
			if (judgement == true)
			{
				std::cout << "The matching lines are Old in " << j << " and New in " << i << std::endl;
				LineData::LineSeg TempFitLine=Merge(OldLine, NewLine);
                q->erase((*q).begin()+i);
                q->insert((*q).begin()+i,TempFitLine) ;
				judgePisMerge[i]=1;
			}
			
		}
	}
	for(int i =0;i<p->size();i++)
	{
		if(judgePisMerge[i]==1)
		{
			p->erase((*p).begin()+i);
		}
		
	}
	
}

void WhetherLineIsSameTotal(vector <LineData::LineSeg> *p, vector <LineData::LineSeg> *r)
{
	//p:old_frame line q:new_frame line
	for (int i = 0; i < p->size(); i++)
	{
		LineData::LineSeg NewLine = (*p)[i];
		for (int j = i+1; j < p->size(); j++)
		{
			LineData::LineSeg OldLine = (*p)[j];
			bool judgement = WhetherTwoBelongOne(OldLine, NewLine);
			if (judgement == true)
			{
				std::cout << "The matching lines are Old in " << j << " and New in " << i << std::endl;
				LineData::LineSeg TempFitLine=Merge(OldLine, NewLine);
				r->push_back(TempFitLine);
			}				

		}
	}
}



int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "test_beihanglinemap_node");
    ros::NodeHandle nh;

    std::shared_ptr<PointSubscriber> point_sub_ptr = std::make_shared<PointSubscriber>(nh, "/scan_from_pointcloud", 1000);
    //std::shared_ptr<LineSubscriber> line_sub_ptr = std::make_shared<LineSubscriber>(nh, "current_scan", 100);
    //std::shared_ptr<TFListener> tf_to_odom_ptr = std::make_shared<TFListener>(nh, "odom", "base_link");

    //std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    //std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    //std::shared_ptr<LinePublisher> line_pub_str=std::make_shared<LinePublisher>(nh,"line_scan",200,"map");
    //std::shared_ptr<LineOnRvizPublisher> lineonviz_pub_ptr=std::make_shared<LineOnRvizPublisher>(nh,"line_segments",200,"map");
    //std::shared_ptr<NewTFListener> tf_to_odom_ptr = std::make_shared<NewTFListener>(nh, "odom", "base_link");
    
    std::deque<PointSet> point_data_buff;
    std::deque<LineData> line_data_buff;
	
    /*Eigen::Matrix4f tf_to_odom = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool line_subscribe_received=false;*/
	std::vector <LineData::LineSeg> Frame_Final;
	std::vector <LineData::LineSeg> * ptr_final;
	ptr_final = &Frame_Final;
    std::vector <LineData::LineSeg> Frame_Set_Before;
	std::vector <LineData::LineSeg> * ptr_setbefore;
	ptr_setbefore = &Frame_Set_Before;
	std::vector <LineData::LineSeg> Frame_Set_After;
	std::vector <LineData::LineSeg> * ptr_setafter;
	ptr_setafter = &Frame_Set_After;
    Mat src(1000,1000, CV_8UC3, Scalar(255, 255, 255));
	Mat Img(1000,1000, CV_8UC3, Scalar(255, 255, 255));
    //two flag for test
    int cnt=0;
    ros::Rate rate(10);
    while (ros::ok()) 
    {
        ros::spinOnce();
        clock_t start,finish;
        double totaltime;
        start=clock();
        //PointSet This_PData;
        /*std::cout<<postheta<<std::endl;
        std::cout<<posx<<std::endl;
        std::cout<<posy<<std::endl;*/
        point_sub_ptr->ParseData(point_data_buff);
        g_odom_suber = nh.subscribe ( "/odom", 1, odometryCallback );
        //tf_to_odom_ptr->LookupData(tf_to_odom);
        double posx,posy,postheta;
        posx=pose_robot[0];
        posy=pose_robot[1];
        postheta=pose_robot[2];
        //tf_to_odom_ptr->LookupData(posx,posy,postheta);
            while (point_data_buff.size() > 0) 
            {
                PointSet point_data = point_data_buff.front();            
                point_data_buff.pop_front();
                NewLineSubscriber LineSub(point_data);
                LineSub.ParseData(line_data_buff);
                std::cout<<"cnt= "<<cnt<<std::endl;
                
                while (line_data_buff.size() > 0)
                {
                    LineData line_data = line_data_buff.front();
                    line_data_buff.pop_front();
                    int line_number=line_data.line_ptr->size();
                    //直接画图
                    for (int i=0;i<line_number;i++)
                    {
                        cv::Point2f pts,pte;//start & end point 
                        (*line_data.line_ptr)[i]=LineRotation(posx,posy,postheta,(*line_data.line_ptr)[i]);
                        generateGrad((*line_data.line_ptr)[i]);
                        /*pts.x=(*line_data.line_ptr)[i].start_point.x*50+1000;
                        pts.y=(*line_data.line_ptr)[i].start_point.y*50+1500;
                        pte.x=(*line_data.line_ptr)[i].end_point.x*50+1000;
                        pte.y=(*line_data.line_ptr)[i].end_point.y*50+1500;
                        line(src, pts, pte, Scalar(0, 255, 0), 1,8,2);*/              
                    }  
					//imshow("GT_MAP",src);
                    //cv::waitKey(1);

                    //从此处开始进行线段地图合并
                    if(cnt%5==0)
                    {
                        for(int i=0;i<Frame_Final.size();i++)
                        {
                            Point pts, pte;
		                    pts.x = Frame_Final[i].start_point.x * 50 + 600;
		                    pts.y = Frame_Final[i].start_point.y * 50 + 600;
		                    pte.x = Frame_Final[i].end_point.x * 50 + 600;
		                    pte.y = Frame_Final[i].end_point.y * 50 + 600;
		                    line(src, pts, pte, Scalar(0, 0, 0), 1, 8, 1);
                        }
                        imshow("GT_MAP",src);
                        cv::waitKey(1);
						for(int i=0;i<Frame_Final.size();i++)
						{
							ptr_setbefore->push_back(Frame_Final[i]);
						}
                        Frame_Final.clear();
						std::cout<<"The size of cleared Frame_Final is "<<Frame_Final.size()<<std::endl;
                        for (int i=0;i<line_number;i++)
                        {
                            Frame_Final.push_back( (*line_data.line_ptr)[i]);
                        } 
                    }
                    else
                    {
						std::cout<<"The size of New Frame is "<<line_data.line_ptr->size()<<std::endl;
						std::cout<<"The size of Old Frame_Final is "<<Frame_Final.size()<<std::endl;
                        WhetherLineIsSame(line_data.line_ptr, ptr_final);
						std::cout<<"The size of Left Frame_New is "<<line_data.line_ptr->size()<<std::endl;
						if(line_data.line_ptr->size()>0)
						{
							for(int i=0;i<line_data.line_ptr->size();i++)
							{
								Frame_Final.push_back( (*line_data.line_ptr)[i]);
							}
						}
						
                    }
                    
                    
                }
				cnt++;
             };
           
            
        

        rate.sleep();
    }
	imwrite("/home/tanqingge/mapbeihang_withmerge.png", src);
	WhetherLineIsSameTotal(ptr_setbefore,ptr_setafter);
	for(int i=0;i<ptr_setafter->size();i++)
	{
		Point pts, pte;
		pts.x = (*ptr_setafter)[i].start_point.x * 50 + 600;
		pts.y = (*ptr_setafter)[i].start_point.y * 50 + 600;
		pte.x = (*ptr_setafter)[i].end_point.x * 50 + 600;
		pte.y = (*ptr_setafter)[i].end_point.y * 50 + 600;
		line(Img, pts, pte, Scalar(0, 0, 0), 1, 8, 1);
                        
	}
	imwrite("/home/tanqingge/mapbeihang_withmergeafter.png", Img);
	imshow("GT_MAP_Final",Img);
    cv::waitKey(10);
    return 0;
}