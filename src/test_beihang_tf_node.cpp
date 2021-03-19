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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>

#define PI 3.1415926
#define theta_dis_threhold 5.0
#define dis_opoint 0.3
#define dis_midtoanother 0.3
#define dis_pq 0.2
#define resolution_large_  0.05
#define board_xmin -30
#define board_xmax 30
#define board_ymin - 30
#define board_ymax 30
#define dis_sigma 1.0
#define theta_sigma 0.5
#define SearchLength_ 0.5

using namespace gpm_slam;
using namespace std;
using namespace cv;


ros::Publisher g_odom_pub;
ros::Subscriber g_odom_sub;
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

struct pose
{
	double x;
	double y;
	double theta;
};

struct LineSeg
{
	double x_start_, y_start_;
	double x_end_, y_end_;
	double k_, b_, theta_;
};

struct CSMCorrelatePose{
	double x_;
	double y_;
	double theta_;
	double score_;
};

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

void generateGradB(LineSeg & lineseg)
{
	double x_dis = lineseg.x_end_ - lineseg.x_start_;
	double y_dis = lineseg.y_end_ - lineseg.y_start_;
	if (x_dis == 0)
	{
		lineseg.k_ = DBL_MAX;
		lineseg.theta_ = PI / 2;
		lineseg.b_ = lineseg.x_end_;
	}
	else
	{
		lineseg.k_ = y_dis / x_dis;
		lineseg.theta_ = atan(lineseg.k_);
		lineseg.b_ = lineseg.y_end_ - lineseg.k_*lineseg.x_end_;

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

void InsertPointToMap(std::vector<Eigen::Vector3f> * linepts_ptr, int x_, int y_,double theta_)
{
	Eigen::Vector3f pt;
	pt[0] = x_;
	pt[1] = y_;
	pt[2] = theta_;
	linepts_ptr->push_back(pt);
}

/*void BresenhaminMap(std::vector <LineSeg> Linetempsegs, float resolution_,int origin_x_,int origin_y_,  int board_x_,int board_y_,std::vector<Eigen::Vector3f> * linepts_ptr)
{
	
	int pnt_cnt = 0;
	int line_number = Linetempsegs.size();
	for (int i = 0; i < line_number; i++)
	{
		double line_theta_ = Linetempsegs[i].theta_;
		float dy = Linetempsegs[i].y_end_- Linetempsegs[i].y_start_;
		float dx = Linetempsegs[i].x_end_ - Linetempsegs[i].x_start_;
		dy = (int)std::abs(dy);
		dx = (int)std::abs(dx);
		int x1 = std::round(Linetempsegs[i].x_start_ / resolution_) + origin_x_;
		int x2 = std::round(Linetempsegs[i].x_end_/ resolution_) + origin_x_;
		int y1 = std::round(Linetempsegs[i].y_start_ / resolution_) + origin_y_;
		int y2 = std::round(Linetempsegs[i].y_end_/ resolution_) + origin_y_;
		//int sign1 = x2-x1?1:-1;
		//int sign2 = y2-y1?1:-1;
		//直线近似于垂直情况
		if (x1 == x2)
		{
			int y_min = y1 > y2 ? y1 : y2;
			int y_max = y1 > y2 ? y2 : y1;
			for (int i = y_min; i < y_max ; i++)
			{
				int j = x1+origin_x_;
				//std::cout << " i= " << i << " j= " << j << std::endl;
				if ((i >= 0) && (j >= 0) && (i < board_x_) && (j < board_y_))
				{
					//setGridBel(j, i, 0);
					//注意Matrix的行列及方向和坐标系XY相反
					int x_ = j;
					int y_ = board_y_ - i;
					InsertPointToMap(linepts_ptr, j, i, line_theta_);
					//setGridBel(i, j, 0);
				}
				else
					continue;
				pnt_cnt++;
			}
		}
		//剩下的情况
		else {
			bool xySwap = false;//用来统计xy是否需要交换
			if (dx < dy)
			{
				xySwap = true;
				std::swap(x1, y1);
				std::swap(x2, y2);
			}
			//保证x的起始点永远小于终止点，是++
			if (x1 > x2) {
				std::swap(x1, x2);
				std::swap(y1, y2);
			}
			int increase = y1 < y2 ? 1 : -1;
			int r1 = 2 * dy;
			int r2 = r1 - 2 * dx;
			int p = r1 - dx;
			//interchange ? setGridBel(y1 + init_x_, x1 + init_y_, 0) : setGridBel(x1 + init_x_, y1 + init_y_, 0);
			//std::cout << " i= " << x1  << " j= " << y1 << std::endl;
			if ((x1 >= 0) && (y1 >= 0) && (x1 < board_x_) && (y1 < board_y_))
			{
				xySwap ? InsertPointToMap(linepts_ptr, board_y_ - y1, x1, line_theta_) : InsertPointToMap(linepts_ptr, x1, board_y_-y1, line_theta_);
			}
			int x = x1, y = y1;
			int x_, y_;
			while (x < x2)
			{
				x++;
				if (p > 0)
				{
					p = p + r2;
					y = y + increase;
					//interchange ? setGridBel(y + init_x_, x + init_y_, 0) : setGridBel(x + init_x_, y + init_y_, 0);
					//std::cout << " i= " << x  << " j= " << y  << std::endl;
					if ((x >= 0) && (y >= 0) && (x < board_x_) && (y < board_y_))
					{
						xySwap ? InsertPointToMap(linepts_ptr, board_y_ - y, x, line_theta_) : InsertPointToMap(linepts_ptr,x, board_y_ - y, line_theta_);
					}
					pnt_cnt++;
				}
				else
				{
					p += r1;
					//interchange ? setGridBel(y + init_x_, x + init_y_, 0) : setGridBel(x + init_x_, y + init_y_, 0);
					//std::cout << " i= " << x  << " j= " << y  << std::endl;
					if ((x >= 0) && (y >= 0) && (x < board_x_) && (y < board_y_))
					{
						xySwap ? InsertPointToMap(linepts_ptr, board_y_ - y, x, line_theta_) : InsertPointToMap(linepts_ptr, x, board_y_ - y, line_theta_);
					}
					pnt_cnt++;
				}
			}
		}
	}
	//std::cout << "the number of count is " << pnt_cnt << std::endl;
	//return pnt_cnt;
	
};*/

void BresenhaminMap(std::vector <LineSeg> Linetempsegs, float resolution_, int origin_x_, int origin_y_, int board_x_, int board_y_, std::vector<Eigen::Vector3f> * linepts_ptr)
{

	int pnt_cnt = 0;
	int line_number = Linetempsegs.size();
	for (int i = 0; i < line_number; i++)
	{
		double line_theta_ = Linetempsegs[i].theta_;
		double k_ = Linetempsegs[i].k_;
		float dy = Linetempsegs[i].y_end_ - Linetempsegs[i].y_start_;
		float dx = Linetempsegs[i].x_end_ - Linetempsegs[i].x_start_;
		dy = std::abs(dy);
		dx = std::abs(dx);
		int x1 = std::round(Linetempsegs[i].x_start_ / resolution_) + origin_x_;
		int x2 = std::round(Linetempsegs[i].x_end_ / resolution_) + origin_x_;
		int y1 = std::round(Linetempsegs[i].y_start_ / resolution_) + origin_y_;
		int y2 = std::round(Linetempsegs[i].y_end_ / resolution_) + origin_y_;
		float xs = Linetempsegs[i].x_start_;
		float xe = Linetempsegs[i].x_end_;
		float ys = Linetempsegs[i].y_start_;
		float ye = Linetempsegs[i].y_end_;
		//int sign1 = x2-x1?1:-1;
		//int sign2 = y2-y1?1:-1;
		//直线近似于垂直情况
		if (x1 == x2)
		{
			int y_min = y1 > y2 ? y2 : y1;
			int y_max = y1 > y2 ? y1 : y2;
			for (int i = y_min; i < y_max; i++)
			{
				int j = x1;
				//std::cout << " i= " << i << " j= " << j << std::endl;
				if ((i >= 0) && (j >= 0) && (i < board_x_) && (j < board_y_))
				{
					//setGridBel(j, i, 0);
					//注意Matrix的行列及方向和坐标系XY相反
					int x_ = j;
					int y_ = i;
					InsertPointToMap(linepts_ptr,x_,y_, line_theta_);
					//setGridBel(i, j, 0);
				}
				else
					continue;
				pnt_cnt++;
			}
		}
		//剩下的情况
		else {
			bool xySwap = false;//用来统计xy是否需要交换
			if (dx < dy)
			{
				xySwap = true;
			}
			//保证x的起始点永远小于终止点，是++
			if (x1 > x2) {
				std::swap(x1, x2);
				std::swap(y1, y2);
				std::swap(xs, xe);
				std::swap(ys, ye);
			}
			int increase = y1 <= y2 ? 1 : -1;
			//将起始点和终点加入ptr中
			if ((x1 >= 0) && (y1 >= 0) && (x1 < board_x_) && (y1 < board_y_))
			{
				 InsertPointToMap(linepts_ptr, x1, y1, line_theta_);
				 pnt_cnt++;
			}
			if ((x2 >= 0) && (y2 >= 0) && (x2 < board_x_) && (y2 < board_y_))
			{
				InsertPointToMap(linepts_ptr, x2, y2, line_theta_);
				pnt_cnt++;
			}
			if (xySwap == true)
			{
				float x = xs;
				float y = ys;
				if (increase == 1)
				{
					while (y < ye - resolution_ )
					{
						y += resolution_;
						x += 1 / k_* resolution_;
						int xp= std::round(x/ resolution_) + origin_x_;
						int yp = std::round(y / resolution_) + origin_y_;
						if ((xp >= 0) && (yp >= 0) && (xp < board_x_) && (yp < board_y_))
							InsertPointToMap(linepts_ptr, xp, yp, line_theta_);
						pnt_cnt++;
					}
				}
				if (increase == -1)
				{
					while (y > ye + resolution_)
					{
						y = y - resolution_;
						x -= 1 / k_* resolution_;
						int xp = std::round(x / resolution_) + origin_x_;
						int yp = std::round(y / resolution_) + origin_y_;
						if ((xp >= 0) && (yp >= 0) && (xp < board_x_) && (yp < board_y_))
							InsertPointToMap(linepts_ptr, xp, yp, line_theta_);
						pnt_cnt++;
					}
				}
			}
			if (xySwap == false)
			{
				float x = xs;
				float y = ys;
				while (x < xe - resolution_)
				{
					x += resolution_;
					y += k_* resolution_;
					int xp = std::round(x / resolution_) + origin_x_;
					int yp = std::round(y / resolution_) + origin_y_;
					if ((xp >= 0) && (yp >= 0) && (xp < board_x_) && (yp < board_y_))
						InsertPointToMap(linepts_ptr, xp, yp, line_theta_);
					pnt_cnt++;
				}
			}
			
			}
		}
	//std::cout << "the number of count is " << pnt_cnt << std::endl;
	//return pnt_cnt;
};

int MAP_INDEX(Mat &map, int i, int j)
{
	int size_x = map.cols;
	int size_y = map.rows;
	int index = i + j * size_x;
	return index;

}
 

void enqueue(Mat &map, unsigned int i, unsigned int j, float theta, std::queue<Eigen::Vector3f>& Q, unsigned char* marked)
{
	//std::cout << "i " << i << "j " << j << std::endl;
	if (marked[MAP_INDEX(map, i, j)])
		return;
	map.at<float>(j, i) = theta;
	Eigen::Vector3f new_q;
	new_q[0] = i;
	new_q[1] = j;
	new_q[2] = theta;
	Q.push(new_q);
	marked[MAP_INDEX(map, i, j)] = 1;
}

void update_theta_map(Mat &theta_map,std::vector <Eigen::Vector3f> * linepts_ptr)
{
	unsigned char* marked;
	std::queue<Eigen::Vector3f> Q;
	marked = new unsigned char[theta_map.cols*theta_map.rows];
	memset(marked, 0, sizeof(unsigned char) * theta_map.cols*theta_map.rows);	
	// Enqueue all the obstacle cells,将由Bresenham得到的线特征点的倾斜角全部放到队列Q中
	for (int num_ = 0; num_ < linepts_ptr->size();num_++)
	{
		Eigen::Vector3f cell_theta;
		float i = (*linepts_ptr)[num_](0);
		float j = (*linepts_ptr)[num_](1);
		float theta_ = (*linepts_ptr)[num_](2);
		cell_theta(0) = i;
		cell_theta(1) = j;
		cell_theta(2) = theta_;
		marked[MAP_INDEX(theta_map, i, j)] = 1;
		theta_map.at<float>(j, i) = theta_;
		Q.push(cell_theta);
	}
//使用BFS对theta_map上所有栅格进行扫描，将距离最近障碍的角度写到对应的栅格中
	while (!Q.empty())
	{
		Eigen::Vector3f current_cell = Q.front();

		//往上、下、左、右四个方向拓展
		int x_current = (int)current_cell(0);
		int y_current = (int)current_cell(1);
		//std::cout << "x_current " << x_current << "y_current " << y_current << std::endl;
		if (x_current > 0)
			enqueue(theta_map, current_cell(0) - 1, current_cell(1), current_cell(2),Q, marked);

		if (y_current > 0)
			enqueue(theta_map, current_cell(0), current_cell(1) - 1, current_cell(2), Q, marked);

		if (x_current < theta_map.cols - 1)
			enqueue(theta_map, current_cell(0) + 1, current_cell(1), current_cell(2), Q, marked);

		if (y_current < theta_map.rows - 1)
			enqueue(theta_map, current_cell(0), current_cell(1) + 1, current_cell(2), Q, marked);

		Q.pop();
	}

	delete[] marked;
}

int main(int argc, char *argv[]) {
    
    ros::init(argc, argv, "test_beihangtf_node");
    ros::NodeHandle nh;

    std::shared_ptr<PointSubscriber> point_sub_ptr = std::make_shared<PointSubscriber>(nh, "/scan_from_pointcloud", 1000);
    //std::shared_ptr<LineSubscriber> line_sub_ptr = std::make_shared<LineSubscriber>(nh, "current_scan", 100);
    //std::shared_ptr<TFListener> tf_to_odom_ptr = std::make_shared<TFListener>(nh, "odom", "base_link");

    //std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    //std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    //std::shared_ptr<LinePublisher> line_pub_str=std::make_shared<LinePublisher>(nh,"line_scan",200,"map");
    //std::shared_ptr<LineOnRvizPublisher> lineonviz_pub_ptr=std::make_shared<LineOnRvizPublisher>(nh,"line_segments",200,"map");
   // std::shared_ptr<NewTFListener> tf_to_odom_ptr = std::make_shared<NewTFListener>(nh, "odom", "base_link");
    g_odom_pub = nh.advertise<nav_msgs::Odometry>("new_odometry", 1000);
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
    Mat matchsrc(1000,1000, CV_8UC3, Scalar(255, 255, 255));
	Mat Img(1000,1000, CV_8UC3, Scalar(255, 255, 255));
	Mat Trajectory_map(1000,1000, CV_8UC3, Scalar(255, 255, 255));
    //two flag for test
	std::ofstream gt_,est_;
	gt_.open("./gtbeihang.txt", ios::out | ios::trunc);
	est_.open("./estbeihang.txt", ios::out | ios::trunc);
	//生成初始化CSM地图
	int origin_x_ = (board_xmax - board_xmin) / (2 * resolution_large_) ;
	int origin_y_ = (board_ymax - board_ymin) / (2 * resolution_large_) ;
	int size_x_ = (board_xmax - board_xmin) / resolution_large_ ;
	int size_y_ = (board_ymax - board_ymin) / resolution_large_ ;
	Mat cv_processedMap;
	Mat src(size_x_+1, size_y_+1, CV_8UC3, Scalar(255, 255, 255));
	Mat map_black(size_x_ + 1, size_y_ + 1, CV_8UC1, Scalar(255));
	Mat theta_map(size_x_ + 1, size_y_ + 1, CV_8UC1, Scalar(255));
	Mat theta_mapf;
	theta_map.convertTo(theta_mapf, CV_32FC1, 1, 0);
	//构建关于theta的score表，thetamapf和粒子对应的theta_的差在[0,pi],angle_step取值
	double angle_score[181];
	for (int i = 0; i < 181; i++)
	{
		//double score =std::exp(-(i * i) / (2*dis_sigma* dis_sigma));
		double score = pow(0.75, i);
		angle_score[i] = score;
	}
	//cnt：帧数计数器
    int cnt=0;
    ros::Rate rate(10);
    while (ros::ok()) 
    {
        ros::spinOnce();
        //clock_t start,finish;
        //double totaltime;
        //start=clock();
        //PointSet This_PData;
        /*std::cout<<postheta<<std::endl;
        std::cout<<posx<<std::endl;
        std::cout<<posy<<std::endl;*/
        point_sub_ptr->ParseData(point_data_buff);
        g_odom_sub = nh.subscribe ( "/odom", 1, odometryCallback );
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
					pose Old_PoseGT,New_PoseGT;
					pose rawpose_OldFrame, rawpose_NewFrame;
					pose Est_initpose;
					std::vector <LineSeg> Line_InOldFrame;
					//std::cout<<"Line_InOldFrame size: "<<Line_InOldFrame.size()<<std::endl;
                    //如果是初始帧，直接接受tf的pose
					if(cnt==0)
					{
						//Old_PoseGT.x=New_PoseGT.x= 0.698;
						Old_PoseGT.x=New_PoseGT.x=posx;
                        rawpose_OldFrame.x=rawpose_NewFrame.x=posx;
						//Old_PoseGT.y=New_PoseGT.y=-0.015;
						Old_PoseGT.y=New_PoseGT.y=posy;
                        rawpose_OldFrame.y=rawpose_NewFrame.y=posy;
						//Old_PoseGT.theta=New_PoseGT.theta=-0.463373;
						Old_PoseGT.theta=New_PoseGT.theta=postheta;
                        rawpose_OldFrame.theta=rawpose_NewFrame.theta=postheta;
                        std::cout<<"posx: "<<posx<<" posy: "<<posy<<" postheta: "<<postheta<<std::endl;

						//jilushuju
						gt_<<Old_PoseGT.x<<" "<<Old_PoseGT.y<<" "<<Old_PoseGT.theta<<"\n";
						est_<<Old_PoseGT.x<<" "<<Old_PoseGT.y<<" "<<Old_PoseGT.theta<<"\n";
					//开始初始化地图
					
                    for (int i=0;i<line_number;i++)
                    {
                        cv::Point2f pts,pte;//start & end point 
                        (*line_data.line_ptr)[i]=LineRotation(Old_PoseGT.x,Old_PoseGT.y,Old_PoseGT.theta,(*line_data.line_ptr)[i]);
                        generateGrad((*line_data.line_ptr)[i]);
						LineSeg Templine;
						Templine.x_start_=(*line_data.line_ptr)[i].start_point.x;
						Templine.y_start_=(*line_data.line_ptr)[i].start_point.y;
						Templine.x_end_=(*line_data.line_ptr)[i].end_point.x;
						Templine.y_end_=(*line_data.line_ptr)[i].end_point.y;
						Templine.k_=(*line_data.line_ptr)[i].k_;
						Templine.theta_=(*line_data.line_ptr)[i].theta_;
						Templine.b_=(*line_data.line_ptr)[i].b_;
						Line_InOldFrame.push_back(Templine);
					}
                        /*pts.x=(*line_data.line_ptr)[i].start_point.x*50+1000;
                        pts.y=(*line_data.line_ptr)[i].start_point.y*50+1500;
                        pte.x=(*line_data.line_ptr)[i].end_point.x*50+1000;
                        pte.y=(*line_data.line_ptr)[i].end_point.y*50+1500;
                        line(matchsrc, pts, pte, Scalar(0, 255, 0), 1,8,2);*/              
                      
					//MatrixInit(size_x_ + 1, size_y_ + 1);
						std::vector <Eigen::Vector3f> pt_old;
						std::vector <Eigen::Vector3f> * ptsold_ptr;
						ptsold_ptr = &pt_old;
						BresenhaminMap(Line_InOldFrame, resolution_large_, origin_x_, origin_y_, size_x_,size_y_,ptsold_ptr);
						update_theta_map(theta_mapf,ptsold_ptr);
						//std::cout<<"theta_map successfully!"<<std::endl;
						for (int i = 0; i < ptsold_ptr->size(); i++)
						{
							cv::Point2f p;
							p.x = (*ptsold_ptr)[i][0];
							p.y = (*ptsold_ptr)[i][1];
							circle(src, p, 1, Scalar(0, 255, 0));
							//将所有障碍物点填充至Map_bel中:
							map_black.at<uchar>(p.y, p.x) = 0;
						}
						map_black.convertTo(map_black, CV_8UC1);
						distanceTransform(map_black, cv_processedMap, CV_DIST_L2, 3);
						for (int i = 0; i < size_x_ + 1 ; i++)
						{
							for (int j = 0; j < size_y_+1 ; j++)
							{
								//cv_processedMap.at<float>(i,j)=1-cv_processedMap.at<float>(i,j)/10;
								//double z = cv_processedMap.at<float>(i, j);
								double z = cv_processedMap.at<float>(i, j);
								double p = pow(0.5,z);
								/*double p;
								if (z*z <= 2)
									p = 1;
								if ((z*z > 2) && (z*z <=4))
									p = 1.5;
								else if ((z*z > 4) && (z*z <= 8))
									p = 2;
								else if ((z*z >8) && (z*z <=13))
									p = 2.5;
								else if ((z >3) && (z < 3))
									p = 3;
								else
									p=z;
								cv_processedMap.at<float>(i, j) = std::exp(-(p*p)/(2*dis_sigma* dis_sigma));*/
								cv_processedMap.at<float>(i, j) = p;
								cv_processedMap.at<float>(i, j) = 255 * cv_processedMap.at<float>(i, j);
							}
						};
                        //imshow("processedmap",cv_processedMap);
                        //cv::waitKey(5);
						cv::Point pose_show;
						pose_show.x=New_PoseGT.x*50+500;
						pose_show.y=New_PoseGT.y*50+500;
						//circle(Trajectory_map, pose_show, 2, Scalar(0, 255, 0));
						//imshow("Trajectory",Trajectory_map);
						//cv::waitKey(1);
                        nav_msgs::Odometry odom;
                        odom.header.stamp = ros::Time::now();
                        odom.header.frame_id = "odom";
                        odom.child_frame_id = "base_link";
        
                        odom.pose.pose.position.x = posx;
                        odom.pose.pose.position.y = posy;
                        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(postheta);
                        odom.pose.pose.orientation = odom_quat;
        
                        odom.twist.twist.angular.z = 0;
                        odom.twist.twist.linear.x = 0;
                        odom.twist.twist.linear.y = 0;
        
                        g_odom_pub.publish(odom);
						std::cout<<"Frame number: "<<cnt<<std::endl;
						std::cout << "Final predict pose is " << " x: " << New_PoseGT.x << " y: " << New_PoseGT.y << " theta: " << New_PoseGT.theta << std::endl;
						Line_InOldFrame.clear();
					}
					//imshow("GT_MAP",matchsrc);
                    //cv::waitKey(1);

                    //从此处开始进行线段地图合并
                    /*if(cnt%5==0)
                    {
                        for(int i=0;i<Frame_Final.size();i++)
                        {
                            Point pts, pte;
		                    pts.x = Frame_Final[i].start_point.x * 50 + 1000;
		                    pts.y = Frame_Final[i].start_point.y * 50 + 1500;
		                    pte.x = Frame_Final[i].end_point.x * 50 + 1000;
		                    pte.y = Frame_Final[i].end_point.y * 50 + 1500;
		                    line(matchsrc, pts, pte, Scalar(255, 255, 0), 1, 8, 2);
                        }
                        imshow("GT_MAP",matchsrc);
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
                    }*/
                    else
                    {
						/*std::cout<<"The size of New Frame is "<<line_data.line_ptr->size()<<std::endl;
						std::cout<<"The size of Old Frame_Final is "<<Frame_Final.size()<<std::endl;
                        WhetherLineIsSame(line_data.line_ptr, ptr_final);
						std::cout<<"The size of Left Frame_New is "<<line_data.line_ptr->size()<<std::endl;
						if(line_data.line_ptr->size()>0)
						{
							for(int i=0;i<line_data.line_ptr->size();i++)
							{
								Frame_Final.push_back( (*line_data.line_ptr)[i]);
							}
						}*/
						gt_<<posx<<" "<<posy<<" "<<postheta<<"\n";
						rawpose_NewFrame.x=posx;
						rawpose_NewFrame.y=posy;
						rawpose_NewFrame.theta=postheta;
                        std::cout<<"posx: "<<posx<<" posy: "<<posy<<" postheta: "<<postheta<<std::endl;
						double delta_x = rawpose_NewFrame.x - rawpose_OldFrame.x;
						double delta_y = rawpose_NewFrame.y - rawpose_OldFrame.y;
						double delta_theta = rawpose_NewFrame.theta - rawpose_OldFrame.theta;

						Est_initpose.x = Old_PoseGT.x + delta_x;
						Est_initpose.y = Old_PoseGT.y + delta_y;
						Est_initpose.theta = Old_PoseGT.theta + delta_theta;
						double x_left = Est_initpose.x - SearchLength_;
						double x_right = Est_initpose.x + SearchLength_;
						double y_left = Est_initpose.y - SearchLength_;
						double y_right = Est_initpose.y + SearchLength_;
						double angle_step = PI / 180;
						pose pose_final;
						pose_final.x = 0;
						pose_final.y = 0;
						pose_final.theta = 0;
						double score_sum = 0.0;
						//std::cout<<"Is Dumped?"<<std::endl;
						if(line_number!=0)
						{
						//CSM粗匹配
						for (int k = 0; k < 361; k++)
						{
							double theta_ = -PI + k * angle_step;
							Eigen::Matrix3f Rot_;
							Rot_<< cos(theta_), -sin(theta_), 0,
									sin(theta_), cos(theta_), 0, 
									0, 0, 1;
							//std::cout<<"Rot_"<<Rot_<<std::endl;
							for (double tppose_x = x_left; tppose_x < x_right; tppose_x = tppose_x + resolution_large_)
							{
								for (double tppose_y = y_left; tppose_y < y_right; tppose_y = tppose_y + resolution_large_)
								{
								//开始将New_Frames的线特征按点匹配计分
									vector <LineSeg> Line_InNewFrame;
									//double newpt_score=0.0;
									double newpt_score=0;
									//将NewFrame中所有的线特征转到粒子对应的坐标系下
									for (int i = 0; i < line_number; i++)
									{
										LineSeg TempSeg;
										Eigen::Vector3f Trans;//平移
										Trans << tppose_x, tppose_y, 0;
										Eigen::Vector3f LineinR_Start, LineinR_End, LineinW_Start, LineinW_End;
										LineinR_Start<<(*line_data.line_ptr)[i].start_point.x,(*line_data.line_ptr)[i].start_point.y, 1;
										LineinR_End << (*line_data.line_ptr)[i].end_point.x, (*line_data.line_ptr)[i].end_point.y, 1;
										LineinW_Start = Rot_ * LineinR_Start + Trans;
										LineinW_End = Rot_ * LineinR_End + Trans;
										TempSeg.x_start_ = LineinW_Start[0];
										TempSeg.y_start_ = LineinW_Start[1];
										TempSeg.x_end_ = LineinW_End[0];
										TempSeg.y_end_ = LineinW_End[1];
										generateGradB(TempSeg);
										Line_InNewFrame.push_back(TempSeg);
									}
									std::vector <Eigen::Vector3f> pt_new;
									std::vector <Eigen::Vector3f> * ptsnew_ptr;
									ptsnew_ptr = &pt_new;
									//std::cout<<"Start Bresenham"<<std::endl;
									BresenhaminMap(Line_InNewFrame, resolution_large_, origin_x_, origin_y_, size_x_, size_y_, ptsnew_ptr);
									//std::cout<<"End Bresenham"<<std::endl;
									for (int i = 0; i < ptsnew_ptr->size(); i++)
									{
										int x_inpt = (*ptsnew_ptr)[i][0];
										int y_inpt = (*ptsnew_ptr)[i][1];
										double theta_inpt= (*ptsnew_ptr)[i][2];
										double dis_score = cv_processedMap.at<float>(y_inpt, x_inpt);
										double theta_inmp = theta_mapf.at<float>(y_inpt, x_inpt);
										int angle_num = round(std::abs(theta_inpt - theta_inmp) / angle_step);
										double theta_score = angle_score[angle_num];
										double score_pt = dis_score * theta_score;
										newpt_score = newpt_score + score_pt;
									}
									if (newpt_score > score_sum)
									{
										score_sum = newpt_score;
										pose_final.x = tppose_x;
										pose_final.y = tppose_y;
										pose_final.theta = theta_;
									}

								}
							}
						}
						std::vector <CSMCorrelatePose> last_set;
						Eigen::Matrix3f Rot_theta;
						Rot_theta<< cos(pose_final.theta), -sin(pose_final.theta), 0,
									sin(pose_final.theta), cos(pose_final.theta), 0, 
									0, 0, 1;
						std::cout<<"origin predict "<<pose_final.x<< " "<<pose_final.y<<" "<<pose_final.theta<<std::endl;
						for (double tppose_x = x_left; tppose_x< x_right; tppose_x = tppose_x + resolution_large_)
						{
							for (double tppose_y = y_left; tppose_y <y_right; tppose_y = tppose_y + resolution_large_)
							{
								//开始将New_Frames的线特征按点匹配计分
								vector <LineSeg> Line_InNewFrameFinal;
								//double newpt_score=0.0;
								double newpt_score=0.0;
								//将NewFrame中所有的线特征转到粒子对应的坐标系下
								for (int i = 0; i < line_number; i++)
								{
									LineSeg TempSegs;
									Eigen::Vector3f Trans;//平移
									Trans << tppose_x, tppose_y, 0;
									Eigen::Vector3f LineinR_Start, LineinR_End, LineinW_Start, LineinW_End;
									LineinR_Start<<(*line_data.line_ptr)[i].start_point.x,(*line_data.line_ptr)[i].start_point.y, 1;
									LineinR_End << (*line_data.line_ptr)[i].end_point.x, (*line_data.line_ptr)[i].end_point.y, 1;
									LineinW_Start = Rot_theta * LineinR_Start + Trans;
									LineinW_End = Rot_theta * LineinR_End + Trans;
									TempSegs.x_start_ = LineinW_Start[0];
									TempSegs.y_start_ = LineinW_Start[1];
									TempSegs.x_end_ = LineinW_End[0];
									TempSegs.y_end_ = LineinW_End[1];
									generateGradB(TempSegs);
									Line_InNewFrameFinal.push_back(TempSegs);
								}
								std::vector <Eigen::Vector3f> pt_new;
								std::vector <Eigen::Vector3f> * ptsnew_ptr;
								ptsnew_ptr = &pt_new;
								BresenhaminMap(Line_InNewFrameFinal, resolution_large_, origin_x_, origin_y_, size_x_, size_y_, ptsnew_ptr);
								for (int i = 0; i < ptsnew_ptr->size(); i++)
								{
									int x_inpt = (*ptsnew_ptr)[i][0];
									int y_inpt = (*ptsnew_ptr)[i][1];
									double theta_inpt= (*ptsnew_ptr)[i][2];
									double dis_score = cv_processedMap.at<float>(y_inpt, x_inpt);
									double theta_inmp = theta_mapf.at<float>(y_inpt, x_inpt);
									int angle_num = round(std::abs(theta_inpt - theta_inmp) / angle_step);
									double theta_score = angle_score[angle_num];
									double score_pt = dis_score * theta_score;
									newpt_score = newpt_score + score_pt;
								}
								CSMCorrelatePose posetmp_;
								posetmp_.x_=tppose_x;
								posetmp_.y_=tppose_y;
								posetmp_.theta_=pose_final.theta;
								posetmp_.score_=newpt_score;
								last_set.push_back(posetmp_);
							}
						}
						double score_sum_=0;
						for (int i=0;i<last_set.size();i++)	
						{
							score_sum_+=last_set[i].score_;
						}
						//std::cout<<score_sum_<<std::endl;
						Eigen::Matrix3f K;
						Eigen::Vector3f u;
						K<<0,0,0,0,0,0,0,0,0;
						u<<0,0,0;

						//std::cout<<"K:"<<std::endl;
						//std::cout<<"u: "<<std::endl;
						for (int i=0;i<last_set.size();i++)	
						{
							last_set[i].score_=last_set[i].score_/score_sum_;
							Eigen::Vector3f posei;
							posei<<last_set[i].x_,last_set[i].y_,last_set[i].theta_;
							K+=posei*posei.transpose()*last_set[i].score_;
							u+=posei*last_set[i].score_;
						}	
						Eigen::Matrix3f ConvMat;
						ConvMat=K-u*u.transpose();
						pose_final.x=u[0];
						pose_final.y=u[1];
						pose_final.theta=u[2];
						//std::cout<<"last predict pose"<<u<<std::endl;
						
						}
						if(line_number==0)
						{
							pose_final.x = Est_initpose.x;
							pose_final.y = Est_initpose.y;
							pose_final.theta = Est_initpose.theta;
							
							/*Eigen::Matrix3f ConvWorld=RotK*ConvRobo*RotK.transpose();
							Eigen::Vector3f pose_odom;
							pose_odom<<rawpose_NewFrame.x,rawpose_NewFrame.y,rawpose_NewFrame.theta;*/
						}
                        //std::cout<<"Frame number: "<<cnt<<std::endl;
						std::cout << "Final predict pose is " << " x: " << pose_final.x << " y: " << pose_final.y << " theta: " << pose_final.theta << std::endl;
                        //carculate COnv of odometry:
						est_<<pose_final.x<<" "<<pose_final.y<<" "<<pose_final.theta<<"\n";
                        nav_msgs::Odometry odom;
						odom.header.stamp = ros::Time::now();
                        odom.header.frame_id = "odom";
                        odom.child_frame_id = "base_link";
        
                        odom.pose.pose.position.x = pose_final.x;
                        odom.pose.pose.position.y = pose_final.y;
                        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_final.theta);
                        odom.pose.pose.orientation = odom_quat;
        
                        odom.twist.twist.angular.z = 0;
                        odom.twist.twist.linear.x = 0;
                        odom.twist.twist.linear.y = 0;
        
                        g_odom_pub.publish(odom);
						
						
						
						cv::Point begin_point,end_point;
                        begin_point.x=Old_PoseGT.x;
                        begin_point.y=Old_PoseGT.y;
                        end_point.x=pose_final.x;
                        end_point.y=pose_final.y;
                        //line(Trajectory_map, begin_point, end_point, Scalar(0), 5);
                        //imshow("Trajectory",Trajectory_map);
						//cv::waitKey(1);
						Old_PoseGT.x=rawpose_NewFrame.x;
						Old_PoseGT.y=rawpose_NewFrame.y;
						Old_PoseGT.theta=rawpose_NewFrame.theta;
						
						
						
						
						
						
						

						/*cv::Point pose_show;
						pose_show.x=Old_PoseGT.x*50+500;
						pose_show.y=Old_PoseGT.y*50+500;
						circle(Trajectory_map, pose_show, 2, Scalar(0, 255, 0));*/
						
						//map_black=Mat::zeros(size_x_ + 1, size_y_ + 1, CV_8UC1);
						if(line_number!=0)
						{
							
						for (int i=0;i<line_number;i++)
                    	{
                        	cv::Point2f pts,pte;//start & end point 
                        	(*line_data.line_ptr)[i]=LineRotation(Old_PoseGT.x,Old_PoseGT.y,Old_PoseGT.theta,(*line_data.line_ptr)[i]);
                        	generateGrad((*line_data.line_ptr)[i]);
							LineSeg Templine;
							Templine.x_start_=(*line_data.line_ptr)[i].start_point.x;
							Templine.y_start_=(*line_data.line_ptr)[i].start_point.y;
							Templine.x_end_=(*line_data.line_ptr)[i].end_point.x;
							Templine.y_end_=(*line_data.line_ptr)[i].end_point.y;
							Templine.k_=(*line_data.line_ptr)[i].k_;
							Templine.theta_=(*line_data.line_ptr)[i].theta_;
							Templine.b_=(*line_data.line_ptr)[i].b_;
							Line_InOldFrame.push_back(Templine);
						}
						//所有地图全部清空
						map_black=255*Mat::ones(size_x_ + 1, size_y_ + 1, CV_8UC1);
						theta_map=255*Mat::ones(size_x_ + 1, size_y_ + 1, CV_8UC1);
						theta_map.convertTo(theta_mapf, CV_32FC1, 1, 0);
						//cv_processedMap=Mat::zeros(size_x_ + 1, size_y_ + 1, CV_32FC1);
						std::vector <Eigen::Vector3f> pt_old;
						std::vector <Eigen::Vector3f> * ptsold_ptr;
						ptsold_ptr = &pt_old;
						BresenhaminMap(Line_InOldFrame, resolution_large_, origin_x_, origin_y_, size_x_,size_y_,ptsold_ptr);
						update_theta_map(theta_mapf,ptsold_ptr);
						//std::cout<<"theta_map successfully!"<<std::endl;
						for (int i = 0; i < ptsold_ptr->size(); i++)
						{
							cv::Point2f p;
							p.x = (*ptsold_ptr)[i][0];
							p.y = (*ptsold_ptr)[i][1];
							//circle(src, p, 1, Scalar(0, 255, 0));
							//将所有障碍物点填充至Map_bel中:
							map_black.at<uchar>(p.y, p.x) = 0;
						}
						map_black.convertTo(map_black, CV_8UC1);
						distanceTransform(map_black, cv_processedMap, CV_DIST_L2, 3);
						for (int i = 0; i < size_x_ + 1 ; i++)
						{
							for (int j = 0; j < size_y_+1 ; j++)
							{
								//cv_processedMap.at<float>(i,j)=1-cv_processedMap.at<float>(i,j)/10;
								double z = cv_processedMap.at<float>(i, j);
								double p=pow(0.5,z);
								/*double p;
								if (z*z <= 2)
									p = 1;
								if ((z*z > 2) && (z*z <=4))
									p = 1.5;
								else if ((z*z > 4) && (z*z <= 8))
									p = 2;
								else if ((z*z >8) && (z*z <=13))
									p = 2.5;
								else if ((z >3) && (z < 3))
									p = 3;
								else
									p=z;
								cv_processedMap.at<float>(i, j) = std::exp(-(p*p)/(2*dis_sigma* dis_sigma));*/
								cv_processedMap.at<float>(i, j) = p;
								cv_processedMap.at<float>(i, j) = 255 * cv_processedMap.at<float>(i, j);
							}
						};
						}
                        //imshow("processedmap",cv_processedMap);
                        //cv::waitKey(5);
						rawpose_OldFrame.x=rawpose_NewFrame.x;
						rawpose_OldFrame.y=rawpose_NewFrame.y;
						rawpose_OldFrame.theta=rawpose_NewFrame.theta;
                    }
                    
                }
				cnt++;
            };
           
            
        

        rate.sleep();
    }
	gt_.close();
	est_.close();
    return 0;
}