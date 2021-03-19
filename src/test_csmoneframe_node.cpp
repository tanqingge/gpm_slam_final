// CSMSingleFrame.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//根据CSM测试相邻帧间的线特征匹配误差

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <cmath>
#include <queue>
#include <fstream>
#include <time.h>

#define PI 3.1415926
#define MaxRange 10
#define MAXSEGMENTS 200
#define connect_th_ 0.10
#define min_distance_ 0.15
#define count_threhold_ 3
#define length_limit_ 0.1
#define MaxRange 10
#define factor 1.1
#define PI 3.1415926

using namespace cv;
using namespace std;

struct LineSeg
{
	double x_start_, y_start_;
	double x_end_, y_end_;
	double k_, b_, theta_;
};

struct pose
{
	double x;
	double y;
	double theta;
};

struct CSMCorrelatePose{
	double x_;
	double y_;
	double theta_;
	double score_;
};

struct SegmentStats {
	float x_start_, y_start_, x_end_, y_end_, xMean_, yMean_;
	int count_; //the horizontal length of the line
	double grad_;//gradient, dy/dx
	double invgrad_; //inv gradient, dx/dy
	float x_, y_, xy_, xx_, yy_; //for gradient stats
};

void GenerateCandidatePoint(const double a[180], vector <cv::Point2f> *p)
{
	int size_ = 180;
	double AngleStep = PI / size_;
	for (int i = 0; i < size_; i++)
	{
		if (a[i] < MaxRange)
		{
			double theta_ = AngleStep * i - PI / 2;
			cv::Point2f TemperPoint;
			TemperPoint.x = a[i] * cos(theta_);
			TemperPoint.y = a[i] * sin(theta_);
			p->push_back(TemperPoint);
		}

	}

}

  void LineDataInit(SegmentStats init_segs_[MAXSEGMENTS])
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

void FitLineWithPoint(vector <cv::Point2f> *p, SegmentStats init_segs_[MAXSEGMENTS])
{
	//clock_t startTime,endTime;
	//startTime = clock();
	int line_num_ = 0;
	int point_size = p->size();
	static int count = 0;
	//std::cout << "points size" << point_size << std::endl;
	//std::cout << "points size" << point_size << std::endl;
	for (int i = 0; i < point_size; i++)
	{
		int seg_state_ = 0;
		float x_temp_ = (*p)[i].x;
		float y_temp_ = (*p)[i].y;
		for (int j = 0; j < line_num_; j++)
		{
			double d_, err_;
			if (init_segs_[j].grad_ < DBL_MAX)
			{
				d_ = init_segs_[j].grad_*x_temp_ - y_temp_ + init_segs_[j].yMean_ - init_segs_[j].grad_*init_segs_[j].xMean_;
				err_ = abs(d_) / sqrt(init_segs_[j].grad_*init_segs_[j].grad_ + 1);
			}
			else
			{
				d_ = init_segs_[j].invgrad_*y_temp_ - x_temp_ + init_segs_[j].xMean_ - init_segs_[j].invgrad_*init_segs_[j].yMean_;
				err_ = abs(d_) / sqrt(init_segs_[j].invgrad_*init_segs_[j].invgrad_ + 1);
			}
			double d1_ = sqrt(pow(x_temp_ - init_segs_[j].x_end_, 2) + pow(y_temp_ - init_segs_[j].y_end_, 2));
			double d2_ = sqrt(pow(x_temp_ - init_segs_[j].x_start_, 2) + pow(y_temp_ - init_segs_[j].y_start_, 2));
			if ((err_ < connect_th_) && ((d1_ < min_distance_) || (d2_ < min_distance_)))
			{
				init_segs_[j].xMean_ = (init_segs_[j].count_*init_segs_[j].xMean_ + x_temp_) / (init_segs_[j].count_ + 1);
				init_segs_[j].yMean_ = (init_segs_[j].count_*init_segs_[j].yMean_ + y_temp_) / (init_segs_[j].count_ + 1);
				init_segs_[j].x_ += x_temp_;
				init_segs_[j].y_ += y_temp_;
				init_segs_[j].xx_ += pow(x_temp_, 2);
				init_segs_[j].yy_ += pow(y_temp_, 2);
				init_segs_[j].xy_ += x_temp_ * y_temp_;
				init_segs_[j].count_ += 1;
				seg_state_ += 1;
				init_segs_[j].grad_ = (init_segs_[j].xy_ - init_segs_[j].x_*init_segs_[j].y_ / init_segs_[j].count_) / (init_segs_[j].xx_ - init_segs_[j].x_*init_segs_[j].x_ / init_segs_[j].count_);
				if (init_segs_[j].grad_ == DBL_MAX)
					init_segs_[j].invgrad_ = (init_segs_[j].xy_ - init_segs_[j].x_*init_segs_[j].y_ / init_segs_[j].count_) / (init_segs_[j].yy_ - init_segs_[j].y_*init_segs_[j].y_ / init_segs_[j].count_);
				init_segs_[j].x_end_ = x_temp_;
				init_segs_[j].y_end_ = y_temp_;
			}
		}
		if ((seg_state_ == 0) && (line_num_ < MAXSEGMENTS))
		{
			init_segs_[line_num_].x_start_ = x_temp_;
			init_segs_[line_num_].y_start_ = y_temp_;
			init_segs_[line_num_].x_end_ = x_temp_;
			init_segs_[line_num_].y_end_ = y_temp_;
			init_segs_[line_num_].xMean_ = x_temp_;
			init_segs_[line_num_].yMean_ = y_temp_;
			init_segs_[line_num_].xx_ = pow(x_temp_, 2);
			init_segs_[line_num_].yy_ = pow(y_temp_, 2);
			init_segs_[line_num_].xy_ = x_temp_ * y_temp_;
			init_segs_[line_num_].count_ = 1;
			init_segs_[line_num_].x_ = x_temp_;
			init_segs_[line_num_].y_ = y_temp_;
			line_num_ += 1;
		}
	}
	count++;
};

void WhetherSegmentIsValid(vector <SegmentStats> *line_ptr, SegmentStats init_segs_[MAXSEGMENTS])
{
	SegmentStats segs_temp_;
	static int num__ = 0;
	for (int i = 0; i < MAXSEGMENTS; i++)
	{
		double length_ = sqrt(pow(init_segs_[i].x_end_ - init_segs_[i].x_start_, 2) + pow(init_segs_[i].y_end_ - init_segs_[i].y_start_, 2));
		//std::cout<< "i:"<<i<<"Length"<<length_<<std::endl;
		if ((init_segs_[i].count_ > count_threhold_) && (length_ >= length_limit_))
		{
			if (init_segs_[i].grad_ != DBL_MAX)
			{
				double x_mean_ = (init_segs_[i].x_start_ + init_segs_[i].x_end_) / 2;
				double y_mean_ = (init_segs_[i].y_start_ + init_segs_[i].y_end_) / 2;
				segs_temp_.x_start_ = x_mean_ - length_ / (2 * sqrt(pow(init_segs_[i].grad_, 2) + 1));
				segs_temp_.y_start_ = y_mean_ - init_segs_[i].grad_*length_ / (2 * sqrt(pow(init_segs_[i].grad_, 2) + 1));
				segs_temp_.x_end_ = x_mean_ + length_ / (2 * sqrt(pow(init_segs_[i].grad_, 2) + 1));
				segs_temp_.y_end_ = y_mean_ + init_segs_[i].grad_*length_ / (2 * sqrt(pow(init_segs_[i].grad_, 2) + 1));
			}
			else if ((init_segs_[i].grad_ == DBL_MAX) && (init_segs_[i].invgrad_ != 0))

			{
				double x_mean_ = (init_segs_[i].x_start_ + init_segs_[i].x_end_) / 2;
				double y_mean_ = (init_segs_[i].y_start_ + init_segs_[i].y_end_) / 2;
				segs_temp_.x_start_ = x_mean_ - init_segs_[i].invgrad_*length_ / (2 * sqrt(pow(init_segs_[i].invgrad_, 2) + 1));
				segs_temp_.y_start_ = y_mean_ - length_ / (2 * sqrt(pow(init_segs_[i].invgrad_, 2) + 1));
				segs_temp_.x_end_ = x_mean_ + init_segs_[i].invgrad_*length_ / (2 * sqrt(pow(init_segs_[i].invgrad_, 2) + 1));
				segs_temp_.y_end_ = y_mean_ + length_ / (2 * sqrt(pow(init_segs_[i].invgrad_, 2) + 1));

			}
			//new_line_data_.push_back(segs_temp_);
			/*segs_temp_.x_start_ = init_segs_[i].x_start_;
			segs_temp_.y_start_ = init_segs_[i].y_start_;
			segs_temp_.x_end_ = init_segs_[i].x_end_;
			segs_temp_.y_end_ = init_segs_[i].y_end_;*/
			line_ptr->push_back(segs_temp_);

		}



	}

}


LineSeg LineRotation(pose pose_, SegmentStats line_ptr)
{
	LineSeg AfterRSeg;
	AfterRSeg.x_start_ = line_ptr.x_start_ * cos(pose_.theta) - line_ptr.y_start_ * sin(pose_.theta) + pose_.x;
	AfterRSeg.y_start_ = line_ptr.y_start_ * cos(pose_.theta) + line_ptr.x_start_ * sin(pose_.theta) + pose_.y;
	AfterRSeg.x_end_ = line_ptr.x_end_ * cos(pose_.theta) - line_ptr.y_end_ * sin(pose_.theta) + pose_.x;
	AfterRSeg.y_end_ = line_ptr.y_end_ * cos(pose_.theta) + line_ptr.x_end_ * sin(pose_.theta) + pose_.y;
	return AfterRSeg;
};

void generateGrad(LineSeg & lineseg)
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

int main()
{
	std::vector <LineSeg> Frame_old;
	std::vector <LineSeg> Frame_New;
	pose Old_PoseGT;
	Old_PoseGT.theta= 2.77968;
	Old_PoseGT.x= 0.656165;
	Old_PoseGT.y= 0.0812728;
	pose New_PoseGT;
	//这是真实GroundTruth
	New_PoseGT.theta = 2.27307;
	New_PoseGT.x= 0.685387;
	New_PoseGT.y= 0.112968;

	pose Est_pose;
	Est_pose.x = 0;
	Est_pose.y = 0;
	Est_pose.theta = 0;
	//frontier激光器intel.bag只有180°前端的数据值

	double scan_old[180] = { 0.0 };
	double scan_new[180] = { 0.0 };
	//读进glf数据

	ifstream in1("/home/tanqingge/catkin_ws/src/gpm_slam/dataset/LaserScan/frame6.txt", ios::in);
	if (!in1)
	{
		cout << "could not open file" << endl;
	}
	else
	{
		for (int i = 0; i < 180; i++)
		{
			in1 >> scan_old[i];
		}
	}
	in1.close();

	ifstream in2("/home/tanqingge/catkin_ws/src/gpm_slam/dataset/LaserScan/frame7.txt", ios::in);
	if (!in2)
	{
		cout << "could not open file" << endl;
	}
	else
	{
		for (int i = 0; i < 180; i++)
		{
			in2 >> scan_new[i];
		}
	}
	in2.close();


	
	vector <cv::Point2f> ScanPoint_Old;
	vector <cv::Point2f> * ptr_pold;
	ptr_pold = &ScanPoint_Old;
	vector <cv::Point2f> ScanPoint_New;
	vector <cv::Point2f> * ptr_pnew;
	ptr_pnew = &ScanPoint_New;
	GenerateCandidatePoint(scan_old, ptr_pold);
	GenerateCandidatePoint(scan_new, ptr_pnew);
	SegmentStats init_segs_old[MAXSEGMENTS];
	LineDataInit(init_segs_old);
	FitLineWithPoint(ptr_pold, init_segs_old);
	vector <SegmentStats> valid_segs_old;
	vector <SegmentStats> *ptr_vlold;
	ptr_vlold = &valid_segs_old;
	std::cout << "Valid Line in Old Frame :" <<ptr_vlold->size()<<endl;
	WhetherSegmentIsValid(ptr_vlold, init_segs_old);
	SegmentStats init_segs_new[MAXSEGMENTS];
	LineDataInit(init_segs_new);
	FitLineWithPoint(ptr_pnew, init_segs_new);
	vector <SegmentStats> valid_segs_new;
	vector <SegmentStats> *ptr_vlnew;
	ptr_vlnew = &valid_segs_new;
	cout << "Valid Line in New Frame :" <<ptr_vlnew->size()<<std:: endl;
	WhetherSegmentIsValid(ptr_vlnew, init_segs_new);
	//debug 绘图示意
	Mat draw_map(1000, 1000, CV_8UC1, Scalar(255));
	//开始将Old Frame转到世界坐标系：
	vector < LineSeg> Line_InOldFrame;
	for (int i = 0; i < ptr_vlold->size(); i++)
	{
		LineSeg TempSeg;
		TempSeg = LineRotation(Old_PoseGT,(*ptr_vlold)[i]);
		generateGrad(TempSeg);
		Line_InOldFrame.push_back(TempSeg);
		std::cout << "The" << i << "Line in World coordinate" << std::endl;
		std::cout << "x_start " << TempSeg.x_start_ << std::endl;
		std::cout << "y_start " << TempSeg.y_start_ << std::endl;
		std::cout << "x_end " << TempSeg.x_end_ << std::endl;
		std::cout << "y_end " << TempSeg.y_end_ << std::endl;
		cv::Point2f beginPoint, endPoint;
		beginPoint.x = TempSeg.x_start_*20+500;
		beginPoint.y = TempSeg.y_start_*20+500;
		endPoint.x = TempSeg.x_end_*20+500;
		endPoint.y = TempSeg.y_end_*20+500;
		line(draw_map, beginPoint, endPoint, Scalar(0), 2);
	}
	std::cout << "Valid Line in Old Frame :" <<ptr_vlold->size()<<endl;
	std::cout << "Valid Line in New Frame :" <<ptr_vlnew->size()<<std:: endl;
	//debug:新frame下robot坐标系line坐标
	for (int i = 0; i < ptr_vlnew->size(); i++)
	{
		std::cout << "The" << i << "New  in Robot coordinate" << std::endl;
		std::cout << "x_start " << (*ptr_vlnew)[i].x_start_ << std::endl;
		std::cout << "y_start " << (*ptr_vlnew)[i].y_start_ << std::endl;
		std::cout << "x_end " << (*ptr_vlnew)[i].x_end_ << std::endl;
		std::cout << "y_end " << (*ptr_vlnew)[i].y_end_ << std::endl;
	}
	
	//生成GridMap：
	float resolution_large_= 0.1;
	float board_xmin = -30;
	float board_xmax = 30;
	float board_ymin = - 30;
	float board_ymax = 30;
	double dis_sigma = 1.0;
	double theta_sigma = 0.5;
	//在计算坐标时应当考虑到Mat是从0开始的。x:[0,600],y:[0,600]
	int origin_x_ = (board_xmax - board_xmin) / 2/resolution_large_ ;
	int origin_y_ = (board_ymax - board_ymin) / 2 / resolution_large_ ;
	int size_x_ = (board_xmax - board_xmin) / resolution_large_ ;
	int size_y_ = (board_ymax - board_ymin) / resolution_large_ ;
	Mat cv_processedMap;
	Mat src(size_x_+1, size_y_+1, CV_8UC3, Scalar(255, 255, 255));
	Mat map_black(size_x_ + 1, size_y_ + 1, CV_8UC1, Scalar(255));
	Mat theta_map(size_x_ + 1, size_y_ + 1, CV_8UC1, Scalar(255));
	Mat theta_mapf;
	theta_map.convertTo(theta_mapf, CV_32FC1, 1, 0);
	//MatrixInit(size_x_ + 1, size_y_ + 1);
	std::vector <Eigen::Vector3f> pt_old;
	std::vector <Eigen::Vector3f> * ptsold_ptr;
	ptsold_ptr = &pt_old;
	BresenhaminMap(Line_InOldFrame, resolution_large_, origin_x_, origin_y_, size_x_,size_y_,ptsold_ptr);
	update_theta_map(theta_mapf,ptsold_ptr);
	std::cout<<"theta_map successfully!"<<std::endl;
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
			double z = cv_processedMap.at<float>(i, j);
			double p = pow(0.5,z);
			/*double p;
			if (z*z < 2)
				p = 1;
			else if ((z*z > 3) && (z*z < 5))
				p = 1.5;
			else if ((z*z >= 5) && (z < 2.5))
				p = 2;
			else if ((z >= 2.5) && (z < 3))
				p = 3;
			else
				p=z;
				
			cv_processedMap.at<float>(i, j) = std::exp(-(z*z)/(2* dis_sigma* dis_sigma));*/
			cv_processedMap.at<float>(i, j) = p;
			cv_processedMap.at<float>(i, j) = 255 * cv_processedMap.at<float>(i, j);
		}
	};
	
	pose rawpose_OldFrame;
	rawpose_OldFrame.theta =2.63029;
	rawpose_OldFrame.x = 0.734;
	rawpose_OldFrame.y = 0.037;
	pose rawpose_NewFrame;
	rawpose_NewFrame.theta =2.10177;
	rawpose_NewFrame.x =  0.737;
	rawpose_NewFrame.y =  0.035;
	double delta_x = rawpose_NewFrame.x - rawpose_OldFrame.x;
	double delta_y = rawpose_NewFrame.y - rawpose_OldFrame.y;
	double delta_theta = rawpose_NewFrame.theta - rawpose_OldFrame.theta;
	double SearchLength_ = 0.5;
	pose Est_initpose;
	Est_initpose.x = Old_PoseGT.x + delta_x;
	Est_initpose.y = Old_PoseGT.y + delta_y;
	Est_initpose.theta = Old_PoseGT.theta + delta_theta;
	double x_left = Est_initpose.x - SearchLength_;
	double x_right = Est_initpose.x + SearchLength_;
	double y_left = Est_initpose.y - SearchLength_;
	double y_right = Est_initpose.y + SearchLength_;
	double angle_step = PI / 180;
	//构建关于theta的score表，thetamapf和粒子对应的theta_的差在[0,pi],angle_step取值
	double angle_score[181];
	for (int i = 0; i < 181; i++)
	{
		//double score =std::exp(-(i * i) / (2 * dis_sigma* dis_sigma));
		double score = pow(0.75, i);
		angle_score[i] = score;
	}
	//计时:
	clock_t start_time, end_time;
	start_time = clock();
	//CSM粗扫描
	pose pose_final;
	pose_final.x = 0;
	pose_final.y = 0;
	pose_final.theta = 0;
	double score_sum = 0.0;
	//ofstream outfile("pose.txt",std::ios::app);
	for (int k = 0; k < 361; k++)
	{
		double theta_ = -PI + k * angle_step;
		Eigen::Matrix3f Rot_;
		Rot_<< cos(theta_), -sin(theta_), 0,
				sin(theta_), cos(theta_), 0, 
				0, 0, 1;
		for (double tppose_x = x_left; tppose_x < x_right; tppose_x = tppose_x + resolution_large_)
		{
			for (double tppose_y = y_left; tppose_y < y_right; tppose_y = tppose_y + resolution_large_)
			{
				//开始将New_Frames的线特征按点匹配计分
				vector <LineSeg> Line_InNewFrame;
				double newpt_score=0.0;
				//将NewFrame中所有的线特征转到粒子对应的坐标系下
				for (int i = 0; i < ptr_vlnew->size(); i++)
				{
					LineSeg TempSeg;
					Eigen::Vector3f Trans;//平移
					Trans << tppose_x, tppose_y, 0;
					Eigen::Vector3f LineinR_Start, LineinR_End, LineinW_Start, LineinW_End;
					LineinR_Start<<(*ptr_vlnew)[i].x_start_,(*ptr_vlnew)[i].y_start_, 1;
					LineinR_End << (*ptr_vlnew)[i].x_end_, (*ptr_vlnew)[i].y_end_, 1;
					LineinW_Start = Rot_ * LineinR_Start + Trans;
					LineinW_End = Rot_ * LineinR_End + Trans;
					TempSeg.x_start_ = LineinW_Start[0];
					TempSeg.y_start_ = LineinW_Start[1];
					TempSeg.x_end_ = LineinW_End[0];
					TempSeg.y_end_ = LineinW_End[1];
					generateGrad(TempSeg);
					Line_InNewFrame.push_back(TempSeg);
				}
				std::vector <Eigen::Vector3f> pt_new;
				std::vector <Eigen::Vector3f> * ptsnew_ptr;
				ptsnew_ptr = &pt_new;
				BresenhaminMap(Line_InNewFrame, resolution_large_, origin_x_, origin_y_, size_x_, size_y_, ptsnew_ptr);
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
				//outfile << "particle info is" <<"\n";
				//outfile << "x :" << tppose_x << " y: " << tppose_y << " theta: " << theta_ << " score: " << newpt_score << "\n";
				//std::cout << " particle info is " << std::endl;
				//std::cout << "x :" << tppose_x << " y: " << tppose_y << " theta: " << theta_ <<" score: "<< newpt_score<<std::endl;
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
	end_time = clock();   //结束时间
	cout << "time = " << double(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;  //输出时间（单位：ｓ）
	//outfile.close();
	//std::cout << "Final predict pose is " << " x: " << pose_final.x << " y: " << pose_final.y << " theta: " << pose_final.theta << " score： "<< score_sum<<std::endl;

	//测试New-GT的score
	//开始将New_Frames的线特征按点匹配计分
	//vector <LineSeg> Line_InNewFrame;
	//double newpt_score = 0.0;
	
	
	//将NewFrame中所有的线特征转到粒子对应的坐标系下
	/*Eigen::Matrix3f Rot_;
	Rot_ << cos(New_PoseGT.theta), -sin(New_PoseGT.theta), 0,
		sin(New_PoseGT.theta), cos(New_PoseGT.theta), 0,
		0, 0, 1;
	//std::cout << "Rot_: " << Rot_ << std::endl;
	for (int i = 0; i < ptr_vlnew->size(); i++)
	{
		LineSeg TempSeg;
		Eigen::Vector3f Trans;//平移
		Trans << New_PoseGT.x, New_PoseGT.y, 0;
		Eigen::Vector3f LineinR_Start, LineinR_End, LineinW_Start, LineinW_End;
		LineinR_Start << (*ptr_vlnew)[i].x_start_, (*ptr_vlnew)[i].y_start_, 1;
		LineinR_End << (*ptr_vlnew)[i].x_end_, (*ptr_vlnew)[i].y_end_, 1;
		LineinW_Start = Rot_ * LineinR_Start + Trans;
		LineinW_End = Rot_ * LineinR_End + Trans;
		TempSeg.x_start_ = LineinW_Start[0];
		TempSeg.y_start_ = LineinW_Start[1];
		TempSeg.x_end_ = LineinW_End[0];
		TempSeg.y_end_ = LineinW_End[1];
		generateGrad(TempSeg);
		Line_InNewFrame.push_back(TempSeg);
		cv::Point2f beginPoint, endPoint;
		beginPoint.x = TempSeg.x_start_ * 20 + 500;
		beginPoint.y = TempSeg.y_start_ * 20 + 500;
		endPoint.x = TempSeg.x_end_ * 20 + 500;
		endPoint.y = TempSeg.y_end_ * 20 + 500;
		line(draw_map, beginPoint, endPoint, Scalar(120), 2);
	}
	std::vector <Eigen::Vector3f> pt_new;
	std::vector <Eigen::Vector3f> * ptsnew_ptr;
	ptsnew_ptr = &pt_new;
	BresenhaminMap(Line_InNewFrame, resolution_large_, origin_x_, origin_y_, size_x_, size_y_, ptsnew_ptr);
	for (int i = 0; i < ptsnew_ptr->size(); i++)
	{
		int x_inpt = (*ptsnew_ptr)[i][0];
		int y_inpt = (*ptsnew_ptr)[i][1];
		double theta_inpt = (*ptsnew_ptr)[i][2];
		double dis_score = cv_processedMap.at<float>(y_inpt, x_inpt);
		double theta_inmp = theta_mapf.at<float>(y_inpt, x_inpt);
		//std::cout << "theta_inpt: " << theta_inpt << std::endl;
		//std::cout << "theta_inmp: " << theta_inmp << std::endl;
		int angle_num = round(std::abs(theta_inpt - theta_inmp) / angle_step);
		double theta_score = angle_score[angle_num];
		double score_pt = dis_score * theta_score;
		//std::cout << "i: " << i << " x: " << x_inpt << " y: " << y_inpt << std::endl;
		//std::cout<< " score: " << dis_score << " " << theta_score << " " << score_pt << std::endl;
		newpt_score = newpt_score + score_pt;
	}*/
	std::cout << " particle info is " << std::endl;
	std::cout << "x :" << New_PoseGT.x << " y: " << New_PoseGT.y << " theta: " << New_PoseGT.theta << std::endl;
	//std::cout << "Bresenham Points" <<ptsnew_ptr->size() << std::endl;
	
	/*pose New_PoseEST;
	New_PoseEST.x = pose_final.x;
	New_PoseEST.y = pose_final.y;
	New_PoseEST.theta = pose_final.theta;
	vector <LineSeg> ESTLine_InNewFrame;*/
	std::cout << "First predict pose is " << " x: " << pose_final.x << " y: " << pose_final.y << " theta: " << pose_final.theta <<std::endl;
	//将NewFrame中所有的线特征转到粒子对应的坐标系下
	/*Eigen::Matrix3f RotEST_;
	RotEST_ << cos(New_PoseEST.theta), -sin(New_PoseEST.theta), 0,
		sin(New_PoseEST.theta), cos(New_PoseEST.theta), 0,
		0, 0, 1;
	//std::cout << "RotEST_: " << RotEST_ << std::endl;
	for (int i = 0; i < ptr_vlnew->size(); i++)
	{
		LineSeg TempSeg;
		Eigen::Vector3f TransEST;//平移
		TransEST << New_PoseEST.x, New_PoseEST.y, 0;
		Eigen::Vector3f LineinR_Start, LineinR_End, LineinW_Start, LineinW_End;
		LineinR_Start << (*ptr_vlnew)[i].x_start_, (*ptr_vlnew)[i].y_start_, 1;
		LineinR_End << (*ptr_vlnew)[i].x_end_, (*ptr_vlnew)[i].y_end_, 1;
		LineinW_Start = RotEST_ * LineinR_Start + TransEST;
		LineinW_End = RotEST_ * LineinR_End + TransEST;
		TempSeg.x_start_ = LineinW_Start[0];
		TempSeg.y_start_ = LineinW_Start[1];
		TempSeg.x_end_ = LineinW_End[0];
		TempSeg.y_end_ = LineinW_End[1];
		generateGrad(TempSeg);
		ESTLine_InNewFrame.push_back(TempSeg);
		cv::Point2f beginPoint, endPoint;
		beginPoint.x = TempSeg.x_start_ * 20 + 500;
		beginPoint.y = TempSeg.y_start_ * 20 + 500;
		endPoint.x = TempSeg.x_end_ * 20 + 500;
		endPoint.y = TempSeg.y_end_ * 20 + 500;
		line(draw_map, beginPoint, endPoint, Scalar(60), 2);
	}
	std::vector <Eigen::Vector3f> pt_newest;
	std::vector <Eigen::Vector3f> * ptsnewest_ptr;
	ptsnewest_ptr = &pt_newest;
	BresenhaminMap(ESTLine_InNewFrame, resolution_large_, origin_x_, origin_y_, size_x_, size_y_, ptsnewest_ptr);
	for (int i = 0; i < ptsnewest_ptr->size(); i++)
	{
		int x_inpt = (*ptsnewest_ptr)[i][0];
		int y_inpt = (*ptsnewest_ptr)[i][1];
		double theta_inpt = (*ptsnewest_ptr)[i][2];
		double dis_score = cv_processedMap.at<float>(y_inpt, x_inpt);
		double theta_inmp = theta_mapf.at<float>(y_inpt, x_inpt);
		//std::cout << "theta_inpt: " << theta_inpt << std::endl;
		//std::cout << "theta_inmp: " << theta_inmp << std::endl;
		int angle_num = round(std::abs(theta_inpt - theta_inmp) / angle_step);
		double theta_score = angle_score[angle_num];
		double score_pt = dis_score * theta_score;
		//std::cout << "i: " << i << " x: " << x_inpt << " y: " << y_inpt << std::endl;
		//std::cout<< " score: " << dis_score << " " << theta_score << " " << score_pt << std::endl;
		newpt_score = newpt_score + score_pt;
	}
	imshow("processedMap",map_black);
	cv::imshow("line_Map", draw_map);
	waitKey(0);*/
	std::vector <CSMCorrelatePose> last_set;
	Eigen::Matrix3f Rot_theta;
	Rot_theta<< cos(pose_final.theta), -sin(pose_final.theta), 0,
				sin(pose_final.theta), cos(pose_final.theta), 0, 
				0, 0, 1;
	std::cout<<pose_final.theta;
	for (double tppose_x = pose_final.x-SearchLength_; tppose_x < pose_final.x+SearchLength_; tppose_x = tppose_x + resolution_large_/2)
		{
			for (double tppose_y =pose_final.y-SearchLength_; tppose_y <pose_final.y+SearchLength_; tppose_y = tppose_y + resolution_large_/2)
			{
				//开始将New_Frames的线特征按点匹配计分
				vector <LineSeg> Line_InNewFrame;
				double newpt_score=0.0;
				//将NewFrame中所有的线特征转到粒子对应的坐标系下
				for (int i = 0; i < ptr_vlnew->size(); i++)
				{
					LineSeg TempSeg;
					Eigen::Vector3f Trans;//平移
					Trans << tppose_x, tppose_y, 0;
					Eigen::Vector3f LineinR_Start, LineinR_End, LineinW_Start, LineinW_End;
					LineinR_Start<<(*ptr_vlnew)[i].x_start_,(*ptr_vlnew)[i].y_start_, 1;
					LineinR_End << (*ptr_vlnew)[i].x_end_, (*ptr_vlnew)[i].y_end_, 1;
					LineinW_Start = Rot_theta * LineinR_Start + Trans;
					LineinW_End = Rot_theta * LineinR_End + Trans;
					TempSeg.x_start_ = LineinW_Start[0];
					TempSeg.y_start_ = LineinW_Start[1];
					TempSeg.x_end_ = LineinW_End[0];
					TempSeg.y_end_ = LineinW_End[1];
					generateGrad(TempSeg);
					Line_InNewFrame.push_back(TempSeg);
				}
				std::vector <Eigen::Vector3f> pt_new;
				std::vector <Eigen::Vector3f> * ptsnew_ptr;
				ptsnew_ptr = &pt_new;
				BresenhaminMap(Line_InNewFrame, resolution_large_, origin_x_, origin_y_, size_x_, size_y_, ptsnew_ptr);
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
		std::cout<<score_sum_<<std::endl;
		Eigen::Matrix3f K;
		Eigen::Vector3f u;
		K<<0,0,0,0,0,0,0,0,0;
		u<<0,0,0;

		std::cout<<"K:"<<std::endl;
		std::cout<<"u: "<<std::endl;
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
		std::cout<<"last predict pose"<<u<<std::endl;
		std::cout<<"Conv Matrix"<<ConvMat<<std::endl;	
	return 0;
}


