#include "gridmap/gridmap.hpp"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>


namespace gpm_slam
{
    GridMap::GridMap(const double& resolution,const int& map_width,const int& map_hight):
    resolution_(resolution),map_width_(map_width),map_height_(map_hight)
    {
        size_x_=map_width_/resolution_+1;
        size_y_=map_height_/resolution_+1;
        Map_bel_.resize(size_x_,size_y_);
        Map_bel_=Eigen::MatrixXd::Constant(size_x_,size_y_,1);
        init_x_=size_x_/2;
        init_y_=size_y_/2;
        std::cout<<"resolution is "<<resolution_<<std::endl;
        std::cout<<"size_x is "<<size_x_<<" size_y is "<<size_y_<<std::endl;
        std::cout<<"init_x is "<<init_x_<<" init_y is "<<init_y_<<std::endl;
    }

    void GridMap::setGridBel(int idx,int idy, int val)
    {
        if(idx>=0&&idx<size_x_&&idy>=0&&idy<size_y_)
        {
            Map_bel_(idx,idy)=val;
        }
        
    }
    int GridMap::getGridBel(int idx,int idy)
    {
        int Grid_data=Map_bel_(idx,idy);
        return Grid_data;
    };

    //用来记录包含的直线特征穿过了矩阵中的多少个元素，穿过的矩阵元素记为1，返回的int类型为矩阵中不为0的元素数量;   
    int GridMap::BresenhaminMap(LineData::LINE* line_ptr)
    {
        int pnt_cnt=0;
        int line_number = line_ptr->size();
        for(int i=0; i<line_number;i++)
        {
            float dy=(*line_ptr)[i].end_point.y-(*line_ptr)[i].start_point.y;
            float dx=(*line_ptr)[i].end_point.x-(*line_ptr)[i].start_point.x;
            dy=(int)std::abs(dy);
            dx=(int)std::abs(dx);
            int x1=std::round((*line_ptr)[i].start_point.x/resolution_);
            int x2=std::round((*line_ptr)[i].end_point.x/resolution_);
            int y1=std::round((*line_ptr)[i].start_point.y/resolution_);
            int y2=std::round((*line_ptr)[i].end_point.y/resolution_);
            /*int sign1 = x2-x1?1:-1;
            int sign2 = y2-y1?1:-1;*/
            //直线近似于垂直情况
            if(x1==x2)
            {
                int y_min=y1>y2?y1:y2;
                int y_max=y1>y2?y2:y1;
                for(int i=(y_min+init_y_);i<(y_max+init_y_);i++)
                {
                    int j= x1 + init_x_;
                    if((i>=0)&&(j>=0))
                        //setGridBel(j,i,0);
                        setGridBel(i,j,0);
                    pnt_cnt++;
                }                
            }
            //剩下的情况
            else{
            bool interchange=false;//用来统计xy是否需要交换
            if(dx<dy)
            {
                interchange=true;
                std::swap(x1,y1);
                std::swap(x2,y2);
            }
            //保证x的起始点永远小于终止点，是++
            if(x1>x2){ 
                std::swap(x1,x2);
                std::swap(y1,y2);
            }
            int increase=y1<y2?1:-1;
            int r1=2*dy;
            int r2=r1-2*dx;
            int p=r1 -dx;
            //interchange?setGridBel(y1+init_x_,x1+init_y_,0):setGridBel(x1+init_x_,y1+init_y_,0);
            interchange?setGridBel(x1+init_y_,y1+init_x_,0):setGridBel(y1+init_y_,x1+init_x_,0);
            int x=x1,y=y1;
            int x_,y_;
            while(x<x2)
            {
                x++;
                if(p>0)
                {
                    p = p + r2;
                    y = y + increase;
                    //interchange?setGridBel(y+init_x_,x+init_y_,0):setGridBel(x+init_x_,y+init_y_,0);
                    interchange?setGridBel(x+init_y_,y+init_x_,0):setGridBel(y+init_y_,x+init_x_,0);
                    pnt_cnt++;
                }
                else
                {
                    p += r1;
                    //interchange?setGridBel(y+init_x_,x+init_y_,0):setGridBel(x+init_x_,y+init_y_,0);
                    interchange?setGridBel(x+init_y_,y+init_x_,0):setGridBel(y+init_y_,x+init_x_,0);
                    pnt_cnt++;
                }
            }
            }
        }
        //std::cout<<"the number of count is "<<pnt_cnt<<std::endl;
    };

    void GridMap::MapInit(LineData::LINE* line_ptr)
    {
        //计数器:
        static int matrix_id=0;
        cv::Mat Cv_Map_,cv_processedMap;
        BresenhaminMap(line_ptr);
        cv::eigen2cv(Map_bel_,Cv_Map_);
        //std::cout<<"eigen trans to Mat success"<<std::endl;
        
        Cv_Map_.convertTo(Cv_Map_,CV_8UC1);
        //std::cout<<"the type of Cv_Map_ is "<< Cv_Map_.type()<<std::endl; 
        cv::distanceTransform(Cv_Map_,cv_processedMap,CV_DIST_L2,CV_DIST_MASK_PRECISE);
        /*cv::imshow("figure",cv_processedMap);
        cv::waitKey(20);
        std::cout<<"the type of Cv_processedMap is "<< cv_processedMap.type()<<std::endl;*/
        //cv::distanceTransform(Cv_Map_,cv_processedMap,CV_DIST_L2,3);
        for(int i=0;i<size_x_;i++)
        {
            for(int j=0;j<size_y_;j++)
            {
                //cv_processedMap.at<float>(i,j)=1-cv_processedMap.at<float>(i,j)/10;
                cv_processedMap.at<float>(i,j)=std::exp(-cv_processedMap.at<float>(i,j));
                cv_processedMap.at<float>(i,j)=100*cv_processedMap.at<float>(i,j);
            }
        }
        
        //判断matrix生成是否正确:
        /*cv::imshow("figure",cv_processedMap);
        cv::waitKey(10000);
        std::cout<<"the type of Cv_processedMap is "<< cv_processedMap.type()<<std::endl;*/
        cv::cv2eigen(cv_processedMap,Map_bel_);
        //std::ofstream outfile;
        //std::cout<<"let record!\n";
        /*outfile.open("/home/tanqingge/catkin_ws/src/gpm_slam/exp_data/matrix.txt",std::ios::out|std::ios::app);
        if(!outfile.is_open())
        {
            std::cout << "can not open file ";
		    exit(EXIT_FAILURE);
        }
        outfile<<'The matrix id is'<<matrix_id<<'\n';
        for(int i=0;i<size_x_;i++)
        {
            for(int j=0;j<size_y_;j++)
            {
               outfile<<Map_bel_(i,j)<<' ';
            }
            outfile<<'\n';
        }*/
        matrix_id++;
        //outfile.close();
    }
    
    void GridMap::MapUpdate(LineData::LINE* line_ptr)
    {
        Eigen::MatrixXd Old_Map_;
        Old_Map_.resize(size_x_,size_y_);
        Old_Map_=Map_bel_;
        MapInit(line_ptr);
        Map_bel_=Map_bel_*0.2+Old_Map_*0.8;
    }


    Eigen::Vector2i GridMap::GetGridId()
        {
        };

    void GridMap::SetlineinMap()
        {
        };

}