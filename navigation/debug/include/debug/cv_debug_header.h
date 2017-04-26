#ifndef DEBUG_CV_DEBUG_HEADER_H_
#define DEBUG_CV_DEBUG_HEADER_H_


#include <opencv/cv.h>
#include <opencv/cvaux.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

cv::Mat plotCharGridMap(unsigned char* tmp_map , unsigned int size_x, unsigned int size_y , std::string tmp_str)
{ 

    cv::Mat M_1=cv::Mat(size_y,size_x,CV_8UC1);
    memcpy(M_1.data,tmp_map,size_x*size_y*sizeof(unsigned char));
    flip(M_1,M_1,0);
    // 
    //cv::Mat M_1_b;
    //createOpenCVDebugMatForCostmap(M_1, M_1_b);
    // 3 channel Mat
    cv::Mat M_3;
    cv::cvtColor( M_1, M_3, CV_GRAY2RGB);
    //imshow(tmp_str.c_str(), M_3);
/*
    printf("[cv_debug_header.h] : cvWaitKey()\n");
    cvWaitKey();
    cvWaitKey();
    printf("[cv_debug_header.h] : cvWaitKey()\n");
*/
    return M_3;
}




void createOpenCVDebugMat(cv::Mat const& src, cv::Mat& dst)
{
    CV_Assert(src.type() == CV_8UC1);
    const int rows = src.rows;
    const int cols = src.cols;
    std::cout << rows << " " << cols << std::endl;
    dst.create(rows, cols, src.type());
    uchar *p;
    for (int i = 0; i < rows; i++)
    {
        p = dst.ptr<uchar>(i);
        for (int j = 0; j < cols; j++)
        {
            //保证映射后的坐标在原图像范围内
            if (j >= 0 && i >= 0 && j < cols && i < rows){
            	if(src.ptr<uchar>(i)[j]==100)//obstacle
            		p[j]=0;
            	else if(src.ptr<uchar>(i)[j]==0)
            		p[j]=255;
            	else //if(src.ptr<uchar>(i)[j]==-1)
            		p[j]=100;

            }
            //else std::cout << "x, y" << x << " " << y << std::endl;
        }
    }
}

void createOpenCVDebugMatForCostmap(cv::Mat const& src, cv::Mat& dst)
{
    CV_Assert(src.type() == CV_8UC1);
    const int rows = src.rows;
    const int cols = src.cols;
    std::cout << rows << " " << cols << std::endl;
    dst.create(rows, cols, src.type());
    uchar *p;
    for (int i = 0; i < rows; i++)
    {
        p = dst.ptr<uchar>(i);
        for (int j = 0; j < cols; j++)
        {
            //保证映射后的坐标在原图像范围内
            if (j >= 0 && i >= 0 && j < cols && i < rows){
            	p[j]=255-src.ptr<char>(i)[j];

            }
            //else std::cout << "x, y" << x << " " << y << std::endl;
        }
    }
}


void translateTransform(cv::Mat const& src, cv::Mat& dst, int dx, int dy)
{
    CV_Assert(src.type() == CV_8UC1);
    const int rows = src.rows;
    const int cols = src.cols;
    std::cout << rows << " " << cols << std::endl;
    dst.create(rows, cols, src.type());
    uchar *p;
    for (int i = 0; i < rows; i++)
    {
        p = dst.ptr<uchar>(i);
        for (int j = 0; j < cols; j++)
        {
            //平移后坐标映射到原图像
            int x = j+dx ;
            int y = i+dy ;
            //保证映射后的坐标在原图像范围内
            if (x >= 0 && y >= 0 && x < cols && y < rows){
            	p[j+0] = src.ptr<uchar>(y)[x+0];
                //p[3*j+0] = src.ptr<uchar>(y)[3*x+0];
                //p[3*j+1] = src.ptr<uchar>(y)[3*x+1];
                //p[3*j+2] = src.ptr<uchar>(y)[3*x+2];
            }
            //else std::cout << "x, y" << x << " " << y << std::endl;
        }
    }
}

void drawPose(cv::Mat& img_map, float x, float y, float yaw, int r, int g, int b, float origin_x, float origin_y, float res)
{
    cv::Point pt;
    pt.x = (x - origin_x) / res;
    pt.y = img_map.rows - (y - origin_y) / res;
    cv::circle(img_map, pt, 10, cv::Scalar(b,g,r));
}

void drawPointWithSize(cv::Mat& img_map, float x, float y, float yaw, int r, int g, int b, float origin_x, float origin_y, float res,unsigned int point_size)
{
    cv::Point pt;
    pt.x = (x - origin_x) / res;
    pt.y = img_map.rows - (y - origin_y) / res;
    cv::circle(img_map, pt, point_size, cv::Scalar(b,g,r));
}

#endif
