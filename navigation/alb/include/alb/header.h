#ifndef HEADER_H
#define HEADER_H

#include <stdio.h>
#include <vector>
#include <string>
#include <stdint.h>
#include<inttypes.h>
#include <map>

#include <iostream>
#include <fstream>
#include <sstream>



typedef  uint32_t uint32;
using namespace std;
namespace Header
{
struct Point 
{
  float x;
  float y;
  float z; 
};

struct Vector3
{
  float x;
  float y;
  float z;
};

struct Quaternion
{
  float x;
  float y;
  float z;
  float w;
};

struct Pose
{
  Point position;
  Quaternion orientation;
};

struct Header
{
  uint32 seq;
  //time stamp;
  string frame_id;
};

struct PoseStamped
{
  Header header;
  Pose pose;
};

struct Linear
{
  float x;
  float y;
  float z;
};

struct Angular
{
  float x;
  float y;
  float z;
};

struct Twist
{
  Linear linear;
  Angular angular;
};

struct PoseWithCovariance
{
  Pose pose;
  //float[36] covariance;
  vector<float> covariance;
};

struct TwistWithCovariance
{
  Twist twist;
  //float[36] covariance;
  vector<float> covariance;
};

struct Odometry
{
  Header header;
  PoseWithCovariance posewithCovariance;
  TwistWithCovariance twistWithCovariance;
};

struct Path
{
  Header header;
  //PoseStamped[]  poses;
  vector<PoseStamped> poses;
};

struct Position2DInt
{
  int x;
  int y;
};

struct MapMetaData
{
  float resolution;
  uint32   width;
  uint32   height;
  Pose  origin;
};

struct OccupancyGrid
{
  Header header;
  MapMetaData info;
  //int8[] data;
  vector<int8_t> data;
};

struct OccupancyGridUpdate
{
  Header header;
  int x;
  int y;
  uint32 width;
  uint32 height;
  //int[] data;
  vector<int> data;
};

struct LaserScan
{
  Header header;
  float  angle_min;
  float  angle_max;
  float  angle_increment;
  float  time_increment;
  float  scan_time;
  float  range_min;
  float  range_max;
//  float[] ranges;
  vector<float> ranges;
  //float[] intensities;
  vector<float> intensities;
};

struct ChannelFloat32
{
  string name;
//  float[] values;
  vector<float> values;
};

struct PointCloud
{
  Header header;
  //Point[] points;
  vector<Point> points;
  //ChannelFloat32[] channels; 
  vector<ChannelFloat32> channels;
};

//struct Point_tf
//{
//  string frame_id_;
//  Point[] point_tf;
//};

struct PointStamped
{
  Header header;
  Point  point;
};

struct Vector3Stamped
{
  Header header;
  Vector3 vector;
};

struct QuaternionStamped
{
  Header header;
  Quaternion quaternion;
};

struct Transform
{
  Vector3 translation;
  Quaternion rotation;
};

struct TransformStamped
{
  Header header;
  string child_frame_id;
  Transform transform;
};

struct Polygon
{
  vector<Point> points;
};

struct PolygonStamped
{
  Header header;
  Polygon polygon;
};

class ParameterReader
{
public:
    ParameterReader( string filename="/home/lixiaopeng/nav_work/costmap_2d/parameters.txt" )
    {
        ifstream fin( filename.c_str() );
        if (!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                // 以‘＃’开头的是注释
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};

}
#endif









