/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#include <navfn/navfn_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <pcl_conversions/pcl_conversions.h>
#include <debug/cv_debug_header.h>
#include <alb/header.h>
#include <limits>


//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_DECLARE_CLASS(navfn, NavfnROS, navfn::NavfnROS, nav_core::BaseGlobalPlanner)

namespace navfn {

  NavfnROS::NavfnROS() 
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {}

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap_ros);
  }

  NavfnROS::NavfnROS(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame)
    : costmap_(NULL),  planner_(), initialized_(false), allow_unknown_(true) {
      //initialize the planner
      initialize(name, costmap, global_frame);
  }
//ALB
  void NavfnROS::initializeNoRos(costmap_2d::Costmap2D* costmap){
      printf("ZACH DEBUG: NavfnRos initializeNoRos\n");
      costmap_=costmap;      	 
      planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

#if 0
      private_nh.param("allow_unknown", allow_unknown_, true);
      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);
      private_nh.param("default_tolerance", default_tolerance_, 0.0);
#endif
      initialized_ = true;
      printf("ZACH DEBUG: NavfnRos initializeNoRos END\n");

}  
//ALB END 
  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string global_frame){
    if(!initialized_){
      costmap_ = costmap;
////////////////////////////////////////////////////////////////////////
      ROS_INFO("ZACH DEBUG: navfn planner initial start\n");
      unsigned int a = costmap->getSizeInCellsX();
      unsigned int b = costmap->getSizeInCellsY();
      double res = costmap->getResolution();
      double origin_x = costmap->getOriginX();
      double origin_y = costmap->getOriginY();
      unsigned char* tmp_map;
      tmp_map = costmap_->getCharMap();

      ROS_INFO("ZACH DEBUG: costmap sizex and y and res is %d %d %f %f %f",a,b,res,origin_x,origin_y);
/*
      int i=0;
      while(i<a*b){
	printf("costmap_[%d] is %d\n",i,*(tmp_map+i));	
	i++;
      }
*/
//  cv::Mat M=cv::Mat(b,a,CV_8UC1);
//  memcpy(M.data,tmp_map,a*b*sizeof(unsigned char));

//  imshow("Nav initial costmap ", M);
//  cvWaitKey(100);
/////////////////////////////////////////////////////


      global_frame_ = global_frame;
      planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));

      ros::NodeHandle private_nh("~/" + name);

      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      private_nh.param("visualize_potential", visualize_potential_, false);

      //if we're going to visualize the potential array we need to advertise
      if(visualize_potential_)
        potarr_pub_.advertise(private_nh, "potential", 1);

      private_nh.param("allow_unknown", allow_unknown_, true);
      private_nh.param("planner_window_x", planner_window_x_, 0.0);
      private_nh.param("planner_window_y", planner_window_y_, 0.0);
      private_nh.param("default_tolerance", default_tolerance_, 0.0);

      //get the tf prefix
      ros::NodeHandle prefix_nh;
      tf_prefix_ = tf::getPrefixParam(prefix_nh);

      make_plan_srv_ =  private_nh.advertiseService("make_plan", &NavfnROS::makePlanService, this);

      initialized_ = true;
    }
    else
      ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
  }

  void NavfnROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point){
    return validPointPotential(world_point, default_tolerance_);
  }

  bool NavfnROS::validPointPotential(const geometry_msgs::Point& world_point, double tolerance){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    double resolution = costmap_->getResolution();
    geometry_msgs::Point p;
    p = world_point;

    p.y = world_point.y - tolerance;

    while(p.y <= world_point.y + tolerance){
      p.x = world_point.x - tolerance;
      while(p.x <= world_point.x + tolerance){
        double potential = getPointPotential(p);
        if(potential < POT_HIGH){
          return true;
        }
        p.x += resolution;
      }
      p.y += resolution;
    }

    return false;
  }

  double NavfnROS::getPointPotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return -1.0;
    }

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return DBL_MAX;

    unsigned int index = my * planner_->nx + mx;
    return planner_->potarr[index];
  }

  bool NavfnROS::computePotential(const geometry_msgs::Point& world_point){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

    unsigned int mx, my;
    if(!costmap_->worldToMap(world_point.x, world_point.y, mx, my))
      return false;

    int map_start[2];
    map_start[0] = 0;
    map_start[1] = 0;

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_start);
    planner_->setGoal(map_goal);

    return planner_->calcNavFnDijkstra();
  }

  void NavfnROS::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  }

  bool NavfnROS::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
    makePlan(req.start, req.goal, resp.plan.poses);

    resp.plan.header.stamp = ros::Time::now();
    resp.plan.header.frame_id = global_frame_;

    return true;
  } 

  void NavfnROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    return makePlan(start, goal, default_tolerance_, plan);
  }

  bool NavfnROS::makePlanNoRos(const Header::PoseStamped& start, 
      const Header::PoseStamped& goal, double tolerance, std::vector<Header::PoseStamped>& plan){

    printf("makePlanNoRos Start \n");
 #if 0
/////////////////////////////////////////////// ZACH DEBUG
    unsigned int a = costmap_->getSizeX();
    unsigned int b = costmap_->getSizeY();
    double res = costmap_->getResolution();
    double origin_x = costmap_->getOriginX();
    double origin_y = costmap_->getOriginY();
    unsigned char* tmp_map;
    tmp_map = costmap_->getCharMap();
    printf("get costmap in NavfnROS makePlan for debug\n");
    //ROS_INFO("ZACH DEBUG: NoRos make plan costmap sizex and y and res is %d %d %f %f %f",a,b,res,origin_x,origin_y);

    cv::Mat M_1=cv::Mat(b,a,CV_8UC1);
    memcpy(M_1.data,tmp_map,a*b*sizeof(unsigned char));
    flip(M_1,M_1,0);
    cv::Mat M_1_b;
    createOpenCVDebugMatForCostmap(M_1, M_1_b);
    cv::Mat M_3;
    cv::cvtColor( M_1_b, M_3, CV_GRAY2RGB);
    //imshow("Nav makePlan costmap M1 ", M_1);
    float sx = start.pose.position.x;
    float sy = start.pose.position.y;
    float gx = goal.pose.position.x;
    float gy = goal.pose.position.y;
    ROS_INFO("ZACH DEBUG: NoRos sx sy gx gy %f %f %f %f",sx,sy,gx,gy);


    drawPointWithSize(M_3,sx,sy,0,0,255,0,origin_x,origin_y,res,7);// green
    drawPointWithSize(M_3,gx,gy,0,255,0,0,origin_x,origin_y,res,7);// green
 

    imshow("NoRos makePlan costmap ", M_3);
    cvWaitKey(100);
/////////////////////////////////////////////// ZACH DEBUG END
#endif
       
    plan.clear();

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    unsigned int mx, my;

    costmap_->worldToMap(wx, wy, mx, my);

    costmap_->setCost(mx,my,costmap_2d::FREE_SPACE);

    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    bool allow_unknown = true;
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown);

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    costmap_->worldToMap(wx, wy, mx, my);

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;
    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    planner_->calcNavFnDijkstra(true);
    double resolution = costmap_->getResolution();
    Header::PoseStamped p, best_pose;
    p = goal;


    bool found_legal = false;
    double best_sdist = std::numeric_limits<double>::max();//1.7976931348623157e+308

    p.pose.position.y = goal.pose.position.y - tolerance;

    while(p.pose.position.y <= goal.pose.position.y + tolerance){
      p.pose.position.x = goal.pose.position.x - tolerance;
      while(p.pose.position.x <= goal.pose.position.x + tolerance){

      //double potential = getPointPotential(p.pose.position);

        double potential;
        {
        if(!costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my))
        potential =  DBL_MAX;

        unsigned int index = my * planner_->nx + mx;
        potential =  planner_->potarr[index];
        }

        //double sdist = sq_distance(p, goal);
        double dx = p.pose.position.x - goal.pose.position.x;
        double dy = p.pose.position.y - goal.pose.position.y;
        double sdist = dx*dx+dy*dy;


        if(potential < POT_HIGH && sdist < best_sdist){
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.pose.position.x += resolution;
      }
      p.pose.position.y += resolution;
    }

    if(found_legal){
      //extract the plan
      if(getPlanFromPotentialNoRos(best_pose, plan)){
        Header::PoseStamped goal_copy = best_pose;
        plan.push_back(goal_copy);
      }
      else{
        printf("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }
#if 0
////////////// ZACH DEBUG
    int i=0;
    unsigned int size_of_plan = plan.size();
    double plan_x = 0, plan_y = 0;
    while(i<size_of_plan){
      plan_x = plan[i].pose.position.x;
      plan_y = plan[i].pose.position.y;
      //printf("ZACH DEBUG : size is %d path is %f %f\n",size_of_plan,plan_x,plan_y);
      i++;
      //drawPose(M_3,plan_x,plan_y,0,255,0,0,origin_x,origin_y,res);
      drawPointWithSize(M_3,plan_x,plan_y,0,255,0,0,origin_x,origin_y,res,2);
    }
    drawPointWithSize(M_3,0,0,0,0,0,255,0,0,res,5);// origin blue
    drawPointWithSize(M_3,3,0,0,255,0,0,0,0,res,5); // x red
    drawPointWithSize(M_3,0,3,0,0,255,0,0,0,res,5); // y green

    drawPointWithSize(M_3,0,0,0,0,0,255,origin_x,origin_y,res,5);// origin blue
    drawPointWithSize(M_3,3,0,0,255,0,0,origin_x,origin_y,res,5); // x red
    drawPointWithSize(M_3,0,3,0,0,255,0,origin_x,origin_y,res,5); // y green

    drawPointWithSize(M_3,wx,wy,0,0,255,0,origin_x,origin_y,res,7);// green

    drawPointWithSize(M_3,sx,sy,0,0,255,0,origin_x,origin_y,res,7);// green
    drawPointWithSize(M_3,gx,gy,0,255,0,0,origin_x,origin_y,res,7);// green
 
#if 0
    cv::Point pt;
    pt.x = 0/res;pt.y = 0/res;
    cv::circle(M_3, pt, 5, cv::Scalar(255,0,0));

    pt.x = 3/res;pt.y = 0/res;
    cv::circle(M_3, pt, 5, cv::Scalar(0,0,255));

    pt.x = 0/res;pt.y = 3/res;
    cv::circle(M_3, pt, 5, cv::Scalar(0,255,0));
#endif

    //drawPointWithSize(M_3,0,10,0,0,0,255,0,0,res,10);

    imshow("NoRos Nav makePlan plan costmap M3 ", M_3);
    cvWaitKey(100);
#endif

    printf("makePlanNoRos End \n");

////////////// ZACH DEBUG END
    return !plan.empty();

 

}

  bool NavfnROS::makePlan(const geometry_msgs::PoseStamped& start, 
      const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::mutex::scoped_lock lock(mutex_);

#if 1
/////////////////////////////////////////////// ZACH DEBUG
      unsigned int a = costmap_->getSizeInCellsX();
      unsigned int b = costmap_->getSizeInCellsY();
      double res = costmap_->getResolution();
      double origin_x = costmap_->getOriginX();
      double origin_y = costmap_->getOriginY();
      unsigned char* tmp_map;
      tmp_map = costmap_->getCharMap();

      ROS_INFO("make plan costmap sizex and y and res is %d %d %f %f %f",a,b,res,origin_x,origin_y);

  cv::Mat M_1=cv::Mat(b,a,CV_8UC1);
  memcpy(M_1.data,tmp_map,a*b*sizeof(unsigned char));
  flip(M_1,M_1,0);
  cv::Mat M_1_b;
  createOpenCVDebugMatForCostmap(M_1, M_1_b);
  cv::Mat M_3;
  cv::cvtColor( M_1_b, M_3, CV_GRAY2RGB);
  //imshow("Nav makePlan costmap M1 ", M_1);
  //imshow("Nav makePlan plan costmap M3 ", M_3);
  //cvWaitKey();
/////////////////////////////////////////////// ZACH DEBUG END
#endif

    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
      return false;
    }

    if(tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
      ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
      return false;
    }

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
      return false;
    }
    ROS_INFO("ZACH DEBUG: wx wy first is %f %f\n",wx,wy);
    ROS_INFO("ZACH DEBUG: mx my first is %d %d\n",mx,my);
    ROS_INFO("ZACH DEBUG: origin_x origin_y first is %f %f\n",origin_x,origin_y);
    ROS_INFO("ZACH DEBUG: size_x_ size_y_ first is %d %d\n",costmap_->getSizeInCellsX(),costmap_->getSizeInCellsY());





    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, mx, my);

#if 0
    {
      static int n = 0;
      static char filename[1000];
      snprintf( filename, 1000, "navfnros-makeplan-costmapB-%04d.pgm", n++ );
      costmap->saveRawMap( std::string( filename ));
    }
#endif

    //make sure to resize the underlying array that Navfn uses
    planner_->setNavArr(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

#if 0
    {
      static int n = 0;
      static char filename[1000];
      snprintf( filename, 1000, "navfnros-makeplan-costmapC-%04d", n++ );
      planner_->savemap( filename );
    }
#endif

    int map_start[2];
    map_start[0] = mx;
    map_start[1] = my;

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    printf("ZACH DEBUG : tolerance is %f\n",tolerance);
    printf("ZACH DEBUG : tolerance is %f\n",tolerance);
    printf("ZACH DEBUG : tolerance is %f\n",tolerance);
 
    if(!costmap_->worldToMap(wx, wy, mx, my)){

      if(tolerance <= 0.0){
        ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
        return false;
      }
      mx = 0;
      my = 0;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);
    planner_->setGoal(map_start);

    bool success = planner_->calcNavFnAstar();
    //planner_->calcNavFnDijkstra(true);

    double resolution = costmap_->getResolution();
    geometry_msgs::PoseStamped p, best_pose;
    p = goal;

    bool found_legal = false;
    double best_sdist = DBL_MAX;

    p.pose.position.y = goal.pose.position.y - tolerance;

    while(p.pose.position.y <= goal.pose.position.y + tolerance){
      p.pose.position.x = goal.pose.position.x - tolerance;
      while(p.pose.position.x <= goal.pose.position.x + tolerance){
        double potential = getPointPotential(p.pose.position);
        double sdist = sq_distance(p, goal);
        if(potential < POT_HIGH && sdist < best_sdist){
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.pose.position.x += resolution;
      }
      p.pose.position.y += resolution;
    }

    if(found_legal){
      //extract the plan
      if(getPlanFromPotential(best_pose, plan)){
        //make sure the goal we push on has the same timestamp as the rest of the plan
        geometry_msgs::PoseStamped goal_copy = best_pose;
        goal_copy.header.stamp = ros::Time::now();
        plan.push_back(goal_copy);
      }
      else{
        ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
      }
    }

    if (visualize_potential_){
      //publish potential array
      pcl::PointCloud<PotarrPoint> pot_area;
      pot_area.header.frame_id = global_frame_;
      pot_area.points.clear();
      std_msgs::Header header;
      pcl_conversions::fromPCL(pot_area.header, header);
      header.stamp = ros::Time::now();
      pot_area.header = pcl_conversions::toPCL(header);

      PotarrPoint pt;
      float *pp = planner_->potarr;
      double pot_x, pot_y;
      for (unsigned int i = 0; i < (unsigned int)planner_->ny*planner_->nx ; i++)
      {
        if (pp[i] < 10e7)
        {
          mapToWorld(i%planner_->nx, i/planner_->nx, pot_x, pot_y);
          pt.x = pot_x;
          pt.y = pot_y;
          pt.z = pp[i]/pp[planner_->start[1]*planner_->nx + planner_->start[0]]*20;
          pt.pot_value = pp[i];
          pot_area.push_back(pt);
        }
      }
      potarr_pub_.publish(pot_area);
    }
    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
////////////// ZACH DEBUG
    int i=0;
    unsigned int size_of_plan = plan.size();
    double plan_x = 0, plan_y = 0;
    while(i<size_of_plan){
      plan_x = plan[i].pose.position.x;
      plan_y = plan[i].pose.position.y;
      //printf("ZACH DEBUG : size is %d path is %f %f\n",size_of_plan,plan_x,plan_y);
      i++;
      //drawPose(M_3,plan_x,plan_y,0,255,0,0,origin_x,origin_y,res);
      drawPointWithSize(M_3,plan_x,plan_y,0,255,0,0,origin_x,origin_y,res,2);
    }
    drawPointWithSize(M_3,0,0,0,0,0,255,0,0,res,5);// origin blue
    drawPointWithSize(M_3,3,0,0,255,0,0,0,0,res,5); // x red
    drawPointWithSize(M_3,0,3,0,0,255,0,0,0,res,5); // y green

    drawPointWithSize(M_3,0,0,0,0,0,255,origin_x,origin_y,res,5);// origin blue
    drawPointWithSize(M_3,3,0,0,255,0,0,origin_x,origin_y,res,5); // x red
    drawPointWithSize(M_3,0,3,0,0,255,0,origin_x,origin_y,res,5); // y green

    drawPointWithSize(M_3,wx,wy,0,0,255,0,origin_x,origin_y,res,7);// green
    ROS_INFO("ZACH DEBUG: wx wy plot is %f %f\n",wx,wy);


    double sx = start.pose.position.x;
    double sy = start.pose.position.y;
    double gx = goal.pose.position.x;
    double gy = goal.pose.position.y;
    ROS_INFO("ZACH DEBUG: Ros sx sy gx gy %f %f %f %f",sx,sy,gx,gy);

    drawPointWithSize(M_3,sx,sy,0,0,255,0,origin_x,origin_y,res,7);// green
    drawPointWithSize(M_3,gx,gy,0,255,0,0,origin_x,origin_y,res,7);// green
 




    cv::Point pt;
    pt.x = 0/res;pt.y = 0/res;
    cv::circle(M_3, pt, 5, cv::Scalar(255,0,0));

    pt.x = 3/res;pt.y = 0/res;
    cv::circle(M_3, pt, 5, cv::Scalar(0,0,255));

    pt.x = 0/res;pt.y = 3/res;
    cv::circle(M_3, pt, 5, cv::Scalar(0,255,0));


    //drawPointWithSize(M_3,0,10,0,0,0,255,0,0,res,10);

    imshow("Nav makePlan plan costmap M3 ", M_3);
    cvWaitKey(100);


////////////// ZACH DEBUG END
    return !plan.empty();
  }

  void NavfnROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    //create a message for the plan 
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(!path.empty())
    {
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
      gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
  }


  bool NavfnROS::getPlanFromPotentialNoRos(const Header::PoseStamped& goal, std::vector<Header::PoseStamped>& plan){

    plan.clear();

    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    unsigned int mx, my;
    costmap_->worldToMap(wx, wy, mx, my);

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->calcPath(costmap_->getSizeInCellsX() * 4);

    //extract the plan
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();

    for(int i = len - 1; i >= 0; --i){
      //convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);

      Header::PoseStamped pose;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    return !plan.empty();
  }

  bool NavfnROS::getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return false;
    }

    //clear the plan, just in case
    plan.clear();

    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame
    if(tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame_)){
      ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                tf::resolve(tf_prefix_, global_frame_).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
      return false;
    }

    double wx = goal.pose.position.x;
    double wy = goal.pose.position.y;

    //the potential has already been computed, so we won't update our copy of the costmap
    unsigned int mx, my;
    if(!costmap_->worldToMap(wx, wy, mx, my)){
      ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
      return false;
    }

    int map_goal[2];
    map_goal[0] = mx;
    map_goal[1] = my;

    planner_->setStart(map_goal);

    planner_->calcPath(costmap_->getSizeInCellsX() * 4);

    //extract the plan
    float *x = planner_->getPathX();
    float *y = planner_->getPathY();
    int len = planner_->getPathLen();
    ros::Time plan_time = ros::Time::now();

    for(int i = len - 1; i >= 0; --i){
      //convert the plan to world coordinates
      double world_x, world_y;
      mapToWorld(x[i], y[i], world_x, world_y);

      geometry_msgs::PoseStamped pose;
      pose.header.stamp = plan_time;
      pose.header.frame_id = global_frame_;
      pose.pose.position.x = world_x;
      pose.pose.position.y = world_y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      plan.push_back(pose);
    }

    //publish the plan for visualization purposes
    publishPlan(plan, 0.0, 1.0, 0.0, 0.0);
    return !plan.empty();
  }
};
