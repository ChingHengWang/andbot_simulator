#include <my_navfn/navfn_no_ros.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

//#include <debug/cv_debug_header.h>
#include <alb/header.h>
#include <limits>


namespace navfn {

  NavfnNoROS::NavfnNoROS() 
    : costmap_(NULL),  planner_(), allow_unknown_(true) {}

  NavfnNoROS::NavfnNoROS(costmap_2d::Costmap2D* costmap)
    : costmap_(NULL),  planner_(),  allow_unknown_(true) {
      //initialize the planner
      initializeNoRos(costmap);
  }
  
void NavfnNoROS::initializeNoRos(costmap_2d::Costmap2D* costmap){
      costmap_=costmap;     
      planner_ = boost::shared_ptr<NavFn>(new NavFn(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY()));
}  

  void NavfnNoROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
  }

  bool NavfnNoROS::makePlanNoRos(const Header::PoseStamped& start, 
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
    printf("get costmap in NavfnNoROS makePlan for debug\n");
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

        double potential;
        {
        if(!costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my))
        potential =  std::numeric_limits<double>::max();//1.7976931348623157e+308
;

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


  bool NavfnNoROS::getPlanFromPotentialNoRos(const Header::PoseStamped& goal, std::vector<Header::PoseStamped>& plan){

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

};
