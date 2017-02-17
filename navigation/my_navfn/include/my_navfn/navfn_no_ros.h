#ifndef MY_NAVFN_NAVFN_NO_ROS_H_
#define MY_NAVFN_NAVFN_NO_ROS_H_

#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <vector>
#include <alb/header.h>
namespace navfn {
  class NavfnNoROS {
    public:
      NavfnNoROS();

      NavfnNoROS(costmap_2d::Costmap2D* costmap);

      void initializeNoRos(costmap_2d::Costmap2D* costmap);

      bool makePlanNoRos(const Header::PoseStamped& start,const Header::PoseStamped& goal,double tolerance, std::vector<Header::PoseStamped>& plan);

      bool getPlanFromPotentialNoRos(const Header::PoseStamped& goal, std::vector<Header::PoseStamped>& plan);

      ~NavfnNoROS(){}

    protected:

      costmap_2d::Costmap2D* costmap_;
      boost::shared_ptr<NavFn> planner_;
      bool allow_unknown_;


    private:

      void mapToWorld(double mx, double my, double& wx, double& wy);

  };
};

#endif
