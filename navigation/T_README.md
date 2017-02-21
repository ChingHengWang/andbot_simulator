TEST
=======================================
#test
##move_base : memeber
      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* planner_costmap_ros_, *controller_costmap_ros_;
      std::vector<Pose>* planner_plan_;
      std::vector<Pose>* latest_plan_;
      std::vector<Pose>* controller_plan_;


##move_base : method

      tc_ = blp_loader_.createInstance(local_planner); //DWA_PLANNER_ROS
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);




void MoveBase::planThread(){
	while(){
		Pose temp_goal = planner_goal_;
		makePlan(temp_goal, *planner_plan_);
		std::vector<Pose>* temp_plan = planner_plan_;
		latest_plan_ = temp_plan;



void MoveBase::executeCb(move_base_goal)
	goal = goalToGlobalFrame(move_base_goal->target_pose);
	planner_goal_ = goal; //go to planThread



bool MoveBase::executeCycle()

	geometry_msgs::Twist cmd_vel;

	tf::Stamped<tf::Pose> global_pose;
	planner_costmap_ros_->getRobotPose(global_pose);
	geometry_msgs::PoseStamped current_position;
	tf::poseStampedTFToMsg(global_pose, current_position);

	controller_plan_ = latest_plan_;
	tc_->setPlan(*controller_plan_)//go to dwa_planner_ros setPlan

switch(state)
	case CONTROLLING:
		if(tc_->isGoalReached()))
			goal reached!
			return
		tc_->computeVelocityCommands(cmd_vel)) //go to dwa_planner_ros computeVelocityCommands
						       // tc_ is dwa_planner_ros		

##dwa_planner_ros
###member
      base_local_planner::LocalPlannerUtil planner_util_;
      costmap_2d::Costmap2DROS* costmap_ros_; //initialize 的時候被定義
      base_local_planner::LatchedStopRotateController latchedStopRotateController_;
      boost::shared_ptr<DWAPlanner> dp_; 
      costmap_2d::Costmap2DROS* costmap_ros_;
      tf::Stamped<Pose> current_pose_;



###method

* initialize
  void DWAPlannerROS::initialize(costmap_2d::Costmap2DROS* costmap_ros) {

      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap);

      dp_ = boost::shared_ptr<DWAPlanner>(new DWAPlanner(name, &planner_util_));


      bool setPlan(const std::vector<Pose>& orig_global_plan) {  //from move_base executeCycle
      	//latchedStopRotateController_.resetLatching();
	return dp_->setPlan(orig_global_plan);

* computeVelocityCommands
      bool computeVelocityCommands(Twist& cmd_vel) { // from move_base tc_->computeVelocity
	  costmap_ros_->getRobotPose(current_pose_))
	  std::vector<Pose> transformed_plan;
	  planner_util_.getLocalPlan(current_pose_, transformed_plan)) // go to local_planner_util method
  }



##dwa_planner
###member
      base_local_planner::LocalPlannerUtil *planner_util_;
      double stop_time_buffer_; 
      double pdist_scale_, gdist_scale_, occdist_scale_;
      Eigen::Vector3f vsamples_;
      double sim_period_;/
      base_local_planner::Trajectory result_traj_;
      double forward_point_distance_;
      std::vector<geometry_msgs::PoseStamped> global_plan_;
      pcl::PointCloud<base_local_planner::MapGridCostPoint>* traj_cloud_;
      pcl_ros::Publisher<base_local_planner::MapGridCostPoint> traj_cloud_pub_;

      base_local_planner::SimpleTrajectoryGenerator generator_;
      base_local_planner::OscillationCostFunction oscillation_costs_;
      base_local_planner::ObstacleCostFunction obstacle_costs_;
      base_local_planner::MapGridCostFunction path_costs_;
      base_local_planner::MapGridCostFunction goal_costs_;
      base_local_planner::MapGridCostFunction goal_front_costs_;
      base_local_planner::MapGridCostFunction alignment_costs_;

      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;


###method
      bool DWAPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan) {
    	oscillation_costs_.resetOscillationFlags();
    	return planner_util_->setPlan(orig_global_plan);
      }


	findBestPath


##local_planner_util dwa_planner_ros and dwa_planner both has this class
###member

  std::vector<Pose> global_plan // move_base setPlan

  costmap_2d::Costmap2D costmap //dwa_planner_ros initialize planner_util and parse into dwa_planner 

  LocalPlannerLimits limits;


###method


bool setPlan(const std::vector<Pose>& orig_global_plan) // from dwa_planner_ros setPlan
  //reset the global plan
  global_plan.clear();
  global_plan = orig_global_plan;
  return true;



//from computeVelocityCommand of dwa_planner_ros
bool LocalPlannerUtil::getLocalPlan(tf::Stamped<tf::Pose>& global_pose, std::vector<Pose>& transformed_plan) 

	base_local_planner::transformGlobalPlan(  // is in goal_function.cpp
     	 *tf_,
     	 global_plan_,  // move_base setPlan
      	global_pose,    // from dwa_planner_ros
      	*costmap_,      // costmap_ is from dwa_planner_ros.initialize()
      	global_frame_,
      	transformed_plan))
					//goal_functions.cpp 
    if(limits.prune_plan) 

    base_local_planner::prunePlan(global_pose, transformed_plan, global_plan_);
 

##latched_stop_rotate_controller







