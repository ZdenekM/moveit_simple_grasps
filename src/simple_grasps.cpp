/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
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
 *********************************************************************/

#include <moveit_simple_grasps/simple_grasps.h>
#include <shape_tools/shape_extents.h>

namespace moveit_simple_grasps
{

// Constructor
SimpleGrasps::SimpleGrasps(moveit_visual_tools::VisualToolsPtr visual_tools) :
  visual_tools_(visual_tools)
{
  ROS_DEBUG_STREAM_NAMED("grasps","Loaded simple grasp generator");
}

// Deconstructor
SimpleGrasps::~SimpleGrasps()
{
}

void SimpleGrasps::fillGraspFromLocalGraspPose(const Eigen::Affine3d & local_grasp, moveit_msgs::Grasp & grasp)
{
    // Change grasp to frame of reference of this custom end effector

    // grasp point to eef frame
    Eigen::Affine3d grasp_pose = local_grasp * eef_conversion_pose_;

    // point in eef frame to global frame (base_link)
    tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp.grasp_pose.pose);
}

void SimpleGrasps::initializeGrasp(moveit_msgs::Grasp & grasp, const GraspData & grasp_data,
        const std_msgs::Header & grasp_header)
{
    // Postures
    grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;
    grasp.grasp_posture = grasp_data.grasp_posture_;
    // grasp_pose
    grasp.grasp_pose.header = grasp_header;

    grasp.grasp_quality = 0.0;

    // Motions
    grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();
    grasp.pre_grasp_approach.direction.header.frame_id = grasp_data.ee_parent_link_;
    grasp.pre_grasp_approach.direction.vector.x = 1;
    grasp.pre_grasp_approach.direction.vector.y = 0;
    grasp.pre_grasp_approach.direction.vector.z = 0;
    grasp.pre_grasp_approach.desired_distance = grasp_data.approach_retreat_desired_dist_;
    grasp.pre_grasp_approach.min_distance = grasp_data.approach_retreat_min_dist_;

    grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();
    grasp.post_grasp_retreat.direction.header.frame_id = grasp_data.base_link_;
    grasp.post_grasp_retreat.direction.vector.x = 0;
    grasp.post_grasp_retreat.direction.vector.y = 0;
    grasp.post_grasp_retreat.direction.vector.z = 1;
    grasp.post_grasp_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_;
    grasp.post_grasp_retreat.min_distance = grasp_data.approach_retreat_min_dist_;

    grasp.post_place_retreat.direction.header.stamp = ros::Time::now();
    grasp.post_place_retreat.direction.header.frame_id = grasp_data.ee_parent_link_;
    grasp.post_place_retreat.direction.vector.x = -1;
    grasp.post_place_retreat.direction.vector.y = 0;
    grasp.post_place_retreat.direction.vector.z = 0;
    grasp.post_place_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_;
    grasp.post_place_retreat.min_distance = grasp_data.approach_retreat_min_dist_;

    // <=0 to disable?
    grasp.max_contact_force = 0;
}

bool SimpleGrasps::generateShapeGrasps(const shape_msgs::SolidPrimitive & shape,
        const geometry_msgs::PoseStamped & object_pose,
        const GraspData& grasp_data, std::vector<moveit_msgs::Grasp>& possible_grasps)
{
    if(shape.type == shape_msgs::SolidPrimitive::BOX) {
        return generateBoxGrasps(shape, object_pose, grasp_data, possible_grasps);
    } else if(shape.type == shape_msgs::SolidPrimitive::CYLINDER) {
        return generateCylinderGrasps(shape, object_pose, grasp_data, possible_grasps);
    } else {
        ROS_ERROR("%s: Shape type %d not implemented.", __PRETTY_FUNCTION__, shape.type);
        return false;
    }
}

void SimpleGrasps::addNewGrasp(moveit_msgs::Grasp & grasp, const Eigen::Affine3d & local_grasp_pose,
        std::vector<moveit_msgs::Grasp> & possible_grasps)
{
    // DEBUG - show original grasp pose before tranform to gripper frame
    if(visual_tools_)
    {
        tf::poseEigenToMsg(object_global_transform_ * local_grasp_pose, grasp.grasp_pose.pose);
        visual_tools_->publishArrow(grasp.grasp_pose.pose, moveit_visual_tools::GREEN);
    }

    fillGraspFromLocalGraspPose(local_grasp_pose, grasp);

    static int grasp_id = 0;
    grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    grasp_id++;

    possible_grasps.push_back(grasp);
}

bool SimpleGrasps::generateBoxGrasps(const shape_msgs::SolidPrimitive & shape,
        const geometry_msgs::PoseStamped & object_pose,
        const GraspData& grasp_data, std::vector<moveit_msgs::Grasp>& possible_grasps)
{
    // prepare transforms and grasp for this request
    tf::poseMsgToEigen(object_pose.pose, object_global_transform_);
    tf::poseMsgToEigen(grasp_data.grasp_pose_to_eef_pose_, eef_conversion_pose_);
    moveit_msgs::Grasp grasp;
    initializeGrasp(grasp, grasp_data, object_pose.header);

    // these should be correct for a box
    double wx, wy, wz;
    shape_tools::getShapeExtents(shape, wx, wy, wz);
    // shape origin should be in the center
    const double box_edge_holdoff = 0.05;   // don't create grasps right at the edge/corner

    // grasps on the x axis sides
    if(wy <= grasp_data.pre_grasp_opening_) {
        Eigen::Affine3d grasp_pose;
        // sides up
        for(int zstep = 0; zstep < grasp_data.linear_steps_; zstep++) {
            double dz = - 0.5 * wz + box_edge_holdoff + 
                zstep * (wz - 2 * box_edge_holdoff)/(grasp_data.linear_steps_ - 1);
            double dx = - 0.5 * wx + grasp_data.grasp_depth_;   // depth = how much from border in
            double dy = 0.0;

            grasp.grasp_quality = cos(M_PI_2 * dz/(0.5 * wz));  // the more centered in z the better

            grasp_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
            grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);

            // opposing side
            grasp_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
            grasp_pose.translation() = Eigen::Vector3d(-dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);
        }
        // top
        for(int xstep = 0; xstep < grasp_data.linear_steps_; xstep++) {
            double dz = 0.5 * wz - grasp_data.grasp_depth_;
            double dy = 0.0;
            double dx = - 0.5 * wx + box_edge_holdoff + 
                xstep * (wx - 2 * box_edge_holdoff)/(grasp_data.linear_steps_ - 1);

            // 0.5 to prefer side grasps
            grasp.grasp_quality = 0.5 * cos(M_PI_2 * dx/(0.5 * wx));  // the more centered in x the better

            grasp_pose = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
            grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);

            // opposing direction
            grasp_pose = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
            grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);
        }
    }

    // grasps on the y axis sides
    if(wx <= grasp_data.pre_grasp_opening_) {
        Eigen::Affine3d grasp_pose;
        // sides up
        for(int zstep = 0; zstep < grasp_data.linear_steps_; zstep++) {
            double dz = - 0.5 * wz + box_edge_holdoff + 
                zstep * (wz - 2 * box_edge_holdoff)/(grasp_data.linear_steps_ - 1);
            double dy = - 0.5 * wy + grasp_data.grasp_depth_;   // depth = how much from border in
            double dx = 0.0;

            grasp.grasp_quality = cos(M_PI_2 * dz/(0.5 * wz));  // the more centered in z the better

            grasp_pose = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ());
            grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);

            // opposing side
            grasp_pose = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ());
            grasp_pose.translation() = Eigen::Vector3d(dx, -dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);
        }
        // top
        for(int ystep = 0; ystep < grasp_data.linear_steps_; ystep++) {
            double dz = 0.5 * wz - grasp_data.grasp_depth_;
            double dx = 0.0;
            double dy = - 0.5 * wy + box_edge_holdoff + 
                ystep * (wy - 2 * box_edge_holdoff)/(grasp_data.linear_steps_ - 1);

            // 0.5 to prefer side grasps
            grasp.grasp_quality = 0.5 * cos(M_PI_2 * dy/(0.5 * wy));  // the more centered in x the better

            grasp_pose = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
            grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);

            // opposing direction
            grasp_pose = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
            grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);
        }
    }
    ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

    return true;
}

bool SimpleGrasps::generateCylinderGrasps(const shape_msgs::SolidPrimitive & shape,
        const geometry_msgs::PoseStamped & object_pose,
        const GraspData& grasp_data, std::vector<moveit_msgs::Grasp>& possible_grasps)
{
    // prepare transforms and grasp for this request
    tf::poseMsgToEigen(object_pose.pose, object_global_transform_);
    tf::poseMsgToEigen(grasp_data.grasp_pose_to_eef_pose_, eef_conversion_pose_);
    moveit_msgs::Grasp grasp;
    initializeGrasp(grasp, grasp_data, object_pose.header);

    // these should be correct for a box
    double wx, wy, wz;
    shape_tools::getShapeExtents(shape, wx, wy, wz);
    // shape origin should be in the center

    const double cyl_edge_holdoff = 0.05;   // don't create grasps right at the edge/corner

    // side grasps, wx/wy should be equal
    if(wy <= grasp_data.pre_grasp_opening_) {
        Eigen::Affine3d grasp_pose;
        // sides up
        for(int zstep = 0; zstep < grasp_data.linear_steps_; zstep++) {
            double dz = - 0.5 * wz + cyl_edge_holdoff + 
                zstep * (wz - 2 * cyl_edge_holdoff)/(grasp_data.linear_steps_ - 1);
            double dz_quality = cos(M_PI_2 * dz/(0.5 * wz));  // the more centered in z the better
            // side around
            for(int ang_step = 0; ang_step < grasp_data.angle_steps_; ang_step++) {
                // no -1 = won't end up at 2 pi = 0
                double da = 2 * M_PI * static_cast<double>(ang_step)/grasp_data.angle_steps_;
                double grasp_radius = 0.5 * wx - grasp_data.grasp_depth_;
                const double over_center_max = 0.05;
                // we grasp as deep as we can, but if we reach past the center of something round,
                // prefer the center
                if(grasp_radius < -over_center_max)
                    grasp_radius = -over_center_max;
                // grasp_radius > 0 is actually bad -> also wy test is too restrictive then
                grasp.grasp_quality = exp(-(grasp_radius + over_center_max)/0.05) * dz_quality;
                double dx = -grasp_radius * cos(da);
                double dy = -grasp_radius * sin(da);

                grasp_pose = Eigen::AngleAxisd(da, Eigen::Vector3d::UnitZ());
                grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
                addNewGrasp(grasp, grasp_pose, possible_grasps);
            }
        }

        // top
        for(int ang_step = 0; ang_step < grasp_data.angle_steps_; ang_step++) {
            // no -1 = won't end up at 2 pi = 0
            double da = 2 * M_PI * static_cast<double>(ang_step)/grasp_data.angle_steps_;

            double dx = 0.0;
            double dy = 0.0;
            double dz = 0.5 * wz - grasp_data.grasp_depth_;

            grasp.grasp_quality = 0.5;

            grasp_pose = Eigen::AngleAxisd(da, Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
            grasp_pose.translation() = Eigen::Vector3d(dx, dy, dz);
            addNewGrasp(grasp, grasp_pose, possible_grasps);
        }
    }

    ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

    return true;
}


// Create all possible grasp positions for a object
bool SimpleGrasps::generateBlockGrasps(const geometry_msgs::Pose& object_pose, const GraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Calculate grasps in two axis in both directions
  ROS_INFO("Generating X_AXIS DOWN HALF grasps.");
  generateAxisGrasps( object_pose, X_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // got no grasps with this alone
  ROS_INFO("Generating X_AXIS UP HALF grasps.");
  generateAxisGrasps( object_pose, X_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gives some grasps... looks ugly
  ROS_INFO("Generating Y_AXIS DOWN HALF grasps.");
  generateAxisGrasps( object_pose, Y_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // GOOD ONES!
  ROS_INFO("Generating Y_AXIS UP HALF grasps.");
  generateAxisGrasps( object_pose, Y_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gave a grasp from top... bad

  return true;
}

// Create grasp positions in one axis
bool SimpleGrasps::generateAxisGrasps(
  const geometry_msgs::Pose& object_pose,
  grasp_axis_t axis,
  grasp_direction_t direction,
  grasp_rotation_t rotation,
  double hand_roll,
  const GraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Create a transform from the object's frame (center of object) to /base_link
  tf::poseMsgToEigen(object_pose, object_global_transform_);

  // Convert to Eigen
  tf::poseMsgToEigen(grasp_data.grasp_pose_to_eef_pose_, eef_conversion_pose_);


  // ---------------------------------------------------------------------------------------------
  // Grasp parameters

  // Create re-usable approach motion
  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();
  pre_grasp_approach.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
  pre_grasp_approach.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // Create re-usable retreat motion
  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();
  post_grasp_retreat.desired_distance = grasp_data.approach_retreat_desired_dist_; // The distance the origin of a robot link needs to travel
  post_grasp_retreat.min_distance = grasp_data.approach_retreat_min_dist_; // half of the desired? Untested.

  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // ---------------------------------------------------------------------------------------------
  // Angle calculations
  double radius = grasp_data.grasp_depth_; //0.12
  double xb;
  double yb = 0.0; // stay in the y plane of the object
  double zb;
  double theta1 = 0.0; // Where the point is located around the object
  double theta2 = 0.0; // UP 'direction'

  // Gripper direction (UP/DOWN) rotation. UP set by default
  if( direction == DOWN )
  {
    theta2 = M_PI;
  }

  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------
  // Begin Grasp Generator Loop
  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------

  /* Developer Note:
   * Create angles 180 degrees around the chosen axis at given resolution
   * We create the grasps in the reference frame of the object, then later convert it to the base link
   */
  for(int i = 0; i <= grasp_data.angle_steps_; ++i)
  {
    // Create a Grasp message
    moveit_msgs::Grasp new_grasp;

    // Calculate grasp pose
    xb = radius*cos(theta1);
    zb = radius*sin(theta1);

    Eigen::Affine3d grasp_pose;

    switch(axis)
    {
      case X_AXIS:
        grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);

        break;
      case Y_AXIS:
        grasp_pose =
          Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
          *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

        break;
      case Z_AXIS:
        ROS_ERROR_STREAM_NAMED("grasp","Z Axis not implemented!");
        return false;

        break;
    }

    /* The estimated probability of success for this grasp, or some other measure of how "good" it is.
     * Here we base bias the score based on how far the wrist is from the surface, preferring a greater
     * distance to prevent wrist/end effector collision with the table
     */
    double score = sin(theta1);
    new_grasp.grasp_quality = std::max(score,0.1); // don't allow score to drop below 0.1 b/c all grasps are ok

    // Calculate the theta1 for next time
    if (rotation == HALF)
      theta1 += M_PI / grasp_data.angle_steps_;
    else
    {
      theta1 += 2*M_PI / grasp_data.angle_steps_;
    }

    // A name for this grasp
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    ++grasp_id;

    // PreGrasp and Grasp Postures --------------------------------------------------------------------------

    // The internal posture of the hand for the pre-grasp only positions are used
    new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;

    // The internal posture of the hand for the grasp positions and efforts are used
    new_grasp.grasp_posture = grasp_data.grasp_posture_;

    // Grasp ------------------------------------------------------------------------------------------------


    // DEBUG - show original grasp pose before tranform to gripper frame
    if(visual_tools_)
    {
      tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
      visual_tools_->publishArrow(grasp_pose_msg.pose, moveit_visual_tools::GREEN);
    }

    // ------------------------------------------------------------------------
    // Optionally roll wrist with respect to object pose
    Eigen::Affine3d roll_gripper;
    roll_gripper = Eigen::AngleAxisd(hand_roll, Eigen::Vector3d::UnitX());
    grasp_pose = grasp_pose * roll_gripper;

    // ------------------------------------------------------------------------
    // Change grasp to frame of reference of this custom end effector

    // Transform the grasp pose
    grasp_pose = grasp_pose * eef_conversion_pose_;

    // ------------------------------------------------------------------------
    // Convert pose to global frame (base_link)
    tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);

    // The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
    new_grasp.grasp_pose = grasp_pose_msg;

    // Other ------------------------------------------------------------------------------------------------

    // the maximum contact force to use while grasping (<=0 to disable)
    new_grasp.max_contact_force = 0;

    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------
    // Approach and retreat
    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------

    // Straight down ---------------------------------------------------------------------------------------
    // With respect to the base link/world frame

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.base_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.base_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

    // Angled with pose -------------------------------------------------------------------------------------
    // Approach with respect to end effector orientation

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.ee_parent_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = 1;
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.ee_parent_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = -1;
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

  }

  ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

  return true;
}

geometry_msgs::PoseStamped SimpleGrasps::getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link)
{
  // Grasp Pose Variables
  geometry_msgs::PoseStamped grasp_pose = grasp.grasp_pose;
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose.pose, grasp_pose_eigen);

  // Get pre-grasp pose first
  geometry_msgs::PoseStamped pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen = grasp_pose_eigen; // Copy original grasp pose to pre-grasp pose

  // Approach direction variables
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // The direction of the pre-grasp
  // Calculate the current animation position based on the percent
  Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
    -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.desired_distance,
    -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.desired_distance,
    -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.desired_distance
  );

  // Decide if we need to change the approach_direction to the local frame of the end effector orientation
  if( grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link )
  {
    // Apply/compute the approach_direction vector in the local frame of the grasp_pose orientation
    pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }
  else
  {
    pre_grasp_approach_direction_local = pre_grasp_approach_direction; //grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }

  // Update the grasp matrix usign the new locally-framed approach_direction
  pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

  // Convert eigen pre-grasp position back to regular message
  tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose.pose);

  // Copy original header to new grasp
  pre_grasp_pose.header = grasp_pose.header;

  return pre_grasp_pose;
}



} // namespace
