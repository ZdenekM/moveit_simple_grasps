/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:   Tests the grasp generator
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>

// Baxter specific properties
#include <moveit_simple_grasps/custom_environment2.h>

namespace baxter_pick_place
{

static const double BLOCK_SIZE_X = 0.08;
static const double BLOCK_SIZE_Y = 0.25;
static const double BLOCK_SIZE_Z = 0.30;

static const double CYL_RADIUS = 0.14;
static const double CYL_HEIGHT = 0.11;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  // class for publishing stuff to rviz
  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // robot-specific data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // which baxter arm are we using
  std::string arm_;
  std::string ee_group_name_;
  std::string planning_group_name_;

  int num_tests_;
public:

  // Constructor
  GraspGeneratorTest(int num_tests)
    : nh_("~"), num_tests_(num_tests)
  {
    nh_.param("arm", arm_, std::string("left"));
    nh_.param("ee_group_name", ee_group_name_, std::string(arm_ + "_gripper"));
    planning_group_name_ = arm_ + "_arm";

    ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))
      ros::shutdown();

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::VisualTools(grasp_data_.base_link_));
    visual_tools_->loadMarkerPub();
    visual_tools_->setLifetime(120.0);
    visual_tools_->setMuted(false);
    ros::Duration(2.0).sleep();
    visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps());
    //simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );
  }

  void testBoxGrasps()
  {
    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::PoseStamped object_pose;
    object_pose.header.frame_id = grasp_data_.base_link_;
    std::vector<moveit_msgs::Grasp> possible_grasps;

    // Allow ROS to catchup
    ros::Duration(2.0).sleep();

    // Loop
    int i = 0;
    while(ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests_);

      // Remove randomness when we are only running one test
      if (num_tests_ == 1)
        generateTestObject(object_pose.pose);
      else
        generateRandomObject(object_pose.pose);

      // Show the block
      visual_tools_->publishBlock(object_pose.pose, moveit_visual_tools::BLUE,
              BLOCK_SIZE_X, BLOCK_SIZE_Y, BLOCK_SIZE_Z);

      possible_grasps.clear();

      // Generate set of grasps for one object
      shape_msgs::SolidPrimitive box;
      box.type = shape_msgs::SolidPrimitive::BOX;
      box.dimensions.resize(3);
      box.dimensions[0] = BLOCK_SIZE_X;
      box.dimensions[1] = BLOCK_SIZE_Y;
      box.dimensions[2] = BLOCK_SIZE_Z;

      //ROS_INFO("Grasps Block Grasps");
      //simple_grasps_->generateBlockGrasps( object_pose, grasp_data_, possible_grasps);
      //visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

      possible_grasps.clear();
      ROS_INFO("Generating Box Grasps");
      simple_grasps_->generateShapeGrasps(box, true, false, object_pose, grasp_data_, possible_grasps);

      // Visualize them
      //visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
      visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

      // Test if done
      ++i;
      if( i >= num_tests_ )
        break;
    }
  }

  void testCylinderGrasps()
  {
    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::PoseStamped object_pose;
    object_pose.header.frame_id = grasp_data_.base_link_;
    std::vector<moveit_msgs::Grasp> possible_grasps;

    // Allow ROS to catchup
    ros::Duration(2.0).sleep();

    // Loop
    int i = 0;
    while(ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests_);

      // Remove randomness when we are only running one test
      if (num_tests_ == 1) {
        generateTestObject(object_pose.pose);
        object_pose.pose.position.x += 0.5;
      } else {
        generateRandomObject(object_pose.pose);
      }

      visual_tools_->publishCylinder(object_pose.pose, moveit_visual_tools::BLUE,
              CYL_HEIGHT, CYL_RADIUS);

      possible_grasps.clear();

      // Generate set of grasps for one object
      shape_msgs::SolidPrimitive cyl;
      cyl.type = shape_msgs::SolidPrimitive::CYLINDER;
      cyl.dimensions.resize(2);
      cyl.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = CYL_RADIUS;
      cyl.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = CYL_HEIGHT;

      //ROS_INFO("Grasps Block Grasps");
      //simple_grasps_->generateBlockGrasps( object_pose, grasp_data_, possible_grasps);
      //visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

      possible_grasps.clear();
      ROS_INFO("Generating Cylinder Grasps");
      simple_grasps_->generateShapeGrasps(cyl, false, true, object_pose, grasp_data_, possible_grasps);

      // Visualize them
      //visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
      visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

      // Test if done
      ++i;
      if( i >= num_tests_ )
        break;
    }
  }


  void generateTestObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    geometry_msgs::Pose start_object_pose;

    start_object_pose.position.x = 0.7;
    start_object_pose.position.y = 0.2;
    start_object_pose.position.z = 1.0;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_object_pose.orientation.x = quat.x();
    start_object_pose.orientation.y = quat.y();
    start_object_pose.orientation.z = quat.z();
    start_object_pose.orientation.w = quat.w();

    // Choose which object to test
    object_pose = start_object_pose;

    //visual_tools_->publishObject( object_pose, OBJECT_SIZE, true );
  }

  void generateRandomObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = fRand(0.4,1.2); //0.55);
    object_pose.position.y = fRand(-0.38,0.38);
    object_pose.position.z = 0.5;

    // Orientation
    double angle = M_PI * fRand(0.1,1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();
  }

  double fRand(double fMin, double fMax)
  {
    double f = drand48();
    return fMin + f * (fMax - fMin);
  }

}; // end of class

} // namespace

int main(int argc, char *argv[])
{
  int num_tests = 1;
  ros::init(argc, argv, "grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main","Simple Grasps Test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Seed random
  srand48(ros::Time::now().toSec());

  // Benchmark time
  ros::WallTime start_time = ros::WallTime::now();

  // Run Tests
  baxter_pick_place::GraspGeneratorTest tester(num_tests);
  //tester.testBoxGrasps();
  tester.testCylinderGrasps();

  // Benchmark time
  double duration = (ros::WallTime::now() - start_time).toSec() * 1e3;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration << " ms");
  //std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
