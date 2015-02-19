#include "moveit_simple_grasps/moveit_simple_grasps.h"
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace moveit_simple_grasps
{

MoveitSimpleGrasps::MoveitSimpleGrasps(moveit_visual_tools::VisualToolsPtr rviz_tools) :
    simple_grasps_(new SimpleGrasps(rviz_tools))
{
}


bool MoveitSimpleGrasps::generateShapeGrasps(const moveit_msgs::CollisionObject & co, bool enclosure, bool edge,
        const GraspData & grasp_data,
        const std::vector<int> & mesh_shape_types,
        std::vector<moveit_msgs::Grasp> & possible_grasps)
{
    bool generated_grasps = false;

    ROS_ASSERT(co.primitives.size() == co.primitive_poses.size());
    for(unsigned int i = 0; i < co.primitives.size(); ++i) {
        geometry_msgs::PoseStamped prim_pose;
        prim_pose.header = co.header;
        prim_pose.pose = co.primitive_poses[i];
        generated_grasps |= simple_grasps_->generateShapeGrasps(co.primitives[i], enclosure, edge,
                prim_pose, grasp_data, possible_grasps);
    }

    ROS_ASSERT(co.meshes.size() == co.mesh_poses.size());
    if(co.meshes.size() != mesh_shape_types.size()) {
        ROS_ERROR("%s: CollisionObject %s contains %zu meshes, but %zu mesh_shape_types are given.",
                __PRETTY_FUNCTION__, co.id.c_str(), co.meshes.size(), co.mesh_poses.size());
        return generated_grasps;
    }

    for(unsigned int i = 0; i < co.meshes.size(); ++i) {
        if(mesh_shape_types[i] < 0) {
            ROS_WARN("Could not generate grasps for mesh with type %d for object %s.",
                    mesh_shape_types[i], co.id.c_str());
            continue;
        }
        shape_msgs::SolidPrimitive shape;
        geometry_msgs::PoseStamped shape_pose;
        shape_pose.header = co.header;
        if(!createPrimitiveFromMesh(co.meshes[i], co.mesh_poses[i], mesh_shape_types[i],
                    shape, shape_pose.pose)) {
            ROS_WARN("Could not create primitive for mesh with type %d for object %s.",
                    mesh_shape_types[i], co.id.c_str());
            continue;
        }

        generated_grasps |= simple_grasps_->generateShapeGrasps(shape, enclosure, edge,
                shape_pose, grasp_data, possible_grasps);
    }

    return generated_grasps;
}

bool MoveitSimpleGrasps::createPrimitiveFromMesh(const shape_msgs::Mesh & mesh, const geometry_msgs::Pose & mesh_pose,
        unsigned char shape_type, shape_msgs::SolidPrimitive & shape, geometry_msgs::Pose & shape_pose)
{
    if(mesh.vertices.empty())
        return false;

    double xmin = HUGE_VAL;
    double ymin = HUGE_VAL;
    double zmin = HUGE_VAL;
    double xmax = -HUGE_VAL;
    double ymax = -HUGE_VAL;
    double zmax = -HUGE_VAL;

    forEach(const geometry_msgs::Point & pt, mesh.vertices) {
        if(pt.x > xmax)
            xmax = pt.x;
        if(pt.x < xmin)
            xmin = pt.x;
        if(pt.y > ymax)
            ymax = pt.y;
        if(pt.y < ymin)
            ymin = pt.y;
        if(pt.z > zmax)
            zmax = pt.z;
        if(pt.z < zmin)
            zmin = pt.z;
    }

    double wx = xmax - xmin;
    double wy = ymax - ymin;
    double wz = zmax - zmin;

    // For the transforms: The mesh is at 0,0,0 in the mesh_pose.
    // vertices might lie anywhere relative to that
    // The shape has a distinctive center
    // First figure that out in the mesh pose and transform to get the shape pose
    double cx = 0.5 * (xmax + xmin);
    double cy = 0.5 * (ymax + ymin);
    double cz = 0.5 * (zmax + zmin);
    tf::Pose mesh_pose_tf;
    tf::poseMsgToTF(mesh_pose, mesh_pose_tf);
    tf::Pose shape_in_mesh(tf::Quaternion(0, 0, 0, 1), tf::Vector3(cx, cy, cz));
    tf::Pose shape_pose_tf = mesh_pose_tf * shape_in_mesh;
    tf::poseTFToMsg(shape_pose_tf, shape_pose);

    shape.type = shape_type;
    if(shape_type == shape_msgs::SolidPrimitive::BOX) {
        shape.dimensions.resize(3);
        shape.dimensions[shape_msgs::SolidPrimitive::BOX_X] = wx;
        shape.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = wy;
        shape.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = wz;
        return true;
    } else if(shape_type == shape_msgs::SolidPrimitive::CYLINDER) {
        shape.dimensions.resize(2);
        shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = std::max(wx, wy);
        shape.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = wz;
        return true;

        // When done with these: Next step is: Use from CO with mesh (the coke) - need the intermeditae pkg
        // that will later also link other stuff if e.g. DB info is not there
        // That thing will become the grasp "planner".
    }

    ROS_ERROR("%s: for shape_type: %d not implemented.", __func__, shape_type);
    return false;
}


}

