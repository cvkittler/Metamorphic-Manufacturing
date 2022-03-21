#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mmm_robot_workspace_generator");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroup group("manipulator");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface; 
  
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object.id = "roof_box";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 20.;
    primitive.dimensions[1] = 20.;
    primitive.dimensions[2] = 0.01;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0.;
    box_pose.position.y = 0.;
    box_pose.position.z =  1.;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    moveit_msgs::CollisionObject collision_object2;
    collision_object2.header.frame_id = group.getPlanningFrame();

    /* The id of the object is used to identify it. */
    collision_object2.id = "roof_box";

    /* Define a box to add to the world. */
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 20.;
    primitive2.dimensions[1] = 0.1;
    primitive2.dimensions[2] = 20.;

    /* A pose for the box (specified relative to frame_id) */
    geometry_msgs::Pose box_pose2;
    box_pose2.orientation.w = 1.0;
    box_pose2.position.x =  0.;
    box_pose2.position.y = 0.3;
    box_pose2.position.z =  0.;

    collision_object2.primitives.push_back(primitive);
    collision_object2.primitive_poses.push_back(box_pose);
    collision_object2.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects2;
    collision_objects2.push_back(collision_object2);
}

