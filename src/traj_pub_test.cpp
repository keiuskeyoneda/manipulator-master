#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointClient;

class VS087
{
    public:
        JointClient* joint_client_;

        VS087()
        {
            // Gazebo 確認用
            joint_client_ = new JointClient("/dual_manipulator/arm_controller/follow_joint_trajectory", true);
            //

            // 実機動作用
            // joint_client_ = new JointClient("joint_trajectory_action", true);
            //

            while(!joint_client_->waitForServer(ros::Duration(5.0)))
            {
                ROS_INFO("Waiting for the follow_joint_trajectory server");
            }
        }

        ~VS087()
        {
            delete joint_client_;
        }

        void startJointTrajectory(control_msgs::FollowJointTrajectoryGoal goal)
        {
            goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
            joint_client_->sendGoal(goal);
        }


        control_msgs::FollowJointTrajectoryGoal JointTrajectory(double jtime)
        {
            control_msgs::FollowJointTrajectoryGoal goal;

            goal.trajectory.header.stamp = ros::Time::now();

            goal.trajectory.joint_names.push_back("joint_1_1");
            goal.trajectory.joint_names.push_back("joint_1_2");
            goal.trajectory.joint_names.push_back("joint_2_1");
            goal.trajectory.joint_names.push_back("joint_2_2");
            goal.trajectory.joint_names.push_back("joint_3_1");
            goal.trajectory.joint_names.push_back("joint_3_2");
            goal.trajectory.joint_names.push_back("joint_4_1");
            goal.trajectory.joint_names.push_back("joint_4_2");
            goal.trajectory.joint_names.push_back("joint_5_1");
            goal.trajectory.joint_names.push_back("joint_5_2");
            goal.trajectory.joint_names.push_back("joint_6_1");
            goal.trajectory.joint_names.push_back("joint_6_2");

            goal.trajectory.points.resize(1);

            goal.trajectory.points[0].positions.resize(12);
            goal.trajectory.points[0].positions[0] = 0.5;
            goal.trajectory.points[0].positions[1] = -0.5;
            goal.trajectory.points[0].positions[2] = 0.5;
            goal.trajectory.points[0].positions[3] = 0.5;
            goal.trajectory.points[0].positions[4] = 0.5;
            goal.trajectory.points[0].positions[5] = 0.5;
            goal.trajectory.points[0].positions[6] = 0.5;
            goal.trajectory.points[0].positions[7] = 0.5;
            goal.trajectory.points[0].positions[8] = 0.5;
            goal.trajectory.points[0].positions[9] = 0.5;
            goal.trajectory.points[0].positions[10] = 0.5;
            goal.trajectory.points[0].positions[11] = 0.5;

            goal.trajectory.points[0].velocities.resize(12);
            for (int i = 0; i < 6; i++)
            {
                goal.trajectory.points[0].velocities[i] = 0.0;
            }

            goal.trajectory.points[0].accelerations.resize(12);
            for (int i = 0; i < 6; i++)
            {
                goal.trajectory.points[0].accelerations[i] = 0.0;
            }

            goal.trajectory.points[0].time_from_start = ros::Duration(jtime);

            return goal;
        }

        actionlib::SimpleClientGoalState getJointState()
        {
            return joint_client_->getState();
        }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "VS087_joint_action");
    ros::NodeHandle nh;

    VS087 arm;

    bool time_out;

    // Phase 1 (Move arm to the object above and open finger) //////////////////////////////////

	ROS_INFO("Phase 1 starts.");

    arm.startJointTrajectory(arm.JointTrajectory(2));
  
    time_out = arm.joint_client_->waitForResult(ros::Duration(10.0));

    if (!time_out) {
		ROS_WARN("Phase 1 is time out.");
		return 0;
	}
	else {
		ROS_INFO("Phase 1 is completed.");
	}

    ////////////////////////////////////////////////////////////////////////////////////////////

    ROS_INFO("All phases are completed.");


    return 0;
}
