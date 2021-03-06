#define _USE_MATH_DEFINES
#include <cmath>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointClient;

typedef struct { double px, py, pz, r11, r12, r13, r21, r22, r23, r31, r32, r33; } fk;
typedef struct { double t1, t2, t3, t4, t5, t6; } ik;

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

        // 軌道生成用の関数
        // このプログラムでは始点と終点の情報のみを指定している
        // 経由点の情報を追加すれば５次補間による直線軌道等も実現可能？

        // 引数 : 目標角度(Arm1)　目標角度(Arm2) 目標への到達時間(秒)
        // Gazeboで試す場合は目標角度の情報のみでよい
        // 実機に移す場合は現在角度の情報が必要(要追加記述)
        control_msgs::FollowJointTrajectoryGoal PositionTrajectory(ik ang1, ik ang2, double t)
        {
            control_msgs::FollowJointTrajectoryGoal goal;

            // 始まりの時間を定義
            goal.trajectory.header.stamp = ros::Time::now();

            // 関節の名前を定義　configに書かれているものに揃える
            goal.trajectory.joint_names.push_back("joint_1_1");
            goal.trajectory.joint_names.push_back("joint_2_1");
            goal.trajectory.joint_names.push_back("joint_3_1");
            goal.trajectory.joint_names.push_back("joint_4_1");
            goal.trajectory.joint_names.push_back("joint_5_1");
            goal.trajectory.joint_names.push_back("joint_6_1");
            goal.trajectory.joint_names.push_back("joint_1_2");
            goal.trajectory.joint_names.push_back("joint_2_2");
            goal.trajectory.joint_names.push_back("joint_3_2");
            goal.trajectory.joint_names.push_back("joint_4_2");
            goal.trajectory.joint_names.push_back("joint_5_2");
            goal.trajectory.joint_names.push_back("joint_6_2");

            // 経由点の数分だけ初期化
            goal.trajectory.points.resize(1);

            // 目標角度　関節の数分だけ初期化
            goal.trajectory.points[0].positions.resize(12);

            // 目標角度　こちらから指定する　このプログラムでは逆運動学の解を受け渡すようになっている
            //Arm1
            goal.trajectory.points[0].positions[0] = ang1.t1;
            goal.trajectory.points[0].positions[1] = ang1.t2;
            goal.trajectory.points[0].positions[2] = ang1.t3;
            goal.trajectory.points[0].positions[3] = ang1.t4;
            goal.trajectory.points[0].positions[4] = ang1.t5;
            goal.trajectory.points[0].positions[5] = ang1.t6;

            //Arm2
            goal.trajectory.points[0].positions[6] = ang2.t1;
            goal.trajectory.points[0].positions[7] = ang2.t2;
            goal.trajectory.points[0].positions[8] = ang2.t3;
            goal.trajectory.points[0].positions[9] = ang2.t4;
            goal.trajectory.points[0].positions[10] = ang2.t5;
            goal.trajectory.points[0].positions[11] = ang2.t6;

            // 目標角度における速度　基本0
            goal.trajectory.points[0].velocities.resize(12);
            for (int i = 0; i < 12; i++)
            {
                goal.trajectory.points[0].velocities[i] = 0.0;
            }

            // 目標角度に何秒で到達するか　引数として指定
            goal.trajectory.points[0].time_from_start = ros::Duration(t);

            return goal;
        }


        // 引数 : 目標への到達時間(秒)
        // Gazeboで試す場合は目標角度の情報のみでよい
        // 実機に移す場合は現在角度の情報が必要(要追加記述)
        control_msgs::FollowJointTrajectoryGoal JointTrajectory(double t)
        {
            control_msgs::FollowJointTrajectoryGoal goal;

            // 始まりの時間を定義
            goal.trajectory.header.stamp = ros::Time::now();

            // 関節の名前を定義　configに書かれているものに揃える
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

            // 経由点の数分だけ初期化
            goal.trajectory.points.resize(1);

            // 目標角度　関節の数分だけ初期化
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

            // 目標角度における速度　基本0
            goal.trajectory.points[0].velocities.resize(12);
            for (int i = 0; i < 6; i++)
            {
                goal.trajectory.points[0].velocities[i] = 0.0;
            }

            // 目標角度に何秒で到達するか　引数として指定
            goal.trajectory.points[0].time_from_start = ros::Duration(t);

            return goal;
        }

        actionlib::SimpleClientGoalState getJointState()
        {
            return joint_client_->getState();
        }
};

// ロボットの順運動学を計算する関数
// 引数 : 目標角度(全部で6個)
fk forward_kin(double t1, double t2, double t3, double t4, double t5, double t6)
{
    double a1,a3,a4,d1,d2,d5;

    double px, py, pz;
    double r11, r12, r13;
    double r21, r22, r23;
    double r31, r32, r33;

    double c1, c2, c3, c4, c5, c6;
    double s1, s2, s3, s4, s5, s6;
    double c23, s23;

    c1 = std::cos(t1);
    s1 = std::sin(t1);
    c2 = std::cos(t2);
    s2 = std::sin(t2);
    c3 = std::cos(t3);
    s3 = std::sin(t3);
    c4 = std::cos(t4);
    s4 = std::sin(t4);
    c5 = std::cos(t5);
    s5 = std::sin(t5);
    c6 = std::cos(t6);
    s6 = std::sin(t6);

    c23 = std::cos(t2 + t3);
    s23 = std::sin(t2 + t3);

    a1 = 30.0;
    a3 = -445.0;
    a4 = -20.0;
    d1 = 197.5;
    d2 = 197.5;
    d5 = 430.0;

    r11 = ((c4*c5*c6-s4*s6)*c23-s5*c6*s23)*c1-(s4*c5*c6+c4*s6)*s1;
    r12 = ((c4*c5*c6-s4*s6)*c23-s5*c6*s23)*s1+(s4*c5*c6+c4*s6)*c1;
    r13 = -(c4*c5*c6-s4*s6)*s23-s5*c6*c23;

    r21 = ((-c4*c5*s6+s4*c6)*c23+s5*s6*s23)*c1-(-s4*c5*s6+c4*c6)*s1;
    r22 = ((-c4*c5*s6+s4*c6)*c23+s5*s6*s23)*s1+(-s4*c5*s6+c4*c6)*c1;
    r23 = (c4*c5*s6+s4*c6)*s23+s5*s6*c23;

    r31 = (c4*s5*c23+c5*s23)*c1-s4*s5*s1;
    r32 = (c4*s5*c23+c5*s23)*s1+s4*s5*c1;
    r33 = -c4*s5*s23+c5*c23;

    px = (a1-s2*a3+c23*a4+s23*d5)*c1;
    py = (a1-s2*a3+c23*a4+s23*d5)*s1;
    pz = d1+d2-c2*a3-s23*a4+c23*d5;

    return { px, py, pz, r11, r12, r13, r21, r22, r23, r31, r32, r33 };
}

// ロボットの逆運動学を計算する関数
// 引数 : 目標位置，姿勢(全部で12個)
ik inverse_kin(double px, double py, double pz, double r11, double r12, double r13, double r21, double r22, double r23, double r31, double r32, double r33)
{
    double a1,a3,a4,d1,d2,d5;

    double t1, t2, t3, t4, t5, t6;

    double c1, c2, c3, c4, c5, c6;
    double s1, s2, s3, s4, s5, s6;
    double c23, s23;

    double K,L;

    a1 = 30.0;
    a3 = -445.0;
    a4 = -20.0;
    d1 = 197.5;
    d2 = 197.5;
    d5 = 430.0;

    t1 = atan2(py, px);

    c1 = cos(t1);
    s1 = sin(t1);

    K = ( pow((-a1 + c1*px + s1*py),2) + pow((pz - d1 - d2),2) - pow(a3,2) - pow(a4,2) - pow(d5,2) ) / ( 2*a3 );

    t3 = atan2( d5,a4 ) - atan2(K, sqrt(pow(d5,2)+pow(a4,2)-pow(K,2)));

    c3 = cos(t3);
    s3 = sin(t3);

    L = a3 + s3*a4 - c3*d5;

    t2 = atan2( (-pz + d1 + d2), (-a1 + c1*px + s1*py) ) - atan2( L,sqrt(pow((-pz + d1 + d2),2) + pow((-a1 + c1*px + s1*py),2) - pow(L,2)) );

    c23 = cos(t2 + t3);
    s23 = sin(t2 + t3);

    t5 = acos( (c1*r13+s1*r23)*s23 + r33*c23 );

    t4 = atan2( (-s1*r13+c1*r23), (c1*r13+s1*r23)*c23 - r33*s23 );

    t6 = atan2( (c1*r12+s1*r22)*s23 + r32*c23, -(c1*r11+s1*r21)*s23 - r31*c23 ); 

    if(t1 < -2.96705972839036 || t1 > 2.96705972839036)
    {
        ROS_ERROR("t1 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        ros::shutdown();
    }  
    else if(t2 < -1.74532925199433 || t2 > 2.35619449019234)
    {
        ROS_ERROR("t2 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        ros::shutdown();
    }  
    else if(t3 < -2.37364778271229 || t3 > 2.67035375555132)
    {
        ROS_ERROR("t3 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        ros::shutdown();
    }  
    else if(t4 < -4.71238898038469 || t4 > 4.71238898038469)
    {
        ROS_ERROR("t4 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        ros::shutdown();
    }  
    else if(t5 < -2.0943951023932 || t5 > 2.0943951023932)
    {
        ROS_ERROR("t5 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        ros::shutdown();
    } 
    else if(t6 < -6.28318530717959 || t6 > 6.28318530717959)
    {
        ROS_ERROR("t6 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        ros::shutdown();
    } 

    printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
    return { t1, t2, t3, t4, t5, t6 };
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "VS087_joint_action");

    VS087 arm;

    bool time_out;

    // Phase 1 

	ROS_INFO("Phase 1 starts.");

    // arm.startJointTrajectory(arm.JointTrajectory(2));
    arm.startJointTrajectory(arm.PositionTrajectory(inverse_kin(100,100,1000,1,0,0,0,1,0,0,0,1),inverse_kin(100,-100,1000,1,0,0,0,1,0,0,0,1),10));
  
    time_out = arm.joint_client_->waitForResult(ros::Duration(15.0));

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
