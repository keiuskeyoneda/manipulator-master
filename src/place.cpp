#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <bits/stdc++.h>

#define N 1
#define S 0
#define E 0//186//188

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointClient;

typedef struct { double px, py, pz, r11, r12, r13, r21, r22, r23, r31, r32, r33; } fk;
typedef struct { double t[5]; } ik;

    int SIZE;   //dataのサイズを保存する関数
    int c;      //逆運動学と順運動学が一致していない数をカウント

class VS087
{
    public:
        JointClient* joint_client_;

        VS087()
        {
            // Gazebo 確認用
            //joint_client_ = new JointClient("/dual_manipulator/arm_controller/follow_joint_trajectory", true);
            joint_client_ = new JointClient("/vs087/arm_controller/follow_joint_trajectory", true);
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
        control_msgs::FollowJointTrajectoryGoal PositionTrajectory(ik ang1[], /*ik ang2[], */double t)
        {
            control_msgs::FollowJointTrajectoryGoal goal;

            // 始まりの時間を定義
            goal.trajectory.header.stamp = ros::Time::now();

            // 関節の名前を定義　configに書かれているものに揃える
            goal.trajectory.joint_names.push_back("joint_1");
            goal.trajectory.joint_names.push_back("joint_2");
            goal.trajectory.joint_names.push_back("joint_3");
            goal.trajectory.joint_names.push_back("joint_4");
            goal.trajectory.joint_names.push_back("joint_5");
            goal.trajectory.joint_names.push_back("joint_6");
          /*  goal.trajectory.joint_names.push_back("joint_1_2");
            goal.trajectory.joint_names.push_back("joint_2_2");
            goal.trajectory.joint_names.push_back("joint_3_2");
            goal.trajectory.joint_names.push_back("joint_4_2");
            goal.trajectory.joint_names.push_back("joint_5_2");
            goal.trajectory.joint_names.push_back("joint_6_2");*/


//std::cout << SIZE/80<< std::endl;

// 経由点の数分だけ初期化
      goal.trajectory.points.resize(SIZE/N);//経由店の数だけ数字を増やす必要あり


          for(int i = 0; i < SIZE/N; i++){

            //std::cout << i << std::endl;

            // 目標角度　関節の数分だけ初期化
            goal.trajectory.points[i].positions.resize(6/*12*/);
            //Arm1
            goal.trajectory.points[i].positions[0] = ang1[i*N].t[0];
            goal.trajectory.points[i].positions[1] = ang1[i*N].t[1];
            goal.trajectory.points[i].positions[2] = ang1[i*N].t[2];
            goal.trajectory.points[i].positions[3] = ang1[i*N].t[3];
            goal.trajectory.points[i].positions[4] = ang1[i*N].t[4];
            goal.trajectory.points[i].positions[5] = ang1[i*N].t[5];

            //Arm2
  /*          goal.trajectory.points[i].positions[6] = ang2[i*N].t[0];
            goal.trajectory.points[i].positions[7] = ang2[i*N].t[1];
            goal.trajectory.points[i].positions[8] = ang2[i*N].t[2];
            goal.trajectory.points[i].positions[9] = ang2[i*N].t[3];
            goal.trajectory.points[i].positions[10] = ang2[i*N].t[4];
            goal.trajectory.points[i].positions[11] = ang2[i*N].t[5];*/


            // 目標角度における速度　基本0
            goal.trajectory.points[i].velocities.resize(6/*12*/);

           if (i+1 == SIZE/N) {
              /* code */
             printf("i+1 == SIZE/N\n");
              for (int j = 0; j < 6/*12*/; j++)
              {
                  goal.trajectory.points[i].velocities[j] = 0.0;
              }
            }


           else if (i+1 > SIZE/N) {
                          /* code */
              printf("i+1 > SIZE/N\n");
              for (int j = 0; j < 6/*12*/; j++)
              {
                  goal.trajectory.points[i].velocities[j] = 0.0;
              }
            }

            else {

              for (int j = 0; j < 6/*12*/; j++)
              {
                  goal.trajectory.points[i].velocities[j] = 0.01;
              }

            }

            // 目標角度に何秒で到達するか　引数として指定
            goal.trajectory.points[i].time_from_start = ros::Duration(t*(i+1));
          }
/*
            // 目標角度における速度　基本0
            goal.trajectory.points[0].velocities.resize(12);
            goal.trajectory.points[1].velocities.resize(12);

            for (int i = 0; i < 12; i++)
            {
                goal.trajectory.points[0].velocities[i] = 1.0;
                goal.trajectory.points[1].velocities[i] = 0.0;
            }

            // 目標角度に何秒で到達するか　引数として指定
            goal.trajectory.points[0].time_from_start = ros::Duration(t);
            goal.trajectory.points[1].time_from_start = ros::Duration(t)+ros::Duration(t);
*/
            return goal;
        }


        // 引数 : 目標への到達時間(秒)
        // Gazeboで試す場合は目標角度の情報のみでよい
        // 実機に移す場合は現在角度の情報が必要(要追加記述)
        control_msgs::FollowJointTrajectoryGoal JointTrajectory(ik an1, /*ik an2,*/ double t)
        {
            control_msgs::FollowJointTrajectoryGoal goal;

            // 始まりの時間を定義
            goal.trajectory.header.stamp = ros::Time::now();

            // 関節の名前を定義　configに書かれているものに揃える
            goal.trajectory.joint_names.push_back("joint_1");
        //    goal.trajectory.joint_names.push_back("joint_1_2");
            goal.trajectory.joint_names.push_back("joint_2");
        //     goal.trajectory.joint_names.push_back("joint_2_2");
            goal.trajectory.joint_names.push_back("joint_3");
        //    goal.trajectory.joint_names.push_back("joint_3_2");
            goal.trajectory.joint_names.push_back("joint_4");
        //    goal.trajectory.joint_names.push_back("joint_4_2");
            goal.trajectory.joint_names.push_back("joint_5");
        //    goal.trajectory.joint_names.push_back("joint_5_2");
            goal.trajectory.joint_names.push_back("joint_6");
        //    goal.trajectory.joint_names.push_back("joint_6_2");

            // 経由点の数分だけ初期化
            goal.trajectory.points.resize(1);

            // 目標角度　関節の数分だけ初期化
          /*  goal.trajectory.points[0].positions.resize(12);
            goal.trajectory.points[0].positions[0] = an1.t[0];
            goal.trajectory.points[0].positions[1] = an2.t[0];
            goal.trajectory.points[0].positions[2] = an1.t[1];
            goal.trajectory.points[0].positions[3] = an2.t[1];
            goal.trajectory.points[0].positions[4] = an1.t[2];
            goal.trajectory.points[0].positions[5] = an2.t[2];
            goal.trajectory.points[0].positions[6] = an1.t[3];
            goal.trajectory.points[0].positions[7] = an2.t[3];
            goal.trajectory.points[0].positions[8] = an1.t[4];
            goal.trajectory.points[0].positions[9] = an2.t[4];
            goal.trajectory.points[0].positions[10] = an1.t[5];
            goal.trajectory.points[0].positions[11] = an2.t[5];*/

              goal.trajectory.points[0].positions.resize(6);
              goal.trajectory.points[0].positions[0] = an1.t[0];
              goal.trajectory.points[0].positions[1] = an1.t[1];
              goal.trajectory.points[0].positions[2] = an1.t[2];
              goal.trajectory.points[0].positions[3] = an1.t[3];
              goal.trajectory.points[0].positions[4] = an1.t[4];
              goal.trajectory.points[0].positions[5] = an1.t[5];

            // 目標角度における速度　基本0
            goal.trajectory.points[0].velocities.resize(6);
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

////////////////////////////////////////////////////////////////////////////////////////////

//追加

//文字列のsplit機能
std::vector<std::string> split(std::string str, char del) {
    int first = 0;
    int last = str.find_first_of(del);
    std::vector<std::string> result;
    while (first < str.size()) {
        std::string subStr(str, first, last - first);
        result.push_back(subStr);
        first = last + 1;
        last = str.find_first_of(del, first);
        if (last == std::string::npos) {
            last = str.size();
        }
    }
    return result;
}

std::vector<std::vector<std::string> >
csv2vector(std::string filename, int ignore_line_num = 1){
    //csvファイルの読み込み
    std::ifstream reading_file;
    reading_file.open(filename, std::ios::in);
    if(!reading_file){
        std::vector<std::vector<std::string> > data;
        return data;
    }
    std::string reading_line_buffer;
    //最初のignore_line_num行を空読みする
    for(int line = 0; line < ignore_line_num; line++){
        getline(reading_file, reading_line_buffer);
        if(reading_file.eof()) break;
    }

    //二次元のvectorを作成
    std::vector<std::vector<std::string> > data;
    while(std::getline(reading_file, reading_line_buffer)){
        if(reading_line_buffer.size() == 0) break;
        std::vector<std::string> temp_data;
        temp_data = split(reading_line_buffer, ',');
        data.push_back(temp_data);
    }
    return data;

}

////////////////////////////////////////////////////////////////////////////////////////////




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
    ik T;

    double c1, c2, c3, c4, c5, c6;
    double s1, s2, s3, s4, s5, s6;
    double c23, s23;

    double K,L,M;

    fk P;

    a1 = 30.0;
    a3 = -445.0;
    a4 = -20.0;
    d1 = 197.5;
    d2 = 197.5;
    d5 = 430.0;

    t1 = atan2(py, px);

    c1 = cos(t1);
    s1 = sin(t1);

    K = -( pow((-a1 + c1*px + s1*py),2) + pow((pz - d1 - d2),2) - pow(a3,2) - pow(a4,2) - pow(d5,2) ) / ( 2*a3 );

    t3 = atan2( d5,a4 ) - atan2(K, sqrt(pow(d5,2)+pow(a4,2)-pow(K,2)));

    c3 = cos(t3);
    s3 = sin(t3);

    L = a3 + s3*a4 - c3*d5;
    M = (pow((-pz + d1 + d2),2) + pow((-a1 + c1*px + s1*py),2) - pow(L,2));

    t2 = atan2( (-pz + d1 + d2), (-a1 + c1*px + s1*py) ) - atan2( L,sqrt(M));

  //  printf("%lf\n",sqrt(pow((-pz + d1 + d2),2) + pow((-a1 + c1*px + s1*py),2) - pow(L,2)) );
  //  printf("%lf\n",pow((-pz + d1 + d2),2) + pow((-a1 + c1*px + s1*py),2) - pow(L,2) );
  //  printf("%lf %lf %lf %lf\n", L, s3, c3, c1) ;
//   printf("%lf %lf\n", pow((-a1 + c1*px + s1*py),2), pow(L,2)) ;

    c23 = cos(t2 + t3);
    s23 = sin(t2 + t3);

    t5 = acos( (c1*r13+s1*r23)*s23 + r33*c23 );

    t4 = atan2( (-s1*r13+c1*r23), (c1*r13+s1*r23)*c23 - r33*s23 );

    t6 = atan2( (c1*r12+s1*r22)*s23 + r32*c23, -(c1*r11+s1*r21)*s23 - r31*c23 );

    if(t1 < -2.96705972839036 || t1 > 2.96705972839036)
    {
        ROS_ERROR("t1 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        printf("%f,%f,%f\n",px,py,pz);
        ros::shutdown();
    }
    else if(t2 < -1.74532925199433 || t2 > 2.35619449019234)
    {
        ROS_ERROR("t2 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        printf("%f,%f,%f\n",px,py,pz);
        ros::shutdown();
    }
    else if(t3 < -2.37364778271229 || t3 > 2.67035375555132)
    {
        ROS_ERROR("t3 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        printf("%f,%f,%f\n",px,py,pz);
        ros::shutdown();
    }
    else if(t4 < -4.71238898038469 || t4 > 4.71238898038469)
    {
        ROS_ERROR("t4 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        printf("%f,%f,%f\n",px,py,pz);
        ros::shutdown();
    }
    else if(t5 < -2.0943951023932 || t5 > 2.0943951023932)
    {
        ROS_ERROR("t5 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        printf("%f,%f,%f\n",px,py,pz);
        ros::shutdown();
    }
    else if(t6 < -6.28318530717959 || t6 > 6.28318530717959)
    {
        ROS_ERROR("t6 is out of range!!!");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        printf("%f,%f,%f\n",px,py,pz);
        ros::shutdown();
    }


    P = forward_kin(t1, t2, t3, t4, t5, t6);

    if(abs(P.px-px)<1.0e-11 && abs(P.py-py)<1.0e-11 && abs(P.pz-pz)<1.0e-11/* && P.r11==r11 && P.r12==r12 && P.r13==r13 && P.r21==r21 && P.r22==r22 && P.r23==r23 && P.r31==r31 && P.r32==r32 && P.r33==r33*/){
      T.t[0] = t1;
      T.t[1] = t2;
      T.t[2] = t3;
      T.t[3] = t4;
      T.t[4] = t5;
      T.t[5] = t6;
    }

    else {
      c++;
      printf("no match [%d]\n",c);
      std::cout<<P.px-px<<","<<P.py-py<<","<<P.pz-pz<<std::endl;
      printf("////////////////////////////////////\n");

      T.t[0] = t1;
      T.t[1] = t2;
      T.t[2] = t3;
      T.t[3] = t4;
      T.t[4] = t5;
      T.t[5] = t6;

    }


  //  printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
    return T;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "VS087_joint_action");

    VS087 arm;

    bool time_out;

    c = 0;

 ////////////////////////////////////////////////////////////////////////////////////////////

//csv2vectorの実行
    std::string filename = "line_test.csv";
    std::vector<std::vector<std::string> > data = csv2vector(filename, 2);//第2引数に指定した数だけその行分読み飛ばす
    ik right[data.size()];// data.sizeはvector関数専用
    ik left[data.size()];//rightは右手,leftは左手



        SIZE = data.size() - S - E;
        std::cout << SIZE << std::endl;


////////////////////////////////////////////////////////////////////////////////////////////////
/*マニピュレータの配置決定*/

  printf("start\n" );

//マニピュレータの動作範囲設定

double x, y, z;         //マニピュレータの位置
double x_2, y_2, z_2;   //マニピュレータの2軸の位置
double max_mani_x, max_mani_y, max_mani_z;

//vs087の場合
//マニピュレータの動作範囲の限界

  max_mani_x = 1751.0;
  max_mani_y = 1751.0;
  max_mani_z = 1620.6;

double max_xy=0.0, min_xy=0.0;
double max_x, min_x, max_y, min_y, max_z, min_z;  //dataの範囲
double max_zz, min_zz, range_zz;    //あるxy地点でのzの最大値と最小値と範囲
double range_xy;    //あるzでのxy方向のデータの範囲
double range_x, range_y, range_z;     //x,y,z方向のデータの範囲
double range, max_r;

    max_x=min_x=max_y=min_y=max_z=min_z=0.0;

//dataのx,y,zそれぞれの範囲を求める
  for(int i=0; i < SIZE; i++){
//z
    if (stod(data[i][10])>max_z) {
      max_z = stod(data[i][10]);
    }
    else if (stod(data[i][10])<min_z) {
      min_z = stod(data[i][10]);
    }
  }

    range_z = max_z - min_z;

double X = min_x, Y = min_y, Z = min_z;
double Z_s[SIZE], Z_e[SIZE];
int b = 0, max_b = 0;
//double Z_r[SIZE][SIZE];
double Z_0=min_z;

double a = 1.0;
int I = 0;
int n = 0;
int next;
double near_z;
double r_max = 0.0;
double r_min = 10.0e10;
int check=0;
int f;


//最も離れた２点を見つける
for ( int i = 0; i < SIZE; i++) {
  for ( int j = 0; j < SIZE; j++) {

    range = sqrt(pow(stod(data[i][8]) - stod(data[j][8]),2) + pow(stod(data[i][9]) - stod(data[j][9]),2) + pow(stod(data[i][10]) - stod(data[j][10]),2));

    if (range > max_r) {
      max_r = range;
//２点のxyの中点にマニピュレータを配置
      x_2 = (stod(data[i][8]) + stod(data[j][8]))/2.0;
      y_2 = (stod(data[i][9]) + stod(data[j][9]))/2.0;
    }

  }
}

//一つのマニピュレータで足りないならエラーを出力
if (max_r > max_mani_x) {
  printf("need more manipulator\n");
}

while(f < 10e10){

for (z_2 = min_z; z_2 <= max_z; z_2 += a) {

  for (int i = 0; i < SIZE; i++) {
//マニピュレータの動作範囲と照らし合わせる
//範囲外のものが１つでもないか確認する
    if (stod(data[i][10]) > z_2 - 152.0) {     //２軸 ±100°までを動作範囲とする
      if (sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - z_2,2)) >= 870.0||sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - z_2,2)) <= 309.3) {

        if (r_min > sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - z_2,2))) {
          r_min = sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - z_2,2));
          near_z = z_2;
          check = 1;
        }
        else{
          check = 2;
        }
      }
    }
    else {
      if (r_min > sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - z_2,2))) {
        r_min = sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - z_2,2));
        near_z = z_2;
        check = 1;
      }
      else{
        check = 2;
      }
    }

  }

  if (check == 0) {

//こんなややこしいことしてるけど２次元配列にすればいいのでは？
        if (z_2 - Z_0 > a) {
          std::cout<<Z<<std::endl<<"/"<<std::endl;
          Z_e[b] = Z;
          b++;
          max_b = b;
          n = 0;
        }
        Z_0 = z_2;
        Z = z_2;
        if (n == 0) {
          std::cout<<Z<<std::endl;
          Z_s[b] = Z;
        }
        n++;


  //これだとZ_r[0][I]に数値が入らない場合がある
    /*    if (z_2 - Z_0 > a) {
          b++;
          max_b = b;

          I = 0;
        }
        Z_0 = z_2;
        Z_r[b][I] = z_2;
        I++;
        */
      }
}



//動作範囲に入っているかを判定
if (n == 0){    //入ってない場合
  for(int i = 0; i < SIZE; i++){
   if (r_max < sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - near_z,2))) {
    r_max = sqrt(pow(stod(data[i][8]) - x_2,2) + pow(stod(data[i][9]) - y_2,2) + pow(stod(data[i][10]) - z_2,2));
    next = i;
    }
  }

//ギリギリになるように配置しないほうがいい...

  double theta = atan2( stod(data[next][9]) - y_2, stod(data[next][8]) - x_2);
  x_2 = x_2 + (sqrt(pow(stod(data[next][8]) - x_2,2) + pow(stod(data[next][9]) - y_2,2))-850)*cos(theta);
  y_2 = y_2 + (sqrt(pow(stod(data[next][8]) - x_2,2) + pow(stod(data[next][9]) - y_2,2))-850)*sin(theta);

}

else{           //入っている場合
  if (b==0) {  //１回も範囲が区切られていない場合
    Z_e[b] = Z;
  }

    for ( b = 0; b <= max_b; b++) {
      std::cout<<Z_s[b]<<"~"<<Z_e[b]<<std::endl;
    }
    //f = 1000;
    break;//のほうがいいかも
  }
  f++;
  //std::cout<<f<<std::endl;
}



/////////////////////////////////////////////////////////////////////////////////////////////////
//動作

    for(int i = 0; i < SIZE; i+=N){

      //right[i] = inverse_kin(stod(data[i][8]) - stod(data[0][2]) +30, stod(data[i][9]) - stod(data[0][3]) - 245, +stod(data[i][10]) - stod(data[0][4]) + 595,0,0,1,0,1,0,-1,0,0);
      right[i] = inverse_kin(stod(data[i+S][8]), stod(data[i+S][9]) - stod(data[0][3]), +stod(data[i+S][10]),0,0,1,0,1,0,-1,0,0);

       //std::vector<std::vector<double>> right
      //left[i] = inverse_kin(stod(data[i][17]) - stod(data[0][11]) +30, stod(data[i][18]) - stod(data[0][12]) + 245, +stod(data[i][19]) - stod(data[0][13]) + 595,0,0,1,0,1,0,-1,0,0);
      left[i] = inverse_kin(stod(data[i+S][17]), stod(data[i+S][18]) - stod(data[0][12]), +stod(data[i+S][19]),0,0,1,0,1,0,-1,0,0);

  //     std::cout << stod(data[i+S][8])<< ","<<  stod(data[i+S][9])<< ","<< stod(data[i+S][10])<< std::endl;
    }
    //二次元配列dataの標準出力への出力
  /*  for(int i = 0; i < data.size(); i+=3){
        for(int j = 0; j < data[i].size(); j++){
            std::cout << data[i][j] << ",";
        }
        std::cout << std::endl;//改行を出力

    }*/

    // Phase 1

  ROS_INFO("Phase 1 starts.");

  ik init;

  init.t[0] = 0.0;
  init.t[1] = 0.0;
  init.t[2] = M_PI/2;
  init.t[3] = 0.0;
  init.t[4] = 0.0;
  init.t[5] = 0.0;


    arm.startJointTrajectory(arm.JointTrajectory(init, /*init,*/ 4.0));

     time_out = arm.joint_client_->waitForResult(ros::Duration(10.0));

     if (!time_out) {
     ROS_WARN("Phase 1 is time out.");
     return 0;
     }
     else {
     ROS_INFO("Phase 1 is completed.");
     }

     //Phase 2

     ROS_INFO("Phase 2 starts.");


    //   arm.startJointTrajectory(arm.JointTrajectory(right[0], /*left[0],*/ 5.0));

        time_out = arm.joint_client_->waitForResult(ros::Duration(10.0));

        if (!time_out) {
        ROS_WARN("Phase 2 is time out.");
        return 0;
        }
        else {
        ROS_INFO("Phase 2 is completed.");
        }



    // Phase 3

  ROS_INFO("Phase 3 starts.");



    ////////////////////////////////////////////////////////////////////////////////////////////
    // arm.startJointTrajectory(arm.JointTrajectory(2));
   arm.startJointTrajectory(arm.PositionTrajectory(right,/*left,*/1));

    time_out = arm.joint_client_->waitForResult(ros::Duration(200.0));

    if (!time_out) {
		ROS_WARN("Phase 3 is time out.");
		return 0;
	}
	else {
		ROS_INFO("Phase 3 is completed.");
	}


    ROS_INFO("All phases are completed.");


    return 0;
}
