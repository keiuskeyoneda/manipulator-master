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
#include <float.h>
#include <algorithm>
#include <limits>

#define N 1 //1にしてもだめだった　なぜ分母がいつも0になるのか　いい感じに戻らない　lineのほうで一回試してみる　か操作度があっているかどうか確認
#define S 0
#define E 0//186//188

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> JointClient;

typedef struct { double px, py, pz, r11, r12, r13, r21, r22, r23, r31, r32, r33; } fk;
typedef struct { double t[5]; } ik;

    int SIZE;   //dataのサイズを保存する関数
    int cc;      //逆運動学と順運動学が一致していない数をカウント
    int CC;

class VS087
{
    public:
        JointClient* joint_client_;

        VS087()
        {
            // Gazebo 確認用
            joint_client_ = new JointClient("/dual_manipulator/arm_controller/follow_joint_trajectory", true);
            //joint_client_ = new JointClient("/vs087/arm_controller/follow_joint_trajectory", true);
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
        control_msgs::FollowJointTrajectoryGoal PositionTrajectory(ik ang1[], ik ang2[], double t)
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
      goal.trajectory.points.resize(SIZE/N);//経由店の数だけ数字を増やす必要あり


          for(int i = 0; i < SIZE/N; i++){

            //std::cout << i << std::endl;

            // 目標角度　関節の数分だけ初期化
            goal.trajectory.points[i].positions.resize(12);
            //Arm1
            goal.trajectory.points[i].positions[0] = ang1[i*N].t[0];
            goal.trajectory.points[i].positions[1] = ang1[i*N].t[1];
            goal.trajectory.points[i].positions[2] = ang1[i*N].t[2];
            goal.trajectory.points[i].positions[3] = ang1[i*N].t[3];
            goal.trajectory.points[i].positions[4] = ang1[i*N].t[4];
            goal.trajectory.points[i].positions[5] = ang1[i*N].t[5];

            //Arm2
            goal.trajectory.points[i].positions[6] = ang2[i*N].t[0];
            goal.trajectory.points[i].positions[7] = ang2[i*N].t[1];
            goal.trajectory.points[i].positions[8] = ang2[i*N].t[2];
            goal.trajectory.points[i].positions[9] = ang2[i*N].t[3];
            goal.trajectory.points[i].positions[10] = ang2[i*N].t[4];
            goal.trajectory.points[i].positions[11] = ang2[i*N].t[5];


            // 目標角度における速度　基本0
            goal.trajectory.points[i].velocities.resize(12);

           if (i+1 == SIZE/N) {
              /* code */
             printf("i+1 == SIZE/N\n");
              for (int j = 0; j < 12; j++)
              {
                  goal.trajectory.points[i].velocities[j] = 0.0;
              }
            }


           else if (i+1 > SIZE/N) {
                          /* code */
              printf("i+1 > SIZE/N\n");
              for (int j = 0; j < 12; j++)
              {
                  goal.trajectory.points[i].velocities[j] = 0.0;
              }
            }

            else {

              for (int j = 0; j < 12; j++)
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
        control_msgs::FollowJointTrajectoryGoal JointTrajectory(ik an1, ik an2, double t)
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
            goal.trajectory.points[0].positions[11] = an2.t[5];

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

    c23 = cos(t2 + t3);
    s23 = sin(t2 + t3);

    t5 = acos( (c1*r13+s1*r23)*s23 + r33*c23 );

    t4 = atan2( (-s1*r13+c1*r23), (c1*r13+s1*r23)*c23 - r33*s23 );

    t6 = atan2( (c1*r12+s1*r22)*s23 + r32*c23, -(c1*r11+s1*r21)*s23 - r31*c23 );

    if(t1 < -2.96705972839036 || t1 > 2.96705972839036 || std::isnan(t1))
    {
        CC = 1;
    }
    else if(t2 < -1.74532925199433 || t2 > 2.35619449019234 || std::isnan(t2))
    {
        CC = 1;
    }
    else if(t3 < -2.37364778271229 || t3 > 2.67035375555132 || std::isnan(t3))
    {
        CC = 1;
    }
    else if(t4 < -4.71238898038469 || t4 > 4.71238898038469 || std::isnan(t4))
    {
        CC = 1;
    }
    else if(t5 < -2.0943951023932 || t5 > 2.0943951023932 || std::isnan(t5))
    {
        CC = 1;
    }
    else if(t6 < -6.28318530717959 || t6 > 6.28318530717959 || std::isnan(t6))
    {
        CC = 1;
    }


    P = forward_kin(t1, t2, t3, t4, t5, t6);
      for (int i = 0; i < SIZE/N; i++) {

    if(abs(P.px-px)<1.0e-11 && abs(P.py-py)<1.0e-11 && abs(P.pz-pz)<1.0e-11/* && P.r11==r11 && P.r12==r12 && P.r13==r13 && P.r21==r21 && P.r22==r22 && P.r23==r23 && P.r31==r31 && P.r32==r32 && P.r33==r33*/){
      T.t[0] = t1;
      T.t[1] = t2;
      T.t[2] = t3;
      T.t[3] = t4;
      T.t[4] = t5;
      T.t[5] = t6;
    }

    else {
      cc++;
      /*printf("no match [%d]\n",cc);
      std::cout<<P.px-px<<","<<P.py-py<<","<<P.pz-pz<<std::endl;
      printf("////////////////////////////////////\n");
*/
      T.t[0] = t1;
      T.t[1] = t2;
      T.t[2] = t3;
      T.t[3] = t4;
      T.t[4] = t5;
      T.t[5] = t6;

    }
  }
  //  printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
    return T;
}


double manipulatability(ik ang_1[], double min){    //最小の可操作度を返す

  double a1, a3, a4, d1, d2, d5;
  double M1,M2;
  std::vector <double> M;
  double M_min /* = DBL_MAX*/;
  double a, b, c, d, e, f;

  int g=0;

  a1 = 30.0;
  a3 = -445.0;
  a4 = -20.0;
  d1 = 197.5;
  d2 = 197.5;
  d5 = 430.0;

  for ( int i = 0; i < SIZE/N; i++) {

    if (g >= 1) {
      continue;
    }
    else {

      a = ang_1[i].t[0];
      b = ang_1[i].t[1];
      c = ang_1[i].t[2];
      d = ang_1[i].t[3];
      e = ang_1[i].t[4];
      f = ang_1[i].t[5];


M1 = abs(	(a3*(sin(c+b)*d5+a4*cos(c+b)-a3*sin(b)+a1)*(cos(b)*sin(c+b)*d5-sin(b)*cos(c+b)*d5+a4*sin(b)*sin(c+b)
+a4*cos(b)*cos(c+b))*sin(e)*(pow(cos(a),2)*pow(sin(c+b),2)*pow(sin(e),2)
+2*cos(a)*sin(a)*sin(c+b)*sin(d)*cos(e)*sin(e)
-2*pow(cos(a),2)*cos(c+b)*sin(c+b)*cos(d)*cos(e)*sin(e)
+pow(sin(a),2)*pow(sin(d),2)*pow(cos(e),2)
-2*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(cos(e),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(cos(d),2)*pow(cos(e),2)
-pow(cos(a),2)*pow(cos(c+b),2)*pow(sin(d),2)
+pow(sin(a),2)*pow(cos(d),2))*abs(sin(c+b)*cos(d)*sin(e)-cos(c+b)*cos(e)))
/((sin(c+b)*cos(d)*sin(e)-cos(c+b)*cos(e))
  *sqrt((pow(cos(a),2)*pow(sin(d),2)+2*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)
  +(pow(sin(c+b),2)+pow(sin(a),2)*pow(cos(c+b),2))*pow(cos(d),2))*pow(sin(e),2)
  +(2*cos(a)*sin(a)*sin(c+b)*sin(d)+(2*pow(sin(a),2)-2)*cos(c+b)*sin(c+b)*cos(d))*cos(e)*sin(e)
  +(pow(sin(a),2)*pow(sin(c+b),2)+pow(cos(c+b),2))*pow(cos(e),2))
*(pow(cos(a),2)*pow(sin(c+b),2)*pow(sin(e),2)*pow(sin(f),2)
+2*cos(a)*sin(a)*sin(c+b)*sin(d)*cos(e)*sin(e)*pow(sin(f),2)
-2*pow(cos(a),2)*cos(c+b)*sin(c+b)*cos(d)*cos(e)*sin(e)*pow(sin(f),2)
+pow(sin(a),2)*pow(sin(d),2)*pow(cos(e),2)
-2*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(cos(e),2)*pow(sin(f),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(cos(d),2)*pow(cos(e),2)*pow(sin(f),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(sin(d),2)*pow(sin(f),2)
+2*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(sin(f),2)
+pow(sin(a),2)*pow(cos(d),2)
+4*pow(cos(a),2)*cos(c+b)*sin(c+b)*sin(d)*sin(e)*cos(f)*sin(f)
+4*cos(a)*sin(a)*cos(c+b)*pow(sin(d),2)*cos(e)*cos(f)*sin(f)
-4*pow(cos(a),2)*pow(cos(c+b),2)*cos(d)*sin(d)*cos(e)*cos(f)*sin(f)+pow(cos(a),2)*pow(sin(c+b),2)*pow(sin(e),2)*pow(cos(f),2)
+2*cos(a)*sin(a)*sin(c+b)*sin(d)*cos(e)*sin(e)*pow(cos(f),2)
-2*pow(cos(a),2)*cos(c+b)*sin(c+b)*cos(d)*cos(e)*sin(e)*pow(cos(f),2)
-2*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(cos(e),2)*pow(cos(f),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(cos(d),2)*pow(cos(e),2)*pow(cos(f),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(sin(d),2)*pow(cos(f),2)
-2*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(cos(f),2)
)
));


  if(std::isnan(M1)){
    //std::cout<<M1<<std::endl;
    M1 = -1.0;
    g++;

  }

  if (M1 < min) {
    g++;
  }

    M.push_back(M1);

    }
  }

  return *std::min_element(M.begin(), M.end());
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "VS087_joint_action");

    VS087 arm;

    bool time_out;

    cc = 0;

 ////////////////////////////////////////////////////////////////////////////////////////////

//csv2vectorの実行
    std::string filename = "cap_s.csv";//"line_test.csv";
    std::vector<std::vector<std::string> > data = csv2vector(filename, 2);//第2引数に指定した数だけその行分読み飛ばす
    ik right[data.size()];// data.sizeはvector関数専用
    ik left[data.size()];//rightは右手,leftは左手

        SIZE = data.size() - S - E;
        std::cout << "datasize:"<<SIZE << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////
/*マニピュレータの配置決定*/
  printf("start\n" );

//check
/*
int checker=0;

for (int i = 0; i < SIZE/N; i++) {
inverse_kin(stod(data[i+S][8]) - stod(data[0][2]) -57, -stod(data[i+S][9]) + stod(data[0][3]) - 350/*+ 245*//*, -stod(data[i+S][10]) + 1400 + 750,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
if (CC == 1) {
  checker++;
}
}
if (checker >= 1) {
  printf("error\n");
  std::cout<<checker<<std::endl;
}
else{
  printf("ok\n");
}
*/
//マニピュレータの動作範囲設定

std::vector<double> x;
std::vector<double> y;
std::vector<double> z;        //マニピュレータの位置
double x_2_1, y_2_1, z_2_1, x_2_2, y_2_2, z_2_2;   //マニピュレータの2軸の位置
double max_mani_x, max_mani_y, max_mani_z;

//vs087の場合
//マニピュレータの動作範囲の限界

max_mani_x = 1751.0;
max_mani_y = 1751.0;
max_mani_z = 1620.6;

double max_x_1, min_x_1, max_y_1, min_y_1, max_z_1, min_z_1;  //dataの範囲
double max_x_2, min_x_2, max_y_2, min_y_2, max_z_2, min_z_2;
double range_x_1, range_y_1, range_z_1, range_x_2, range_y_2, range_z_2;     //x,y,z方向のデータの範囲
double range, max_r;

max_x_1=max_y_1=max_z_1=-10.0e100;
min_x_1=min_y_1=min_z_1=10.0e100;
max_x_2=max_y_2=max_z_2=-10.0e100;
min_x_2=min_y_2=min_z_2=10.0e100;

//dataのx,y,zそれぞれの範囲を求める
for(int i=0; i < SIZE/N; i++){
//1
  //x
  if (stod(data[i*N][8])>max_x_1) {
    max_x_1 = stod(data[i*N][8]);
  }
  if (stod(data[i*N][8])<min_x_1) {
    min_x_1 = stod(data[i*N][8]);
  }
  //y
  if (stod(data[i*N][9])>max_y_1) {
    max_y_1 = stod(data[i*N][9]);
  }
  if (stod(data[i*N][9])<min_y_1) {
    min_y_1 = stod(data[i*N][9]);
  }
  //z
  if (stod(data[i*N][10])>max_z_1) {
    max_z_1 = stod(data[i*N][10]);
  }
  if (stod(data[i*N][10])<min_z_1) {
    min_z_1 = stod(data[i*N][10]);
  }
//2
  //x
  if (stod(data[i*N][17])>max_x_2) {
    max_x_2 = stod(data[i*N][17]);
  }
  if (stod(data[i*N][17])<min_x_2) {
    min_x_2 = stod(data[i*N][17]);
  }
  //y
  if (stod(data[i*N][18])>max_y_2) {
    max_y_2 = stod(data[i*N][18]);
  }
  if (stod(data[i*N][18])<min_y_2) {
    min_y_2 = stod(data[i*N][18]);
  }
  //z
  if (stod(data[i*N][19])>max_z_2) {
    max_z_2 = stod(data[i*N][19]);
  }
  if (stod(data[i*N][19])<min_z_2) {
    min_z_2 = stod(data[i*N][19]);
  }
}

  range_x_1 = max_x_1 - min_x_1;
  range_y_1 = max_y_1 - min_y_1;
  range_z_1 = max_z_1 - min_z_1;

  range_x_2 = max_x_2 - min_x_2;
  range_y_2 = max_y_2 - min_y_2;
  range_z_2 = max_z_2 - min_z_2;

//2つのマニピュレータで足りないならエラーを出力

  if (range_x_1 > max_mani_x) {
    printf("need more manipulator[x]\n" );
  }
  if (range_y_1 > max_mani_y) {
    printf("need more manipulator[y]\n" );
  }
  if (range_z_1 > max_mani_z) {
    printf("need more manipulator[z]\n" );
  }

  if (range_x_2 > max_mani_x) {
    printf("need more manipulator[x]\n" );
  }
  if (range_y_2 > max_mani_y) {
    printf("need more manipulator[y]\n" );
  }
  if (range_z_2 > max_mani_z) {
    printf("need more manipulator[z]\n" );
  }
std::vector<std::vector<double> > X(2);
std::vector<std::vector<double> > Y(2);
std::vector<std::vector<double> > Z(2);
int bb_1 = 0, bb_2 = 0, max_bb_1, max_bb_2;

double a = 100.0;

double check_x[2], check_y[2], check_z[2];
int check_min[2]={1000,1000};
int check;


double x_max_1 = max_x_1 + 875.5;
double y_max_1 = max_y_1 + 875.5;
double z_max_1 = max_z_1 + 745.1;
double x_min_1 = min_x_1 - 875.5;
double y_min_1 = min_y_1 - 875.5;
double z_min_1 = min_z_1 - 875.5;

double x_max_2 = max_x_2 + 875.5;
double y_max_2 = max_y_2 + 875.5;
double z_max_2 = max_z_2 + 745.1;
double x_min_2 = min_x_2 - 875.5;
double y_min_2 = min_y_2 - 875.5;
double z_min_2 = min_z_2 - 875.5;

ik inv;

//マニピュレータの配置位置の座標を切りのいい数字にしたい

std::cout<<x_min_1-30.0<<"~"<<x_max_1-30.0<<":"<<y_min_1<<"~"<<y_max_1<<":"<<z_min_1+195.0<<"~"<<z_max_1+195.0<<std::endl;
std::cout<<x_min_2-30.0<<"~"<<x_max_2-30.0<<":"<<y_min_2<<"~"<<y_max_2<<":"<<z_min_2+195.0<<"~"<<z_max_2+195.0<<std::endl;

//ちゃんと繰り上げするコードを作ったほうがいい
if (x_min_1 > 0) {
  if (fmod(x_min_1,a)== 0) {
    x_min_1 = (int) ((x_min_1-30.0)/a)*a+30.0;
  }
  else{
  x_min_1 = (int) ((x_min_1-30.0)/a+1.0)*a+30.0;
  }
}
else{
  x_min_1 = (int) ((x_min_1-30.0)/a)*a+30.0;
}
if (y_min_1 > 0) {
  if (fmod(y_min_1,a)== 0) {
    y_min_1 = (int) (y_min_1/a)*a;
  }
  else{
  y_min_1 = (int) (y_min_1/a+1.0)*a;
  }
}
else{
  y_min_1 = (int) (y_min_1/a)*(a);
}
if (z_min_1 > 0) {
  if (fmod(z_min_1,a)== 0) {
    z_min_1 = (int) ((z_min_1+395.0)/a)*a-395.0;
  }
  else{
  z_min_1 = (int) ((z_min_1+395.0)/a+1.0)*a-395.0;
  }
}
else{
  z_min_1 = (int) ((z_min_1+395.0)/a)*a-395.0;
}


if (x_min_2 > 0) {
  if (fmod(x_min_2,a)== 0) {
    x_min_2 = (int) ((x_min_2-30.0)/a)*a+30.0;
  }
  else{
  x_min_2 = (int) ((x_min_2-30.0)/a+1.0)*a+30.0;
  }
}
else{
  x_min_2 = (int) ((x_min_2-30.0)/a)*a+30.0;
}
if (y_min_2 > 0) {
  if (fmod(y_min_2,a)== 0) {
    y_min_2 = (int) (y_min_2/a)*a;
  }
  else{
  y_min_2 = (int) (y_min_2/a+1.0)*a;
  }
}
else{
  y_min_2 = (int) (y_min_2/a)*(a);
}
if (z_min_2 > 0) {
  if (fmod(z_min_2,a)== 0) {
    z_min_2 = (int) ((z_min_2+395.0)/a)*a-395.0;
  }
  else{
  z_min_2 = (int) ((z_min_2+395.0)/a+1.0)*a-395.0;
  }
}
else{
  z_min_2 = (int) ((z_min_2+395.0)/a)*a-395.0;
}

/*if (z_min_1 <= 1000.0 - 195.0) {
  z_min_1 = 1000.0 - 195.0;
}
if (z_min_2 <= 1000.0 - 195.0) {
  z_min_2 = 1000.0 - 195.0;
}

z_max_1 = 2100.0 - 195.0;
z_max_2 = 2100.0 - 195.0;*/

std::cout<<x_min_1<<","<<y_min_1<<","<<z_min_1<<","<<x_min_2<<","<<y_min_2<<","<<z_min_2<<std::endl;

//どうにかして範囲を減らせないか
for (x_2_1 = x_min_1; x_2_1 <= x_max_1; x_2_1 += a) {
  for (y_2_1 = y_min_1; y_2_1 <= y_max_1; y_2_1 += a) {
    for (z_2_1 = z_min_1; z_2_1 <= z_max_1; z_2_1 += a) {

      check = 0;
//マニピュレータの動作範囲と照らし合わせる
//逆行列に入れて角度から動作範囲と照らし合わせる
//範囲外のものが１つでもないか確認する
//ロボットの自己接触も考慮
      for (int i = 0; i < SIZE/N; i++) {

        if (check <= 1) {
          continue;
        }
        else{

          if (sqrt(pow(stod(data[i*N][8]) - x_2_1,2) + pow(stod(data[i*N][9]) - y_2_1,2) + pow(stod(data[i*N][10]) - z_2_1,2)) <= 875.5){
            if(stod(data[i*N][10])-z_2_1 <= 224.5){
              if( sqrt(pow(stod(data[i*N][8]) - x_2_1,2) + pow(stod(data[i*N][9]) - y_2_1,2) + pow(stod(data[i*N][10]) - z_2_1,2)) > 224.5/*309.3*/) {
                CC = 0;
                inv = inverse_kin(stod(data[i*N+S][8])-x_2_1+30.0, -stod(data[i*N+S][9])+y_2_1, -stod(data[i*N+S][10])+z_2_1-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
                if (CC == 1) {
                  check++;
                }
              }
              else {
                check++;
              }
            }
            else if(stod(data[i*N][10])-z_2_1 < 400.0){
              if( sqrt(pow(stod(data[i*N][8]) - x_2_1,2) + pow(stod(data[i*N][9]) - y_2_1,2)) > 100.0/*224.5,309.3*/) {
                CC = 0;
                inv = inverse_kin(stod(data[i*N+S][8])-x_2_1+30.0, -stod(data[i*N+S][9])+y_2_1, -stod(data[i*N+S][10])+z_2_1-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
                if (CC == 1) {
                  check++;
                }
              }
              else {
                check++;
              }
            }
            else {
                CC = 0;

                inv = inverse_kin(stod(data[i*N+S][8])-x_2_1+30.0, -stod(data[i*N+S][9])+y_2_1, -stod(data[i*N+S][10])+z_2_1-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
                if (CC == 1) {
                  check++;
                }

              else {
                check++;
              }
            }
          }

          else {
            check++;
          }
        }
      }


      if (check == 0) {
          X[0].push_back(x_2_1);
          Y[0].push_back(y_2_1);
          Z[0].push_back(z_2_1);
        bb_1++;
        max_bb_1 = bb_1;
      }

      if (check < check_min[0]) {
        check_x[0] = x_2_1;
        check_y[0] = y_2_1;
        check_z[0] = z_2_1;
        check_min[0] = check;
      }

    }
  }
}

//std::cout<<"[1]"<<check_min[0]<<std::endl;


for (x_2_2 = x_min_2; x_2_2 <= x_max_2; x_2_2 += a) {
  for (y_2_2 = y_min_2; y_2_2 <= y_max_2; y_2_2 += a) {
    for (z_2_2 = z_min_2; z_2_2 <= z_max_2; z_2_2 += a) {


      check = 0;
//マニピュレータの動作範囲と照らし合わせる
//逆行列に入れて角度から動作範囲と照らし合わせる
//範囲外のものが１つでもないか確認する
//ロボットの自己接触も考慮
//ただし、少し大きめに設定している
      for (int i = 0; i < SIZE; i+=N) {

        if (check <= 1) {
          continue;
        }
        else{
          if (sqrt(pow(stod(data[i][17]) - x_2_2,2) + pow(stod(data[i][18]) - y_2_2,2) + pow(stod(data[i][19]) - z_2_2,2)) < 875.5){
            if(stod(data[i][19])-z_2_2 <= 224.5){
              if( sqrt(pow(stod(data[i][17]) - x_2_2,2) + pow(stod(data[i][18]) - y_2_2,2) + pow(stod(data[i][19]) - z_2_2,2)) > 309.3) {
                CC = 0;
                inverse_kin(stod(data[i+S][17])-x_2_2+30.0, -stod(data[i+S][18])+y_2_2, -stod(data[i+S][19])+z_2_2-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
                if (CC == 1) {
                  check++;
                }
              }
              else {
                check++;
              }
            }
            else if (stod(data[i][19])-z_2_2 <= 400.0) {
              if( sqrt(pow(stod(data[i][17]) - x_2_2,2) + pow(stod(data[i][18]) - y_2_2,2)) > 100.0) {
                CC = 0;
                inverse_kin(stod(data[i+S][17])-x_2_2+30.0, -stod(data[i+S][18])+y_2_2, -stod(data[i+S][19])+z_2_2-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
                if (CC == 1) {
                  check++;
                }
              }
              else {
                check++;
              }
            }
            else {
                inverse_kin(stod(data[i+S][17])-x_2_2+30.0, -stod(data[i+S][18])+y_2_2, -stod(data[i+S][19])+z_2_2-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
                if (CC == 1) {
                  check++;
                }
            }
          }

          else {
            check++;
          }
        }
      }


      if (check == 0) {
          X[1].push_back(x_2_2);
          Y[1].push_back(y_2_2);
          Z[1].push_back(z_2_2);
        bb_2++;
        max_bb_2 = bb_2;
      }

      if (check < check_min[1]) {
        check_x[1] = x_2_1;
        check_y[1] = y_2_1;
        check_z[1] = z_2_1;
        check_min[1] = check;
      }
    //  std::cout<<check<<std::endl;

    }
  }
}


//std::cout<<X[0][0]<<std::endl;

//std::cout<<"[2]"<<check_min[1]<<std::endl;

int bb_max_1[8];
int bb_max_2[8];
int bb_max11,bb_max12,bb_max13,bb_max14,bb_max15;
int bb_max21,bb_max22,bb_max23,bb_max24,bb_max25;
int bb_max16,bb_max26,bb_max17,bb_max27,bb_max18,bb_max28;
double pos_x[2][8], pos_y[2][8], pos_z[2][8];
double pair_max;
int ff;
int i_max, j_max;

//動作範囲に入っている位置が一つでもあるかを判定
if (bb_1 == 0 || bb_2 == 0){    //入ってない場合
  printf("need more manipulator\n" );
}

else{

//配置可能な場所を出力
  /*for (bb = 0; bb <= max_bb; bb++) {
    std::cout<<X[bb]<<","<<Y[bb]<<","<<Z[bb]<<std::endl;
  }*/

//可操作度を導入

  printf("start manipulatability\n" );

  ik mani_1[data.size()];
  ik mani_2[data.size()];
  double tmp;
  int tmp_bb;
  int dd=0;
  int max_dd;
  const double min_mani_max = 0.0;
  std::cout<<max_bb_1<<","<<max_bb_2<<std::endl;
  std::vector<std::vector<double> > min_mani(2);
  //double min_mani[max_bb_1][max_bb_2];//可操作度の最小
  printf("Now loading...\n" );

//可操作度の計算
//最小を(5個位)記憶してそれよりも小さい点があったらそこで中断するようにする
  for (bb_1 = 0; bb_1 < max_bb_1; bb_1++) {
      for(int i = 0; i < SIZE; i+=N){
        mani_1[i/N]
          =inverse_kin(stod(data[i+S][8])-(X[0][bb_1]-30.0),-stod(data[i+S][9])+(Y[0][bb_1]),-stod(data[i+S][10])+(Z[0][bb_1]-395.0),1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
      }
      min_mani[0].push_back(manipulatability(mani_1, min_mani_max));
      //std::cout<<manipulatability(mani_1)<<std::endl;sort
  }


/*
  std::cout<<"[11]"<<X[0][bb_max11]-30.0<<","<<Y[0][bb_max11]<<","<<Z[0][bb_max11]-395.0<<":"<<min_mani[0][bb_max11]<<std::endl;
  std::cout<<"[12]"<<X[0][bb_max12]-30.0<<","<<Y[0][bb_max12]<<","<<Z[0][bb_max12]-395.0<<":"<<min_mani[0][bb_max12]<<std::endl;
*/
    for (bb_2 = 0; bb_2 < max_bb_2; bb_2++) {
      for(int i = 0; i < SIZE; i+=N){
        mani_2[i/N] = inverse_kin(stod(data[i+S][17])-(X[1][bb_2]-30.0), -stod(data[i+S][18])+(Y[1][bb_2]), -stod(data[i+S][19])+(Z[1][bb_2]-395.0),1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));
      }
      min_mani[1].push_back(manipulatability(mani_2, min_mani_max));
    }

//std::cout<<"end manipulatability"<<std::endl;

    std::ofstream data_file1;
    std::string filename1 = "data_right2.csv";
    data_file1.open(filename1, std::ios::out);
    if (!data_file1.is_open()) {
      std::cout<<"~~~~~~~~~failed to open"<<filename1<<"~~~~~~~~~"<<std::endl;
    }

//printf("=====right=====\n");
for (z_2_1 = z_min_1; z_2_1 <= z_max_1; z_2_1 += a) {
    for (bb_1 = 0; bb_1 < max_bb_1; bb_1++) {
      if (min_mani[0][bb_1] > min_mani_max) {
        if (Z[0][bb_1] == z_2_1) {
          //std::cout<<X[0][bb_1]-30.0<<","<<Y[0][bb_1]<<","<<Z[0][bb_1]+195.0<<":"<<min_mani[0][bb_1]<<std::endl;
          data_file1<<X[0][bb_1]-30.0<<","<<Y[0][bb_1]<<","<<Z[0][bb_1]+195.0<<","<<min_mani[0][bb_1]<<std::endl;
        }
      }
    }
}
  data_file1.close();

  std::ofstream data_file2;
  std::string filename2 = "data_left2.csv";
  data_file2.open(filename2, std::ios::out);
  if (!data_file2.is_open()) {
    std::cout<<"~~~~~~~~~failed to open"<<filename2<<"~~~~~~~~~"<<std::endl;
  }

//printf("=====left=====\n");
for (z_2_2 = z_min_2; z_2_2 <= z_max_2; z_2_2 += a) {
  for (bb_2 = 0; bb_2 < max_bb_2; bb_2++) {
    if (min_mani[1][bb_2] > min_mani_max) {
      if (Z[1][bb_2] == z_2_2) {
        //std::cout<<X[1][bb_2]-30.0<<","<<Y[1][bb_2]<<","<<Z[1][bb_2]+195.0<<":"<<min_mani[1][bb_2]<<std::endl;
        data_file2<<X[1][bb_2]-30.0<<","<<Y[1][bb_2]<<","<<Z[1][bb_2]+195.0<<","<<min_mani[1][bb_2]<<std::endl;
      }
    }
  }
}
  data_file2.close();

  std::ofstream data_file12;
  std::string filename12 = "data_right22.csv";
  data_file12.open(filename12, std::ios::out);
  if (!data_file12.is_open()) {
    std::cout<<"~~~~~~~~~failed to open"<<filename12<<"~~~~~~~~~"<<std::endl;
  }
printf("=====right=====\n");
  for (z_2_1 = z_min_1; z_2_1 <= z_max_1; z_2_1 += a) {
      for (bb_1 = 0; bb_1 < max_bb_1; bb_1++) {
        if (min_mani[0][bb_1] > 0.5e7) {
          if (Z[0][bb_1] == z_2_1) {
            std::cout<<X[0][bb_1]-30.0<<","<<Y[0][bb_1]<<","<<Z[0][bb_1]+195.0<<":"<<min_mani[0][bb_1]<<std::endl;
            data_file12<<X[0][bb_1]-30.0<<","<<Y[0][bb_1]<<","<<Z[0][bb_1]+195.0<<","<<min_mani[0][bb_1]<<std::endl;
          }
        }
      }
  }

    data_file12.close();

    std::ofstream data_file22;
    std::string filename22 = "data_left22.csv";
    data_file22.open(filename22, std::ios::out);
    if (!data_file22.is_open()) {
      std::cout<<"~~~~~~~~~failed to open"<<filename22<<"~~~~~~~~~"<<std::endl;
    }


printf("=====left=====\n");
  for (z_2_2 = z_min_2; z_2_2 <= z_max_2; z_2_2 += a) {
    for (bb_2 = 0; bb_2 < max_bb_2; bb_2++) {
      if (min_mani[1][bb_2] > 0.5e7) {
        if (Z[1][bb_2] == z_2_2) {
          std::cout<<X[1][bb_2]-30.0<<","<<Y[1][bb_2]<<","<<Z[1][bb_2]+195.0<<":"<<min_mani[1][bb_2]<<std::endl;
          data_file22<<X[1][bb_2]-30.0<<","<<Y[1][bb_2]<<","<<Z[1][bb_2]+195.0<<","<<min_mani[1][bb_2]<<std::endl;
        }
      }
    }
  }
      data_file22.close();

}
/////////////////////////////////////////////////////////////////////////////////////////////////
//動作

    for(int i = 0; i < SIZE; i+=N){

      right[i] = inverse_kin(stod(data[i+S][8])-X[0][bb_max11]+30.0, -stod(data[i+S][9])+Y[0][bb_max11], -stod(data[i+S][10])+Z[0][bb_max11]-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));

      left[i] = inverse_kin(stod(data[i][17])-X[1][bb_max21]+30.0, -stod(data[i][18])+Y[1][bb_max21], -stod(data[i][19])+Z[1][bb_max21]-395.0,1/sqrt(2),0,1/sqrt(2),0,1,0,-1/sqrt(2),0,1/sqrt(2));

    }


    // Phase 1

  ROS_INFO("Phase 1 starts.");

  ik init;

  init.t[0] = 0.0;
  init.t[1] = 0.0;
  init.t[2] = M_PI/2;
  init.t[3] = 0.0;
  init.t[4] = 0.0;
  init.t[5] = 0.0;


    arm.startJointTrajectory(arm.JointTrajectory(init, init, 4.0));

     time_out = arm.joint_client_->waitForResult(ros::Duration(20.0));

     if (!time_out) {
     ROS_WARN("Phase 1 is time out.");
     return 0;
     }
     else {
     ROS_INFO("Phase 1 is completed.");
     }

     //Phase 2

     ROS_INFO("Phase 2 starts.");


       arm.startJointTrajectory(arm.JointTrajectory(right[0], left[0], 5.0));

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
   arm.startJointTrajectory(arm.PositionTrajectory(right,left,0.1));

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
