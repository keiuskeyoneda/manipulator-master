#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <bits/stdc++.h>
#include <float.h>
#include <algorithm>
#include <limits>

#define N 2 //1にしてもだめだった　なぜ分母がいつも0になるのか　いい感じに戻らない　lineのほうで一回試してみる　か操作度があっているかどうか確認
#define S 0
#define E 0//186//188

using namespace std;


typedef struct { double px, py, pz, r11, r12, r13, r21, r22, r23, r31, r32, r33; } fk;
typedef struct { double t[5]; } ik;

    int SIZE;   //dataのサイズを保存する関数
    int cc;      //逆運動学と順運動学が一致していない数をカウント
    int CC;

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

  //  printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
    return T;
}


double manipulatability(ik ang_1[]/*, ik ang_2[], double min*/){    //最小の可操作度を返す

  double a1, a3, a4, d1, d2, d5;
  double M1,M2;
  std::vector <double> M;
  double M_min /* = DBL_MAX*/;
  double a, b, c, d, e, f;
  //double a_1, b_1, c_1, d_1, e_1, f_1;
  //double a_2, b_2, c_2, d_2, e_2, f_2;
  int g=0;

  a1 = 30.0;
  a3 = -445.0;
  a4 = -20.0;
  d1 = 197.5;
  d2 = 197.5;
  d5 = 430.0;

  for ( int i = 0; i < SIZE; i+=N) {

    /*if (g == 1) {
      continue;
    }
    else {*/
    /*  a_1 = ang_1[i].t[0];
      b_1 = ang_1[i].t[1];
      c_1 = ang_1[i].t[2];
      d_1 = ang_1[i].t[3];
      e_1 = ang_1[i].t[4];
      f_1 = ang_1[i].t[5];*/

      a = ang_1[i].t[0];
      b = ang_1[i].t[1];
      c = ang_1[i].t[2];
      d = ang_1[i].t[3];
      e = ang_1[i].t[4];
      f = ang_1[i].t[5];
/*
      a_2 = ang_2[i*N].t[0];
      b_2 = ang_2[i*N].t[1];
      c_2 = ang_2[i*N].t[2];
      d_2 = ang_2[i*N].t[3];
      e_2 = ang_2[i*N].t[4];
      f_2 = ang_2[i*N].t[5];
*/
//ヤコビ行列3×3のときの可操作度
/*
M =	abs(-a3*(sin(c+b)*d5+a4*cos(c+b)-a3*sin(b)+a1)*(cos(b)*sin(c+b)*d5-sin(b)*cos(c+b)*d5+a4*sin(b)*sin(c+b)+a4*cos(b)*cos(c+b)));
*/

//追加

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



/*
M1 =	abs(a3*(sin(c+b)*d5+a4*cos(c+b)-a3*sin(b)+a1)*(cos(b)*sin(c+b)*d5-sin(b)*cos(c+b)*d5+a4*sin(b)*sin(c+b)+a4*cos(b)*cos(c+b))*sin(e)
*(pow(cos(a),2)*pow(sin(c+b),2)*pow(sin(e),2)
+2.0*cos(a)*sin(a)*sin(c+b)*sin(d)*cos(e)*sin(e)
-2.0*pow(cos(a),2)*cos(c+b)*sin(c+b)*cos(d)*cos(e)*sin(e)
+pow(sin(a),2)*pow(sin(d),2)*pow(cos(e),2)
-2.0*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(cos(e),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(cos(d),2)*pow(cos(e),2)
-pow(cos(a),2)*pow(cos(c+b),2)*pow(sin(d),2)
+pow(sin(a),2)*pow(cos(d),2))
*abs(sin(c+b)*cos(d)*sin(e)-cos(c+b)*cos(e))
/((sin(c+b)*cos(d)*sin(e)-cos(c+b)*cos(e))
*sqrt((pow(cos(a),2)*pow(sin(d),2)+2.0*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)+(pow(sin(c+b),2)+pow(sin(a),2)*pow(cos(c+b),2))*pow(cos(d),2))
*pow(sin(e),2)
+(2.0*cos(a)*sin(a)*sin(c+b)*sin(d)
+(2.0*pow(sin(a),2)-2.0)*cos(c+b)*sin(c+b)*cos(d))*cos(e)*sin(e)
+(pow(sin(a),2)*pow(sin(c+b),2)+pow(cos(c+b),2))
*pow(cos(e),2))
*(
 pow(cos(a),2)*pow(sin(c+b),2)*pow(sin(e),2)
 +pow(cos(a),2)*pow(cos(c+b),2)*pow(sin(d),2)
+pow(sin(a),2)*pow(sin(d),2)*pow(cos(e),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(cos(d),2)*pow(cos(e),2)
+pow(sin(a),2)*pow(cos(d),2)
-2.0*pow(cos(a),2)*cos(c+b)*sin(c+b)*cos(d)*cos(e)*sin(e)
+2.0*cos(a)*sin(a)*sin(c+b)*sin(d)*cos(e)*sin(e)
-2.0*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(cos(e),2)
+2.0*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*(pow(sin(f),2)-pow(cos(f),2))
+4.0*cos(a)*sin(a)*cos(c+b)*pow(sin(d),2)*cos(e)*cos(f)*sin(f)
+4.0*pow(cos(a),2)*cos(c+b)*sin(c+b)*sin(d)*sin(e)*cos(f)*sin(f)
-4.0*pow(cos(a),2)*pow(cos(c+b),2)*cos(d)*sin(d)*cos(e)*cos(f)*sin(f)
)
));
*/

//もともと
/*
M1 = abs(	(a3*(sin(c_1+b_1)*d5+a4*cos(c_1+b_1)-a3*sin(b_1)+a1)*(cos(b_1)*sin(c_1+b_1)*d5-sin(b_1)*cos(c_1+b_1)*d5+a4*sin(b_1)*sin(c_1+b_1)
+a4*cos(b_1)*cos(c_1+b_1))*sin(e_1)*(pow(cos(a_1),2)*pow(sin(c_1+b_1),2)*pow(sin(e_1),2)
+2*cos(a_1)*sin(a_1)*sin(c_1+b_1)*sin(d_1)*cos(e_1)*sin(e_1)
-2*pow(cos(a_1),2)*cos(c_1+b_1)*sin(c_1+b_1)*cos(d_1)*cos(e_1)*sin(e_1)
+pow(sin(a_1),2)*pow(sin(d_1),2)*pow(cos(e_1),2)
-2*cos(a_1)*sin(a_1)*cos(c_1+b_1)*cos(d_1)*sin(d_1)*pow(cos(e_1),2)
+pow(cos(a_1),2)*pow(cos(c_1+b_1),2)*pow(cos(d_1),2)*pow(cos(e_1),2)
-pow(cos(a_1),2)*pow(cos(c_1+b_1),2)*pow(sin(d_1),2)
+pow(sin(a_1),2)*pow(cos(d_1),2))*abs(sin(c_1+b_1)*cos(d_1)*sin(e_1)-cos(c_1+b_1)*cos(e_1)))
/((sin(c_1+b_1)*cos(d_1)*sin(e_1)-cos(c_1+b_1)*cos(e_1))
  *sqrt((pow(cos(a_1),2)*pow(sin(d_1),2)+2*cos(a_1)*sin(a_1)*cos(c_1+b_1)*cos(d_1)*sin(d_1)
  +(pow(sin(c_1+b_1),2)+pow(sin(a_1),2)*pow(cos(c_1+b_1),2))*pow(cos(d_1),2))*pow(sin(e_1),2)
  +(2*cos(a_1)*sin(a_1)*sin(c_1+b_1)*sin(d_1)+(2*pow(sin(a_1),2)-2)*cos(c_1+b_1)*sin(c_1+b_1)*cos(d_1))*cos(e_1)*sin(e_1)
  +(pow(sin(a_1),2)*pow(sin(c_1+b_1),2)+pow(cos(c_1+b_1),2))*pow(cos(e_1),2))
*(pow(cos(a_1),2)*pow(sin(c_1+b_1),2)*pow(sin(e_1),2)*pow(sin(f_1),2)
+2*cos(a_1)*sin(a_1)*sin(c_1+b_1)*sin(d_1)*cos(e_1)*sin(e_1)*pow(sin(f_1),2)
-2*pow(cos(a_1),2)*cos(c_1+b_1)*sin(c_1+b_1)*cos(d_1)*cos(e_1)*sin(e_1)*pow(sin(f_1),2)
+pow(sin(a_1),2)*pow(sin(d_1),2)*pow(cos(e_1),2)
-2*cos(a_1)*sin(a_1)*cos(c_1+b_1)*cos(d_1)*sin(d_1)*pow(cos(e_1),2)*pow(sin(f_1),2)
+pow(cos(a_1),2)*pow(cos(c_1+b_1),2)*pow(cos(d_1),2)*pow(cos(e_1),2)*pow(sin(f_1),2)
+pow(cos(a_1),2)*pow(cos(c_1+b_1),2)*pow(sin(d_1),2)*pow(sin(f_1),2)
+2*cos(a_1)*sin(a_1)*cos(c_1+b_1)*cos(d_1)*sin(d_1)*pow(sin(f_1),2)
+pow(sin(a_1),2)*pow(cos(d_1),2)
+4*pow(cos(a_1),2)*cos(c_1+b_1)*sin(c_1+b_1)*sin(d_1)*sin(e_1)*cos(f_1)*sin(f_1)
+4*cos(a_1)*sin(a_1)*cos(c_1+b_1)*pow(sin(d_1),2)*cos(e_1)*cos(f_1)*sin(f_1)
-4*pow(cos(a_1),2)*pow(cos(c_1+b_1),2)*cos(d_1)*sin(d_1)*cos(e_1)*cos(f_1)*sin(f_1)+pow(cos(a_1),2)*pow(sin(c_1+b_1),2)*pow(sin(e_1),2)*pow(cos(f_1),2)
+2*cos(a_1)*sin(a_1)*sin(c_1+b_1)*sin(d_1)*cos(e_1)*sin(e_1)*pow(cos(f_1),2)
-2*pow(cos(a_1),2)*cos(c_1+b_1)*sin(c_1+b_1)*cos(d_1)*cos(e_1)*sin(e_1)*pow(cos(f_1),2)
-2*cos(a_1)*sin(a_1)*cos(c_1+b_1)*cos(d_1)*sin(d_1)*pow(cos(e_1),2)*pow(cos(f_1),2)
+pow(cos(a_1),2)*pow(cos(c_1+b_1),2)*pow(cos(d_1),2)*pow(cos(e_1),2)*pow(cos(f_1),2)
+pow(cos(a_1),2)*pow(cos(c_1+b_1),2)*pow(sin(d_1),2)*pow(cos(f_1),2)
-2*cos(a_1)*sin(a_1)*cos(c_1+b_1)*cos(d_1)*sin(d_1)*pow(cos(f_1),2)
)
));
*/
  /*  if (M1 < min) {
      g = 1;
      continue;
    }
*/
/*
M2 = abs(	(a3*(sin(c_2+b_2)*d5+a4*cos(c_2+b_2)-a3*sin(b_2)+a1)*(cos(b_2)*sin(c_2+b_2)*d5-sin(b_2)*cos(c_2+b_2)*d5+a4*sin(b_2)*sin(c_2+b_2)
+a4*cos(b_2)*cos(c_2+b_2))*sin(e_2)*(pow(cos(a_2),2)*pow(sin(c_2+b_2),2)*pow(sin(e_2),2)
+2*cos(a_2)*sin(a_2)*sin(c_2+b_2)*sin(d_2)*cos(e_2)*sin(e_2)
-2*pow(cos(a_2),2)*cos(c_2+b_2)*sin(c_2+b_2)*cos(d_2)*cos(e_2)*sin(e_2)
+pow(sin(a_2),2)*pow(sin(d_2),2)*pow(cos(e_2),2)
-2*cos(a_2)*sin(a_2)*cos(c_2+b_2)*cos(d_2)*sin(d_2)*pow(cos(e_2),2)
+pow(cos(a_2),2)*pow(cos(c_2+b_2),2)*pow(cos(d_2),2)*pow(cos(e_2),2)
-pow(cos(a_2),2)*pow(cos(c_2+b_2),2)*pow(sin(d_2),2)
+pow(sin(a_2),2)*pow(cos(d_2),2))*abs(sin(c_2+b_2)*cos(d_2)*sin(e_2)-cos(c_2+b_2)*cos(e_2)))
/((sin(c_2+b_2)*cos(d_2)*sin(e_2)-cos(c_2+b_2)*cos(e_2))
  *sqrt((pow(cos(a_2),2)*pow(sin(d_2),2)+2*cos(a_2)*sin(a_2)*cos(c_2+b_2)*cos(d_2)*sin(d_2)
  +(pow(sin(c_2+b_2),2)+pow(sin(a_2),2)*pow(cos(c_2+b_2),2))*pow(cos(d_2),2))*pow(sin(e_2),2)
  +(2*cos(a_2)*sin(a_2)*sin(c_2+b_2)*sin(d_2)+(2*pow(sin(a_2),2)-2)*cos(c_2+b_2)*sin(c_2+b_2)*cos(d_2))*cos(e_2)*sin(e_2)
  +(pow(sin(a_2),2)*pow(sin(c_2+b_2),2)+pow(cos(c_2+b_2),2))*pow(cos(e_2),2))
*(pow(cos(a_2),2)*pow(sin(c_2+b_2),2)*pow(sin(e_2),2)*pow(sin(f_2),2)
+2*cos(a_2)*sin(a_2)*sin(c_2+b_2)*sin(d_2)*cos(e_2)*sin(e_2)*pow(sin(f_2),2)
-2*pow(cos(a_2),2)*cos(c_2+b_2)*sin(c_2+b_2)*cos(d_2)*cos(e_2)*sin(e_2)*pow(sin(f_2),2)
+pow(sin(a_2),2)*pow(sin(d_2),2)*pow(cos(e_2),2)
-2*cos(a_2)*sin(a_2)*cos(c_2+b_2)*cos(d_2)*sin(d_2)*pow(cos(e_2),2)*pow(sin(f_2),2)
+pow(cos(a_2),2)*pow(cos(c_2+b_2),2)*pow(cos(d_2),2)*pow(cos(e_2),2)*pow(sin(f_2),2)
+pow(cos(a_2),2)*pow(cos(c_2+b_2),2)*pow(sin(d_2),2)*pow(sin(f_2),2)
+2*cos(a_2)*sin(a_2)*cos(c_2+b_2)*cos(d_2)*sin(d_2)*pow(sin(f_2),2)
+pow(sin(a_2),2)*pow(cos(d_2),2)
+4*pow(cos(a_2),2)*cos(c_2+b_2)*sin(c_2+b_2)*sin(d_2)*sin(e_2)*cos(f_2)*sin(f_2)
+4*cos(a_2)*sin(a_2)*cos(c_2+b_2)*pow(sin(d_2),2)*cos(e_2)*cos(f_2)*sin(f_2)
-4*pow(cos(a_2),2)*pow(cos(c_2+b_2),2)*cos(d_2)*sin(d_2)*cos(e_2)*cos(f_2)*sin(f_2)+pow(cos(a_2),2)*pow(sin(c_2+b_2),2)*pow(sin(e_2),2)*pow(cos(f_2),2)
+2*cos(a_2)*sin(a_2)*sin(c_2+b_2)*sin(d_2)*cos(e_2)*sin(e_2)*pow(cos(f_2),2)
-2*pow(cos(a_2),2)*cos(c_2+b_2)*sin(c_2+b_2)*cos(d_2)*cos(e_2)*sin(e_2)*pow(cos(f_2),2)
-2*cos(a_2)*sin(a_2)*cos(c_2+b_2)*cos(d_2)*sin(d_2)*pow(cos(e_2),2)*pow(cos(f_2),2)
+pow(cos(a_2),2)*pow(cos(c_2+b_2),2)*pow(cos(d_2),2)*pow(cos(e_2),2)*pow(cos(f_2),2)
+pow(cos(a_2),2)*pow(cos(c_2+b_2),2)*pow(sin(d_2),2)*pow(cos(f_2),2)
-2*cos(a_2)*sin(a_2)*cos(c_2+b_2)*cos(d_2)*sin(d_2)*pow(cos(f_2),2)
)
));

    if (M2 < min) {
      g = 1;
      continue;
    }
*/
//M2 = sin(c+b)
/*a3*(sin(c+b)*d5+a4*cos(c+b)-a3*sin(b)+a1)
*(cos(b)*sin(c+b)*d5-sin(b)*cos(c+b)*d5+a4*sin(b)*sin(c+b)
+a4*cos(b)*cos(c+b))*sin(e)*(pow(cos(a),2)*pow(sin(c+b),2)*pow(sin(e),2)
+2*cos(a)*sin(a)*sin(c+b)*sin(d)*cos(e)*sin(e)
-2*pow(cos(a),2)*cos(c+b)*sin(c+b)*cos(d)*cos(e)*sin(e)
+pow(sin(a),2)*pow(sin(d),2)*pow(cos(e),2)
-2*cos(a)*sin(a)*cos(c+b)*cos(d)*sin(d)*pow(cos(e),2)
+pow(cos(a),2)*pow(cos(c+b),2)*pow(cos(d),2)*pow(cos(e),2)
-pow(cos(a),2)*pow(cos(c+b),2)*pow(sin(d),2)
+pow(sin(a),2)*pow(cos(d),2))*abs(sin(c+b)*cos(d)*sin(e)-cos(c+b)*cos(e))*/
;

//std::cout<<"c="<<b<<std::endl;

  if(std::isnan(M1)){
    //std::cout<<M1<<std::endl;
    M1 = -1.0;

    if (std::isnan(M2)) {
      g++;
    }
  //  return M1;
  }

  /* if (i == 0) {
      M_min = M1;
    }
  */
    M.push_back(M1);

/*
    if (M1 < M_min) {
      M_min = M1;
    }
*/
/*    if (M2 < M_min) {
      M_min = M2;
    }
*/
    }
//  }
std::sort(M.begin(), M.end());
//std::cout<<g<<std::endl;

return std::accumulate(M.begin(), M.end(),0.0);
//  return M[0]+M[1]+M[2]+M[3]+M[4]+M[5]+M[6];
}



int main(int argc, char** argv)
{
    cc = 0;

 ////////////////////////////////////////////////////////////////////////////////////////////

//csv2vectorの実行
    std::string filename = "capture543.csv";//"line_test.csv";
    std::vector<std::vector<std::string> > data = csv2vector(filename, 2);//第2引数に指定した数だけその行分読み飛ばす
    ik right[data.size()];// data.sizeはvector関数専用
    ik left[data.size()];//rightは右手,leftは左手

        SIZE = data.size() - S - E;
        std::cout << SIZE << std::endl;

////////////////////////////////////////////////////////////////////////////////////////////////
/*マニピュレータの配置決定*/
  printf("start\n" );

//check

int checker=0;
/*
for (int i = 0; i < SIZE/N; i++) {
inverse_kin(stod(data[i+S][8]) - stod(data[0][2]) -57, -stod(data[i+S][9]) + stod(data[0][3]) - 350/*+ 245*//*, -stod(data[i+S][10]) + 1400 + 750,0,0,1,0,1,0,-1,0,0);
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
for(int i=0; i < SIZE; i++){
//1
  //x
  if (stod(data[i][8])>max_x_1) {
    max_x_1 = stod(data[i][8]);
  }
  if (stod(data[i][8])<min_x_1) {
    min_x_1 = stod(data[i][8]);
  }
  //y
  if (stod(data[i][9])>max_y_1) {
    max_y_1 = stod(data[i][9]);
  }
  if (stod(data[i][9])<min_y_1) {
    min_y_1 = stod(data[i][9]);
  }
  //z
  if (stod(data[i][10])>max_z_1) {
    max_z_1 = stod(data[i][10]);
  }
  if (stod(data[i][10])<min_z_1) {
    min_z_1 = stod(data[i][10]);
  }
//2
  //x
  if (stod(data[i][17])>max_x_2) {
    max_x_2 = stod(data[i][17]);
  }
  if (stod(data[i][17])<min_x_2) {
    min_x_2 = stod(data[i][17]);
  }
  //y
  if (stod(data[i][18])>max_y_2) {
    max_y_2 = stod(data[i][18]);
  }
  if (stod(data[i][18])<min_y_2) {
    min_y_2 = stod(data[i][18]);
  }
  //z
  if (stod(data[i][19])>max_z_2) {
    max_z_2 = stod(data[i][19]);
  }
  if (stod(data[i][19])<min_z_2) {
    min_z_2 = stod(data[i][19]);
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

double a = 10.0;

double check_x[2], check_y[2], check_z[2];
int check_min[2]={1000,1000};
int check;
int ff;

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

printf("111\n" );

//どうにかして範囲を減らせないか
for (x_2_1 = x_min_1; x_2_1 <= x_max_1; x_2_1 += a) {
  for (y_2_1 = y_min_1; y_2_1 <= y_max_1; y_2_1 += a) {
    for (z_2_1 = z_min_1; z_2_1 <= z_max_1; z_2_1 += a) {

      check = 0;
//マニピュレータの動作範囲と照らし合わせる
//逆行列に入れて角度から動作範囲と照らし合わせる
//範囲外のものが１つでもないか確認する
//ロボットの自己接触も考慮
//ただし、少し大きめに設定している
      for (int i = 0; i < SIZE/N; i++) {

      /*  if (check >= 1) {
          continue;
        }*/

          if (sqrt(pow(stod(data[i*N][8]) - x_2_1,2) + pow(stod(data[i*N][9]) - y_2_1,2) + pow(stod(data[i*N][10]) - z_2_1,2)) <= 875.5){
            if(stod(data[i*N][10])-z_2_1 <= 224.5/*0*//*||stod(data[i*N][10])-z_2_1 > 400.0*/){
              if( sqrt(pow(stod(data[i*N][8]) - x_2_1,2) + pow(stod(data[i*N][9]) - y_2_1,2) + pow(stod(data[i*N][10]) - z_2_1,2)) > 224.5/*309.3*/) {
                CC = 0;
                inv = inverse_kin(stod(data[i*N+S][8])-x_2_1+30.0, -stod(data[i*N+S][9])+y_2_1, -stod(data[i*N+S][10])+z_2_1-395.0,0,0,1,0,1,0,-1,0,0);
                if (CC == 1) {
                  check++;
                }
              }
              else {
                check++;
              }
            }
            else if(stod(data[i*N][10])-z_2_1 > 400.0){
              if( sqrt(pow(stod(data[i*N][8]) - x_2_1,2) + pow(stod(data[i*N][9]) - y_2_1,2) + pow(stod(data[i*N][10]) - z_2_1,2)) > 100.0/*224.5,309.3*/) {
                CC = 0;
                inv = inverse_kin(stod(data[i*N+S][8])-x_2_1+30.0, -stod(data[i*N+S][9])+y_2_1, -stod(data[i*N+S][10])+z_2_1-395.0,0,0,1,0,1,0,-1,0,0);
                if (CC == 1) {
                  check++;
                }
              }
              else {
                check++;
              }
            }
            else {
              if( sqrt(pow(stod(data[i*N][8]) - x_2_1,2) + pow(stod(data[i*N][9]) - y_2_1,2)) > 224.5/*309.3*/) {
                CC = 0;

                inv = inverse_kin(stod(data[i*N+S][8])-x_2_1+30.0, -stod(data[i*N+S][9])+y_2_1, -stod(data[i*N+S][10])+z_2_1-395.0,0,0,1,0,1,0,-1,0,0);
                if (CC == 1) {
                  check++;
                }
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



      if (check == 0) {
          X[0].push_back(x_2_1);
          Y[0].push_back(y_2_1);
          Z[0].push_back(z_2_1);
        bb_1++;
        max_bb_1 = bb_1;
        std::cout<<inv.t[0]<<","<<inv.t[1]<<","<<inv.t[2]<<","<<inv.t[3]<<","<<inv.t[4]<<","<<inv.t[5]<<std::endl;
      }

      if (check < check_min[0]) {
        check_x[0] = x_2_1;
        check_y[0] = y_2_1;
        check_z[0] = z_2_1;
        check_min[0] = check;
      }
    //  std::cout<<check<<std::endl;

    }
  }
}


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
          if (sqrt(pow(stod(data[i][8]) - x_2_2,2) + pow(stod(data[i][9]) - y_2_2,2) + pow(stod(data[i][10]) - z_2_2,2)) <= 875.0){
            if(stod(data[i][10])-z_2_2 <= 0||stod(data[i][10])-z_2_2 > 400.0){
              if( sqrt(pow(stod(data[i][8]) - x_2_2,2) + pow(stod(data[i][9]) - y_2_2,2) + pow(stod(data[i][10]) - z_2_2,2)) > 309.3) {
                CC = 0;
                inverse_kin(stod(data[i+S][8])-x_2_2+30.0, -stod(data[i+S][9])+y_2_2, -stod(data[i+S][10])+z_2_2-395.0,0,0,1,0,1,0,-1,0,0);
                if (CC == 1) {
                  check++;
                }
              }
              else {
                check++;
              }
            }
            else {
              if( sqrt(pow(stod(data[i][8]) - x_2_2,2) + pow(stod(data[i][9]) - y_2_2,2)) > 309.3) {
                CC = 0;

                inverse_kin(stod(data[i+S][8])-x_2_2+30.0, -stod(data[i+S][9])+y_2_2, -stod(data[i+S][10])+z_2_2-395.0,0,0,1,0,1,0,-1,0,0);
                if (CC == 1) {
                  check++;
                }
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

printf("aa\n" );

std::cout<<check_x[0]<<","<<check_y[0]<<","<<check_z[0]<<","<<check_min[0]<<",,"<<check_x[1]<<","<<check_y[1]<<","<<check_z[1]<<","<<check_min[1]<<std::endl;

int bb_max_1[8];
int bb_max_2[8];
int bb_max11,bb_max12,bb_max13,bb_max14,bb_max15;
int bb_max21,bb_max22,bb_max23,bb_max24,bb_max25;
int bb_max18,bb_max28;

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
  double min_mani_max[2][8] = {-1.5e100,-1.4e100,-1.3e100,-1.2e100,-1.1e100,-1.0e100,-0.9e100,-0.8e100, -1.5e100,-1.4e100,-1.3e100,-1.2e100,-1.1e100,-1.0e100,-0.9e100,-0.8e100};
  std::cout<<max_bb_1<<","<<max_bb_2<<std::endl;
  std::vector<std::vector<double> > min_mani(2);
  //double min_mani[max_bb_1][max_bb_2];//可操作度の最小
  printf("b\n" );

//可操作度の計算
//最小を(5個位)記憶してそれよりも小さい点があったらそこで中断するようにする
  for (bb_1 = 0; bb_1 < max_bb_1; bb_1++) {
      for(int i = 0; i < SIZE; i+=N){
        mani_1[i]
          =inverse_kin(stod(data[i+S][8])-(X[0][bb_1]-30.0),-stod(data[i+S][9])+(Y[0][bb_1]),-stod(data[i+S][10])+(Z[0][bb_1]-395.0),0.0,0.0,1.0,0.0,1.0,0.0,-1.0,0.0,0.0);
      }
      min_mani[0].push_back(manipulatability(mani_1/*, min_mani_max[0][7]*/));
      //std::cout<<manipulatability(mani_1)<<std::endl;sort
      if (min_mani[0][bb_1]>min_mani_max[0][7]) {
        min_mani_max[0][7] = min_mani[0][bb_1];
        bb_max_1[7] = bb_1;
        for (int i = 0; i < 8; i++) {
          for (int j = i+1; j < 8; j++) {

            if (min_mani_max[0][i] < min_mani_max[0][j]) {
              tmp = min_mani_max[0][i];
              min_mani_max[0][i] = min_mani_max[0][j];
              min_mani_max[0][j] = tmp;
              tmp_bb = bb_max_1[i];
              bb_max_1[i] = bb_max_1[j];
              bb_max_1[j] = tmp_bb;
            }
          }
        }
      }
  }

    for (bb_2 = 0; bb_2 < max_bb_2; bb_2++) {
      for(int i = 0; i < SIZE; i+=N){
        mani_2[i/N] = inverse_kin(stod(data[i+S][17])-(X[1][bb_2]-30.0), -stod(data[i+S][18])+(Y[1][bb_2]), -stod(data[i+S][19])+(Z[1][bb_2]-395.0),0.0,0.0,1.0,0.0,1.0,0.0,-1.0,0.0,0.0);
      }
      min_mani[1].push_back(manipulatability(mani_2/*, min_mani_max[1][7]*/));

     if (min_mani[1][bb_2]>min_mani_max[1][7]) {
        min_mani_max[1][7] = min_mani[1][bb_2];
        bb_max_2[7] = bb_2;
        for (int i = 0; i < 8; i++) {
          for (int j = i+1; j < 8; j++) {

            if (min_mani_max[1][i] < min_mani_max[1][j]) {
              tmp = min_mani_max[1][i];
              min_mani_max[1][i] = min_mani_max[1][j];
              min_mani_max[1][j] = tmp;
              tmp_bb = bb_max_2[i];
              bb_max_2[i] = bb_max_2[j];
              bb_max_2[j] = tmp_bb;
            }
          }
        }
      }
    }

printf("aaa\n" );

//std::cout<<min_mani[0][0]<<std::endl;

//std::sort(min_mani[0].rbegin(), min_mani[0].rend());

//可操作度を考慮した上での配置場所の出力

  /*for(int i = 0; i < SIZE; i+=N){
    mani[i] = inverse_kin(stod(data[i+S][8])-(X[bb_max]-30.0), stod(data[i+S][9])-(Y[bb_max]), +stod(data[i+S][10])-(Z[bb_max]-395.0),0,0,1,0,1,0,-1,0,0);
  }*/
  bb_max11 = bb_max_1[0];
  bb_max21 = bb_max_2[0];
  bb_max12 = bb_max_1[1];
  bb_max22 = bb_max_2[1];
  bb_max13 = bb_max_1[2];
  bb_max23 = bb_max_2[2];
  bb_max14 = bb_max_1[3];
  bb_max24 = bb_max_2[3];
  bb_max15 = bb_max_1[4];
  bb_max25 = bb_max_2[4];
  bb_max18 = bb_max_1[7];
  bb_max28 = bb_max_2[7];

/*    std::cout<<X[0][bb_max11]-30.0<<","<<Y[0][bb_max11]<<","<<Z[0][bb_max11]-395.0<<","<<
    X[1][bb_max21]-30.0<<","<<Y[1][bb_max21]<<","<<Z[1][bb_max21]-395.0<<":"<<min_mani[0][bb_max11]<<":"<<min_mani[1][bb_max21]<<std::endl;

    std::cout<<X[0][bb_max12]-30.0<<","<<Y[0][bb_max12]<<","<<Z[0][bb_max12]-395.0<<","<<
    X[1][bb_max22]-30.0<<","<<Y[1][bb_max22]<<","<<Z[1][bb_max22]-395.0<<":"<<min_mani[0][bb_max12]<<":"<<min_mani[1][bb_max22]<<std::endl;

    std::cout<<X[0][bb_max13]-30.0<<","<<Y[0][bb_max13]<<","<<Z[0][bb_max13]-395.0<<","<<
    X[1][bb_max23]-30.0<<","<<Y[1][bb_max23]<<","<<Z[1][bb_max23]-395.0<<":"<<min_mani[0][bb_max13]<<":"<<min_mani[1][bb_max23]<<std::endl;

    std::cout<<X[0][bb_max14]-30.0<<","<<Y[0][bb_max14]<<","<<Z[0][bb_max14]-395.0<<","<<
    X[1][bb_max24]-30.0<<","<<Y[1][bb_max24]<<","<<Z[1][bb_max24]-395.0<<":"<<min_mani[0][bb_max14]<<":"<<min_mani[1][bb_max24]<<std::endl;

    std::cout<<X[0][bb_max15]-30.0<<","<<Y[0][bb_max15]<<","<<Z[0][bb_max15]-395.0<<","<<
    X[1][bb_max25]-30.0<<","<<Y[1][bb_max25]<<","<<Z[1][bb_max25]-395.0<<":"<<min_mani[0][bb_max15]<<":"<<min_mani[1][bb_max25]<<std::endl;

    std::cout<<X[0][bb_max18]-30.0<<","<<Y[0][bb_max18]<<","<<Z[0][bb_max18]-395.0<<","<<
    X[1][bb_max28]-30.0<<","<<Y[1][bb_max28]<<","<<Z[1][bb_max28]-395.0<<":"<<min_mani[0][bb_max18]<<":"<<min_mani[1][bb_max28]<<std::endl;*/

    std::cout<<min_mani[0].size()<<std::endl;

   for (int i = 0; i < 8/*min_mani[0].size()*/; i++) {
      std::cout<<min_mani[0][i]<<std::endl;
}
}
/////////////////////////////////////////////////////////////////////////////////////////////////
//動作


    return 0;
}
