#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

typedef struct { double t1, t2, t3, t4, t5, t6; } ik;

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
        temp_data = split(reading_line_buffer, '\t');
        data.push_back(temp_data);
    }
    return data;

}

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


//densoのurdfに書いてあるリミット
    if(t1 < -2.96705972839036 || t1 > 2.96705972839036)
    {
        printf("t1 is out of range!!!\n");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        exit(1);
    }
    else if(t2 < -1.74532925199433 || t2 > 2.35619449019234)
    {
        printf("t2 is out of range!!!\n");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        printf("%f\n",sqrt(pow((-pz + d1 + d2),2) + pow((-a1 + c1*px + s1*py),2) - pow(L,2)));
      //  exit(1);
    }
    else if(t3 < -2.37364778271229 || t3 > 2.67035375555132)
    {
        printf("t3 is out of range!!!\n");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        exit(1);
    }
    else if(t4 < -4.71238898038469 || t4 > 4.71238898038469)
    {
        printf("t4 is out of range!!!\n");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        exit(1);
    }
    else if(t5 < -2.0943951023932 || t5 > 2.0943951023932)
    {
        printf("t5 is out of range!!!\n");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        exit(1);
    }
    else if(t6 < -6.28318530717959 || t6 > 6.28318530717959)
    {
        printf("t6 is out of range!!!\n");
        printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
        exit(1);
    }

    printf("%f,%f,%f,%f,%f,%f\n",t1,t2,t3,t4,t5,t6);
    return { t1, t2, t3, t4, t5, t6 };
}

int main(int argc, char *argv[]){
    //csv2vectorの実行
    std::string filename = "capture_s.txt";
    std::vector<std::vector<std::string> > data = csv2vector(filename, 2);//第2引数に指定した数だけその行分読み飛ばす

    for(int i = 0; i < data.size(); i+=3){
      inverse_kin(stod(data[i][8]) - stod(data[i][2]) +30, -stod(data[i][9]) + stod(data[i][3]), -stod(data[i][10]) + stod(data[i][4]) - 395,1,0,0,0,1,0,0,0,1);
    }
    //二次元配列dataの標準出力への出力
  /*  for(int i = 0; i < data.size(); i+=3){
        for(int j = 0; j < data[i].size(); j++){
            std::cout << data[i][j] << ",";
        }
        std::cout << std::endl;//改行を出力
    }*/
    for(int i = 0; i < data.size(); i+=3){
      std::cout << stod(data[i][8]) - stod(data[i][2]) +30 << ","<<  -stod(data[i][9]) + stod(data[i][3]) << ","<< -stod(data[i][10]) + stod(data[i][4]) - 395<< std::endl;
    }
}
