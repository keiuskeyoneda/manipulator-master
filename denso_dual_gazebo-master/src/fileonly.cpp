#include <vector>
#include <string>
#include <fstream>
#include <iostream>

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
csv2vector(std::string filename, int ignore_line_num = 0){
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

int main(int argc, char *argv[]){
    //csv2vectorの実行
    std::vector<std::vector<std::string> > data = csv2vector(argv[1], 2);//第2引数に指定した数だけその行分読み飛ばす

    //二次元配列dataの標準出力への出力
    for(int i = 0; i < data.size(); i++){
        for(int j = 0; j < data[i].size(); j++){
            std::cout << data[i][j] << ",";
        }
        std::cout << std::endl;
    }
}
