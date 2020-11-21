#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;

int main() {
    cv::Mat img, dst, gray, mono;

    img = cv::imread("/home/yugo/arrc/programming/2020rc/penguin.jpg");
    cv::resize(img, dst, cv::Size(), 0.5, 0.5);

    cv::cvtColor(dst, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, mono, 190, 255, cv::THRESH_BINARY);

    //ノイズ消し
    erode (mono, mono, cv::Mat(), cv::Point(-1, -1), 2);
    dilate(mono, mono, cv::Mat(), cv::Point(-1, -1), 2);

    //labeling
    cv::Mat labeling, stats, centroids, img_out;
    int nlab = cv::connectedComponentsWithStats(mono, labeling, stats, centroids);
    for (int i = 0; i < nlab; i++) {
        //ラベルiを取り出す
        cv::compare(labeling, i, img_out, cv::CMP_EQ);
        //出力画像の出力
        cv::imshow("Labelling", img_out);
        cv::waitKey(0);//入力待ち
    }

    /*while(1) {
        cv::imshow("aa", mono);
        if(cv::waitKey(1) == 'q') {
            cout << "finish!!" << endl;
            break;
        }
    }*/
}

