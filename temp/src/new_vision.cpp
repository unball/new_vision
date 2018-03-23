#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[]){
    if (argc < 2){
        std::cout << "Missing Arguments: aborting" << std::endl;
        exit(1);
    }

    cv::VideoCapture raw_input;
    raw_input.open(argv[1]);

    if (raw_input.isOpened() == 0){
        std::cout << "deu merda" << std::endl; //TODO: VideoCapture::open() nao esta funcionando
    }

    while (raw_input.isOpened()){
        cv::Mat frame;

        raw_input >> frame;
        cv::imshow("raw_input", frame);
    }

    return (0);
}
