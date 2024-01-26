#include "radar.h"

#include <QApplication>
#include<opencv2/opencv.hpp>
using namespace cv;
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Radar w;
    w.resize(1280,720);
    w.show();
    return a.exec();
}
