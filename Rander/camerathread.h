#ifndef CAMERATHREAD_H
#define CAMERATHREAD_H

#include "qdebug.h"
#include <QThread>
#include <QObject>
#include <QImage>
#include <opencv2/opencv.hpp>

class CameraThread : public QThread {
    Q_OBJECT
signals:
    void imageReady(const QImage &image);
    void logReady(QString data);
    void mapReady(std::vector<std::array<int,4>> target);
    void hpReady(std::vector<std::array<int,3>> hp);
    void roiReady(QRectF roi);

public:
    CameraThread();
    ~CameraThread();

private:
    bool running = true;
    cv::VideoCapture camera;
    cv::Mat frame;
    //x,y,type,id
    std::vector<std::array<int,4>> target;
    //hp,type,id
    std::vector<std::array<int,3>> hp;
    QRectF roi;


protected:
    void run();
};


#endif // CAMERATHREAD_H
