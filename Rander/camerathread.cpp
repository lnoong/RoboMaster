#include "camerathread.h"

CameraThread::CameraThread()
{
    camera.open(0);
    roi = *new QRectF(50,50,290,290);
}

void CameraThread::run()
{
    int x = 0;
    while (running)
    {
        target.clear();
        hp.clear();
        camera >> frame;
        if(frame.empty())
            continue;
        QImage image(frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888);
        image = image.rgbSwapped();
        ////////test//////////
        emit imageReady(image);

        emit logReady("null");

        std::array<int,4> r= {x++,120,0,3};
        std::array<int,4> b= {x++,x++,1,5};
        std::array<int,4> c= {200,100,0,1};
        target.push_back(c);
        target.push_back(r);
        target.push_back(b);
        emit mapReady(target);

        std::array<int,3> hpR1= {60,0,1};
        std::array<int,3> hpR2= {x,1,1};
        hp.push_back(hpR2);
        hp.push_back(hpR1);
        emit hpReady(hp);

        roi.setRect(x++,100,100,100);
        emit roiReady(roi);
        //////////test//////////
        QThread::usleep(10000);
    }
}


CameraThread::~CameraThread()
{
    running = false;
    camera.release();
    quit();
    wait();
}
