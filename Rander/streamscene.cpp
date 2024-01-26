#include "streamscene.h"


StreamScene::StreamScene(QObject *parent)
    : QGraphicsScene{parent}
{
    cameraThread = new CameraThread();
    connect(cameraThread, &CameraThread::imageReady, this, &StreamScene::handleImage);
    connect(cameraThread, &CameraThread::logReady, this, &StreamScene::handleLog);
    connect(cameraThread, &CameraThread::mapReady, this, &StreamScene::handleMap);
    connect(cameraThread, &CameraThread::hpReady, this, &StreamScene::handleHp);
    connect(cameraThread, &CameraThread::roiReady, this, &StreamScene::handleRoi);
    cameraThread->start();

}
void StreamScene::handleImage(const QImage &image)
{
    clear();
    addPixmap(QPixmap::fromImage(image));
}
void StreamScene::handleLog(QString data)
{
    emit logChange(data);
}
void StreamScene::handleMap(std::vector<std::array<int,4>> target)
{

    emit mapChange(target);
}

void StreamScene::handleHp(std::vector<std::array<int,3>> hp)
{

    emit hpChange(hp);
}

void StreamScene::handleRoi(QRectF roi)
{

    emit roiChanged(roi);
}
StreamScene::~StreamScene()
{
    delete cameraThread;
}
