#ifndef STREAMSCENE_H
#define STREAMSCENE_H

#include "camerathread.h"
#include <QGraphicsScene>
class StreamScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit StreamScene(QObject *parent = nullptr);
    ~StreamScene();

signals:
    void cameraStop();
    void logChange(QString data);
    void mapChange(std::vector<std::array<int,4>> target);
    void hpChange(std::vector<std::array<int,3>> hp);
    void roiChanged(QRectF roi);
private:
    CameraThread *cameraThread;


public slots:
    void handleImage(const QImage &image);
    void handleLog(QString data);
    void handleMap(std::vector<std::array<int,4>> target);
    void handleHp(std::vector<std::array<int,3>> hp);
    void handleRoi(QRectF roi);
};

#endif // STREAMSCENE_H
