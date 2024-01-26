#include "informationupdater.h"
#include "qdebug.h"

InformationUpdater::InformationUpdater(QObject *parent)
    : QObject{parent}
{

    timer = new QTimer();
    connect(timer,&QTimer::timeout,this,&InformationUpdater::infoUpdate);
    timer->start(10);
}

void InformationUpdater::infoUpdate()
{

    emit mapUpdate(_target);
    emit logUpdate(_data);
    emit hpUpdate(_hp);
    emit roiUpdate(_roi);
}
void InformationUpdater::logChange(QString data)
{
    _data = data;
}
void InformationUpdater::mapChange(std::vector<std::array<int,4>> target)
{
    _target = target;
}
void InformationUpdater::hpChange(std::vector<std::array<int, 3> > hp)
{
    _hp = hp;
}
void InformationUpdater::roiChanged(QRectF roi)
{
    _roi = roi;
}
