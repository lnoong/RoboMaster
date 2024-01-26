#include "mapscene.h"

#include "qdebug.h"
#include "qpainter.h"

MapScene::MapScene(QObject *parent)
    : QGraphicsScene{parent}
{
    backgroundMap.load("/home/zl/QtProjects/Radar/Radar/images/map.jpg");
    QBrush backgroundBrush(backgroundMap);
    setSceneRect(0,0,backgroundMap.width(),backgroundMap.height());
    setBackgroundBrush(backgroundBrush);

    for(int i=0;i<8;i++)
    {
        R[i] = new NumEllipseItem(0,0,0,i+1);
        B[i] = new NumEllipseItem(0,0,1,i+1);
        addItem(R[i]);
        addItem(B[i]);
    }
}



void MapScene::mapUpdate(std::vector<std::array<int,4>> target)
{

    for(int i = 0;i<target.size();i++)
        if(target[i][2] == 0)
            R[target[i][3]-1]->setPos(target[i][0],target[i][1]);
        else if(target[i][2] == 1)
            B[target[i][3]-1]->setPos(target[i][0],target[i][1]);
        else
            continue;
    update();
}
