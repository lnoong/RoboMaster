#ifndef MAPSCENE_H
#define MAPSCENE_H

#include <QGraphicsScene>
#include <QGraphicsItem>
#include "numellipseitem.h"
class MapScene : public QGraphicsScene
{
public:
    explicit MapScene(QObject *parent = nullptr);


private:
    QPixmap backgroundMap;
    NumEllipseItem *R[8];
    NumEllipseItem *B[8];

public slots:
    void mapUpdate(std::vector<std::array<int,4>> target);

};


#endif // MAPSCENE_H
