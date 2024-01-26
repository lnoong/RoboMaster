#ifndef NUMELLIPSEITEM_H
#define NUMELLIPSEITEM_H

#include <QGraphicsItem>

class NumEllipseItem : public QGraphicsItem
{
public:
    NumEllipseItem(int x_, int y_, int type_, int id_);

    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = nullptr) override;
    void setPos(int x,int y);
private:
    int x = 0;
    int y = 0;
    int diameter = 20;
    int type = 0;
    int id = 0;
};

#endif // NUMELLIPSEITEM_H
