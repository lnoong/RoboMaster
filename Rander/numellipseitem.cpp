#include "numellipseitem.h"
#include <QPainter>

NumEllipseItem::NumEllipseItem(int x_, int y_, int type_, int id_)
    : x(x_), y(y_), type(type_), id(id_) {}

QRectF NumEllipseItem::boundingRect() const {
    return QRectF(x - diameter / 2, y - diameter / 2, diameter, diameter);
}

void NumEllipseItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    Q_UNUSED(option);
    Q_UNUSED(widget);

    if(type==0)
    {
        painter->setPen(QPen(QColor(0xFA0000), 5));
        painter->setBrush(QColor(0xFA0000));
    }
    else if(type == 1)
    {
        painter->setPen(QPen(QColor(0x0000FA), 5));
        painter->setBrush(QColor(0x0000FA));
    }
    else
    {
        painter->setPen(QPen(QColor(0x00FA00), 5));
        painter->setBrush(QColor(0x00FA00));
    }

    painter->drawEllipse(x - diameter / 2, y - diameter / 2, diameter, diameter);

    painter->setFont(QFont("Arial", 14));
    painter->setPen(Qt::white);
    QRectF textRect = painter->boundingRect(QRectF(x - diameter / 2, y - diameter / 2, diameter, diameter), Qt::AlignCenter, QString::number(id));
    painter->drawText(QPointF(x - textRect.width() / 2, y + textRect.height() / 4), QString::number(id));
}
void NumEllipseItem::setPos(int x,int y)
{
    this->x = x;
    this->y = y;
}
