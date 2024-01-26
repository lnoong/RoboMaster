#ifndef RADAR_H
#define RADAR_H

#include <QMainWindow>
#include "informationupdater.h"
#include "mapscene.h"
#include "streamscene.h"
#include <QTime>
QT_BEGIN_NAMESPACE
namespace Ui {
class Radar;
}
QT_END_NAMESPACE

class Radar : public QMainWindow
{
    Q_OBJECT

public:
    Radar(QWidget *parent = nullptr);
    ~Radar();

private:
    Ui::Radar *ui;
    MapScene *mapScene;
    StreamScene *streamScene;
    InformationUpdater *informationUpdater;
public slots:
    void logUpdate(QString data);
    void hpUpdate(std::vector<std::array<int,3>> hp);
    void roiUpdate(QRectF roi);
};
#endif // RADAR_H
