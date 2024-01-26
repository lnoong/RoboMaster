#ifndef INFORMATIONUPDATER_H
#define INFORMATIONUPDATER_H

#include <QObject>
#include <QTimer>
#include <QImage>
class InformationUpdater : public QObject
{
    Q_OBJECT
public:
    explicit InformationUpdater(QObject *parent = nullptr);

private:
    QTimer *timer;
    QString _data = "testimformation";
    std::vector<std::array<int,4>> _target;
    std::vector<std::array<int,3>> _hp;
    QRectF _roi;

public slots:
    void infoUpdate();
    void logChange(QString data);
    void mapChange(std::vector<std::array<int,4>> target);
    void hpChange(std::vector<std::array<int,3>> hp);
    void roiChanged(QRectF roi);
signals:
    void mapUpdate(std::vector<std::array<int,4>> target);
    void logUpdate(QString data);
    void hpUpdate(std::vector<std::array<int,3>> hp);
    void roiUpdate(QRectF roi);
};

#endif // INFORMATIONUPDATER_H
