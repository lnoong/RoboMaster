#include "radar.h"
#include "./ui_radar.h"


Radar::Radar(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::Radar)
{
    ui->setupUi(this);
    this->setMenuBar(nullptr);
    ui->statusBar->setStyleSheet("background-color: #505050");

    mapScene = new MapScene();
    ui->graphicsView_map->setFixedSize(mapScene->width(),mapScene->height());
    ui->graphicsView_map->setScene(mapScene);
    ui->graphicsView_map->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView_map->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    QGraphicsScene *logoScene = new QGraphicsScene();
    QPixmap logo("/home/zl/QtProjects/Radar/Radar/images/logo.png");
    QBrush logoBrush(logo);
    logoScene->setSceneRect(0,0,logo.width(),logo.height());
    logoScene->setBackgroundBrush(logoBrush);
    ui->graphicsView_logo->setFixedSize(logoScene->width(),logoScene->height());
    ui->graphicsView_logo->setScene(logoScene);
    ui->graphicsView_logo->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView_logo->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    streamScene = new StreamScene();
    ui->graphicsView_full->setScene(streamScene);
    // ui->graphicsView_full->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    // ui->graphicsView_full->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);



    ui->graphicsView_roi->setScene(streamScene);
    ui->graphicsView_roi->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    ui->graphicsView_roi->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    informationUpdater = new InformationUpdater();
    connect(informationUpdater,&InformationUpdater::mapUpdate,mapScene,&MapScene::mapUpdate);
    connect(informationUpdater,&InformationUpdater::logUpdate,this,&Radar::logUpdate);
    connect(informationUpdater,&InformationUpdater::hpUpdate,this,&Radar::hpUpdate);
    connect(informationUpdater,&InformationUpdater::roiUpdate,this,&Radar::roiUpdate);

    connect(streamScene,&StreamScene::logChange,informationUpdater,&InformationUpdater::logChange);
    connect(streamScene,&StreamScene::mapChange,informationUpdater,&InformationUpdater::mapChange);
    connect(streamScene,&StreamScene::hpChange,informationUpdater,&InformationUpdater::hpChange);
    connect(streamScene,&StreamScene::roiChanged,informationUpdater,&InformationUpdater::roiChanged);

}
void Radar::logUpdate(QString data)
{
    QString sendData = data;
    sendData = QString::number(QTime::currentTime().hour())+":"
               +QString::number(QTime::currentTime().minute())+":"
               +QString::number(QTime::currentTime().second())+": "
               +sendData;
    ui->plainTextEdit_logInfo->appendPlainText(sendData);

};

void Radar::hpUpdate(std::vector<std::array<int,3>> hp)
{
    for(int i = 0;i<hp.size();i++)
    {
        if(hp[i][1]==0)
        {
            switch (hp[i][2])
            {
            case 1:
                ui->progressBar_R1->setValue(hp[i][0]);
                break;
            case 2:
                ui->progressBar_R2->setValue(hp[i][0]);
                break;
            case 3:
                ui->progressBar_R3->setValue(hp[i][0]);
                break;
            case 4:
                ui->progressBar_R4->setValue(hp[i][0]);
                break;
            case 5:
                ui->progressBar_R5->setValue(hp[i][0]);
                break;
            case 6:
                ui->progressBar_RGuard->setValue(hp[i][0]);
                break;
            case 7:
                ui->progressBar_ROutPos->setValue(hp[i][0]);
                break;
            case 8:
                ui->progressBar_RBase->setValue(hp[i][0]);
                break;
            default:
                break;
            }

        }
        else if(hp[i][1]==1)
        {

            switch (hp[i][2])
            {
            case 1:
                ui->progressBar_B1->setValue(hp[i][0]);
                break;
            case 2:
                ui->progressBar_B2->setValue(hp[i][0]);
                break;
            case 3:
                ui->progressBar_B3->setValue(hp[i][0]);
                break;
            case 4:
                ui->progressBar_B4->setValue(hp[i][0]);
                break;
            case 5:
                ui->progressBar_B5->setValue(hp[i][0]);
                break;
            case 6:
                ui->progressBar_BGuard->setValue(hp[i][0]);
                break;
            case 7:
                ui->progressBar_BOutPos->setValue(hp[i][0]);
                break;
            case 8:
                ui->progressBar_BBase->setValue(hp[i][0]);
                break;
            default:
                break;
            }
        }
        else
            continue;
    }

};
void Radar::roiUpdate(QRectF roi)
{
    ui->graphicsView_roi->fitInView(roi);
};
Radar::~Radar()
{
    delete mapScene;
    delete streamScene;
    delete ui;
}
