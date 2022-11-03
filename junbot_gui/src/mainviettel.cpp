#include "mainviettel.h"
#include "ui_mainviettel.h"
#include <QGraphicsScene>
#include <QCheckBox>
#include <QMessageBox>
#include <QVector>

MainViettel::MainViettel(int argc, char **argv, QWidget *parent) :
  QWidget(parent),
  ui(new Ui::MainViettel),
  m_qnode(argc, argv)
{
  ui->setupUi(this);
}

MainViettel::~MainViettel()
{
  delete ui;
}

bool MainViettel::connectMaster(QString master_ip, QString ros_ip, bool use_envirment)
{

}
