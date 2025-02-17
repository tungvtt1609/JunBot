#include "QLoginWidget.h"
#include "AppConstants.h"
#include "ui_loginwidget.h"

#include <QCompleter>
#include <QDebug>
#include <QHostAddress>
#include <QMessageBox>
#include <QMovie>
#include <QNetworkInterface>
#include <QPainter>
#include <QPropertyAnimation>
#include <QStringListModel>

QLoginWidget::QLoginWidget(QWidget *parent)
  : QCustomMoveWidget(parent),
    ui(new Ui::LoginWidget)
{
  ui->setupUi(this);
  ui->btnWinClose->setIcon(QIcon(":/image/data/images/close.png"));
  ui->btnWinMin->setIcon(QIcon(":/image/data/images/min.png"));
  ui->btnWinClose_2->setIcon(QIcon(":/image/data/images/close.png"));
  ui->btnWinMin_2->setIcon(QIcon(":/image/data/images/min.png"));
  this->setAttribute(Qt::WA_TranslucentBackground);
  this->setWindowFlags(Qt::FramelessWindowHint);

  this->setWindowFlags(Qt::FramelessWindowHint);

  InitWidget();

  readSettings();

  QImage img(":/background/data/background/vmc_1.jpg");

  ui->label_video->setPixmap(QPixmap::fromImage(img).scaled(ui->label_video->size(), Qt::KeepAspectRatioByExpanding, Qt::SmoothTransformation));

  // check IP in local network
  foreach (QHostAddress address, QNetworkInterface::allAddresses()) {
    if (address.protocol() == QAbstractSocket::IPv4Protocol) {
      QString addre = address.toString();

      CONSOLE << addre;

      if (addre.split(".")[0] == "192") {
        m_qRosIp = addre;
        m_qMasterIp = "http://" + addre + ":11311";
      } else if (addre.split(".")[0] == "10") {
        m_qRosIp = addre;
        m_qMasterIp = "http://" + addre + ":11311";
      } else if (addre.split(".")[0] == "172") {
        m_qRosIp = addre;
        m_qMasterIp = "http://" + addre + ":11311";
      }
    }
  }

  // Set ip for ROS master connection
  connect(ui->pushButton_auto, &QPushButton::clicked, [=]() {
    ui->lineEditMasterIp->setText(m_qMasterIp);
    ui->lineEditRosIp->setText(m_qRosIp);
  });

  connect(ui->pushButton_setting, &QPushButton::clicked, [=]() {
    // Setting topic page
    ui->stackedWidget->setCurrentIndex(1);
  });

  // Hint
  connect(ui->pushButton_hellp, &QPushButton::clicked, [=]() {
    QDesktopServices::openUrl(
          QUrl(QString("https://github.com/ScarecrowStraw/JunBotGUI/blob/main/README.md")));
  });

  connect(ui->pushButton_return, &QPushButton::clicked, [=]() {
    QMessageBox msg(this);
    msg.setWindowTitle("back to main interface");
    msg.setText("Whether to save the settings");
    msg.setIcon(QMessageBox::Information);
    msg.setStandardButtons(QMessageBox::Yes |
                           QMessageBox::No);

    if (msg.exec() == QMessageBox::Yes)
    {
      slot_writeSettings();
    }

    // login page (local IP and Master address)
    ui->stackedWidget->setCurrentIndex(0);
  });

  // Window size setting
  connect(ui->btnWinClose, &QPushButton::clicked, [=]() {
    this->close();
  });
  connect(ui->btnWinMin, &QPushButton::clicked, [=]() {
    this->showMinimized();
  });
  connect(ui->btnWinClose_2, &QPushButton::clicked, [=]() {
    this->close();
  });
  connect(ui->btnWinMin_2, &QPushButton::clicked, [=]() {
    this->showMinimized();
  });

  // saving setting
  connect(ui->pushButton_save, SIGNAL(clicked()), this, SLOT(slot_writeSettings()));

  if (ui->checkBoxAutoLogin->isChecked()) {
    ui->btnLogin->setText("CANCEL");
    ui->btnLogin->setStyleSheet(
          "border:0px;background-color:rgb(211, 215, 207);color:WHITE；");
    QTimer::singleShot(2000, this, SLOT(slot_autoLoad()));
  }
}

QLoginWidget::~QLoginWidget()
{
  delete ui;
}

void QLoginWidget::slot_autoLoad()
{
  if (m_bIsConnect) {
    ConnectMaster();
  }
}

// App setting value
void QLoginWidget::slot_writeSettings()
{
  QSettings main_setting("junbot_gui", "settings");
  main_setting.setValue("topic/topic_odom", ui->lineEdit_odm->text());
  main_setting.setValue("topic/topic_power", ui->lineEdit_power->text());
  main_setting.setValue("topic/topic_goal", ui->lineEdit_goal_topic->text());
  main_setting.setValue("topic/topic_init_pose",
                        ui->lineEdit_start_postopic->text());
  main_setting.setValue("main/turn_thre", ui->lineEdit_turnLightThre->text());
  main_setting.setValue("main/show_mode", ui->radioButton_robot->isChecked()
                        ? "robot"
                        : "control");
  main_setting.setValue("main/thread_num", ui->spinBox_thread_num->value());
  main_setting.setValue("main/framerate", ui->spinBox_frameRate->value());
  main_setting.setValue("frame/laserFrame", ui->lineEdit_laserFrame->text());
  main_setting.setValue("frame/mapFrame", ui->lineEdit_mapFrame->text());
  main_setting.setValue("frame/baseFrame", ui->lineEdit_baseFrame->text());
  QStringList name_data;
  QStringList topic_data;
  QStringList format_data;
  name_data.append(ui->video0_name_set->text());
  name_data.append(ui->video0_name_set_2->text());
  name_data.append(ui->video0_name_set_3->text());
  name_data.append(ui->video0_name_set_4->text());
  topic_data.append(ui->video0_topic_set->text());
  topic_data.append(ui->video0_topic_set_2->text());
  topic_data.append(ui->video0_topic_set_3->text());
  topic_data.append(ui->video0_topic_set_4->text());
  main_setting.setValue("video/names", name_data);
  main_setting.setValue("video/topics", topic_data);

  QSettings settings("junbot_gui", "Displays");
  settings.setValue("Grid/enable", Grid_Check->isChecked());
  settings.setValue("GlobalOption/FixedFrame", fixed_box->currentText());
  settings.setValue("Grid/count", Cell_Count_Box->text());
  settings.setValue("Map/enable", Map_Check->isChecked());
  settings.setValue("Map/topic", Map_Topic_box->currentText());
  settings.setValue("Map/scheme", Map_Color_Scheme_box->currentText());
  // settings.value("Map/scheme",Map_Color_Scheme_box->);
  settings.setValue("Laser/enable", Laser_Check->isChecked());
  settings.setValue("Laser/topic", Laser_Topic_box->currentText());
  settings.setValue("Polygon/enable", Polygon_Check->isChecked());
  settings.setValue("Polygon/topic", Polygon_Topic_box->currentText());
  settings.setValue("RobotModel/enable", RobotModel_Check->isChecked());
  settings.setValue("GlobalMap/enable", GlobalMap_Check->isChecked());
  settings.setValue("GlobalMap/topic", Global_CostMap_Topic_box->currentText());
  settings.setValue("GlobalPlan/topic",
                    Global_Planner_Topic_box->currentText());
  settings.setValue("GlobalPlan/color",
                    Global_Planner_Color_box->currentText());
  settings.setValue("LocalMap/enable", LocalMap_Check->isChecked());
  settings.setValue("LocalMap/topic", Local_CostMap_Topic_box->currentText());
  settings.setValue("LocalPlan/topic", Local_Planner_Topic_box->currentText());
  settings.setValue("LocalPlan/color", Local_Planner_Color_box->currentText());
}

void QLoginWidget::changeEvent(QEvent *e) {
  QWidget::changeEvent(e);
  switch (e->type()) {
    case QEvent::LanguageChange:
      ui->retranslateUi(this);
      break;
    default:
      break;
  }
}


void QLoginWidget::readSettings() {
  QSettings settings("junbot_gui", "Displays");
  bool Grid_enable = settings.value("Grid/enable", bool(true)).toBool();
  double Grid_count = settings.value("Grid/count", double(20)).toDouble();
  QString FixedFrame =
      settings.value("GlobalOption/FixedFrame", "map").toString();
  bool Map_enable = settings.value("Map/enable", bool(true)).toBool();
  QString Map_topic = settings.value("Map/topic", QString("/map")).toString();
  double Map_alpha = settings.value("Map/alpha", double(0.7)).toDouble();
  QString Map_scheme = settings.value("Map/scheme", QString("map")).toString();

  bool Laser_enable = settings.value("Laser/enable", bool(true)).toBool();
  QString Laser_topic =
      settings.value("Laser/topic", QString("/scan")).toString();

  bool Polygon_enable = settings.value("Polygon/enable", bool(true)).toBool();
  QString Polygon_topic =
      settings
          .value("Polygon/topic", QString("/move_base/local_costmap/footprint"))
          .toString();

  bool RobotModel_enable =
      settings.value("RobotModel/enable", bool(true)).toBool();
  bool GlobalMap_enable =
      settings.value("GlobalMap/enable", bool(true)).toBool();
  QString GlobalMap_topic =
      settings
          .value("GlobalMap/topic",
                 QString("/move_base/global_costmap/costmap"))
          .toString();
  QString GlobalMap_plan =
      settings.value("GlobalPlan/topic", QString("/move_base/NavfnROS/plan"))
          .toString();
  QString GlobalMap_plan_color =
      settings.value("GlobalPlan/color", QString("255;0;0")).toString();
  bool LocalMap_enable = settings.value("LocalMap/enable", bool(true)).toBool();
  QString LocalMap_topic =
      settings
          .value("LocalMap/topic", QString("/move_base/local_costmap/costmap"))
          .toString();
  QString LocalMap_plan =
      settings
          .value("LocalPlan/topic",
                 QString("/move_base/DWAPlannerROS/local_plan"))
          .toString();
  QString LocalMap_plan_color =
      settings.value("LocalPlan/color", QString("0;0;255")).toString();
  Grid_Check->setChecked(Grid_enable);
  Cell_Count_Box->setValue(Grid_count);
  Map_Check->setChecked(Map_enable);
  Map_Topic_box->setCurrentText(Map_topic);
  Map_Color_Scheme_box->setCurrentText(Map_scheme);
  Laser_Check->setChecked(Laser_enable);
  Laser_Topic_box->setCurrentText(Laser_topic);
  RobotModel_Check->setChecked(RobotModel_enable);
  GlobalMap_Check->setChecked(GlobalMap_enable);
  Global_CostMap_Topic_box->setCurrentText(GlobalMap_topic);
  Global_Planner_Topic_box->setCurrentText(GlobalMap_plan);
  Global_Planner_Color_box->setCurrentText(GlobalMap_plan_color);
  LocalMap_Check->setChecked(LocalMap_enable);
  Local_CostMap_Topic_box->setCurrentText(LocalMap_topic);
  Local_Planner_Topic_box->setCurrentText(LocalMap_plan);
  Local_Planner_Color_box->setCurrentText(LocalMap_plan_color);
  Polygon_Check->setChecked(Polygon_enable);
  Polygon_Topic_box->setCurrentText(Polygon_topic);
  fixed_box->setCurrentText(FixedFrame);

  QSettings main_setting("junbot_gui", "settings");
  QStringList names = main_setting.value("video/names").toStringList();
  QStringList topics = main_setting.value("video/topics").toStringList();
  ui->lineEdit_laserFrame->setText(
      main_setting.value("frame/laserFrame", "/base_scan").toString());
  ui->lineEdit_mapFrame->setText(
      main_setting.value("frame/mapFrame", "/map").toString());
  ui->lineEdit_baseFrame->setText(
      main_setting.value("frame/baseFrame", "/base_link").toString());
  if (names.size() == 4) {
    ui->video0_name_set->setText(names[0]);
    ui->video0_name_set_2->setText(names[1]);
    ui->video0_name_set_3->setText(names[2]);
    ui->video0_name_set_4->setText(names[3]);
  }
  if (topics.size() == 4) {
    ui->video0_topic_set->setText(topics[0]);
    ui->video0_topic_set_2->setText(topics[1]);
    ui->video0_topic_set_3->setText(topics[2]);
    ui->video0_topic_set_4->setText(topics[3]);
  }

  ui->lineEdit_odm->setText(
      main_setting.value("topic/topic_odom", "raw_odom").toString());
  ui->lineEdit_power->setText(
      main_setting.value("topic/topic_power", "power").toString());
  ui->lineEdit_goal_topic->setText(
      main_setting.value("topic/topic_goal", "move_base_simple/goal")
          .toString());
  ui->lineEdit_start_postopic->setText(
      main_setting.value("topic/topic_init_pose", "initialpose").toString());
  ui->lineEdit_turnLightThre->setText(
      main_setting.value("main/turn_thre", "0.2").toString());
  ui->spinBox_frameRate->setValue(
      main_setting.value("main/framerate", 40).toInt());
  ui->spinBox_thread_num->setValue(
      main_setting.value("main/thread_num", 6).toInt());
  QSettings connect_info("junbot_gui", "connect_info");
  ui->lineEditMasterIp->setText(
      connect_info.value("master_url", m_qMasterIp).toString());

  ui->lineEditRosIp->setText(
      connect_info.value("host_url", m_qRosIp).toString());

  ui->checkBoxAutoLogin->setChecked(
      connect_info.value("auto_connect", bool(false)).toBool());
  if (main_setting.value("main/show_mode", "control").toString() == "control") {
    ui->radioButton_control->setChecked(true);
  } else {
    ui->radioButton_robot->setChecked(true);
  }
}


void QLoginWidget::on_btnLogin_clicked() {
  if (ui->btnLogin->text() != "CANCEL") {
    ConnectMaster();
  } else {
    ui->btnLogin->setText("CONNECT");
    ui->btnLogin->setStyleSheet(
        "border:0px;background-color:#F81243;color:WHITE；");
    m_bIsConnect = false;
  }
}

// Connect to ROS master => open MainWindow
void QLoginWidget::ConnectMaster() {
  int argc;
  char **argv;
//  if (mainWindow != NULL) {
//    mainWindow->close();
//  }

  if (mainViettel != NULL) {
    mainViettel->close();
  }
  
  mainViettel = new MainViettel(argc, argv);

//  connect(mainWindow, SIGNAL(signalDisconnect()), this,
//          SLOT(slot_ShowWindow()));

  connect(mainViettel, SIGNAL(signalDisconnect()), this,
          SLOT(slot_ShowWindow()));

  QCoreApplication::processEvents();
  ui->btnLogin->setText("CANCEL");
  ui->btnLogin->setStyleSheet(
      "border:0px;background-color:rgb(211, 215, 207);color:WHITE；");

//  bool isConnect = mainWindow->connectMaster(
//      ui->lineEditMasterIp->text(), ui->lineEditRosIp->text());

  bool isConnect = mainViettel->connectMaster(
      ui->lineEditMasterIp->text(), ui->lineEditRosIp->text());

  if (isConnect) {
    QSettings connect_info("junbot_gui", "connect_info");
    connect_info.setValue("master_url", ui->lineEditMasterIp->text());
    connect_info.setValue("host_url", ui->lineEditRosIp->text());
    connect_info.setValue("auto_connect", ui->checkBoxAutoLogin->isChecked());
    ui->btnLogin->setText("CONNECT");
    ui->btnLogin->setStyleSheet(
        "border:0px;background-color:#F81243;color:WHITE；");
    this->hide();
//    mainWindow->show();

    mainViettel->show();
  } else {
    ui->checkBoxAutoLogin->setChecked(false);
    ui->btnLogin->setText("CONNECT");
    ui->btnLogin->setStyleSheet(
        "border:0px;background-color:#F81243;color:WHITE；");
    QMessageBox::information(NULL, "Connection failed",
                             "Connection failed! Please check your connection configuration or restart and try again!",
                             QMessageBox::Yes);
  }
}

void QLoginWidget::slot_ShowWindow() {
  this->show();
  ui->btnLogin->setEnabled(true);
  ui->btnLogin->setStyleSheet(
      "border:0px;background-color:#F81243;color:WHITE；");
}

void QLoginWidget::SltAnimationFinished()
{
  // pending
}

void QLoginWidget::SltEditFinished()
{
  // pending
}

void QLoginWidget::InitWidget() {
  ui->stackedWidget->setCurrentIndex(0);
  // header
  ui->treeWidget->setHeaderLabels(QStringList() << "key"
                                                << "value");

  // Add UI for treeWidget (Layer Setting)
  // GLobal Options
  QTreeWidgetItem *Global =
      new QTreeWidgetItem(QStringList() << "Global Options");
  Global->setIcon(0, QIcon(":/image/data/images/options.png"));

  ui->treeWidget->addTopLevelItem(Global);
  Global->setExpanded(true);

  // FixFrame
  QTreeWidgetItem *Fixed_frame =
      new QTreeWidgetItem(QStringList() << "Fixed Frame");
  fixed_box = new QComboBox();
  fixed_box->addItem("map");
  fixed_box->setMaximumWidth(150);
  fixed_box->setEditable(true);
  Global->addChild(Fixed_frame);
  ui->treeWidget->setItemWidget(Fixed_frame, 1, fixed_box);

  // Grid
  QTreeWidgetItem *Grid = new QTreeWidgetItem(QStringList() << "Grid");
  Grid->setIcon(0, QIcon(":/image/data/images/classes/Grid.png"));
  // checkbox
  Grid_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(Grid);
  ui->treeWidget->setItemWidget(Grid, 1, Grid_Check);
  Grid->setExpanded(true);

  // Plane Cell
  QTreeWidgetItem *Cell_Count =
      new QTreeWidgetItem(QStringList() << "Plane Cell Count");
  Grid->addChild(Cell_Count);
  Cell_Count_Box = new QSpinBox();
  Cell_Count_Box->setValue(13);
  Cell_Count_Box->setMaximumWidth(150);
  ui->treeWidget->setItemWidget(Cell_Count, 1, Cell_Count_Box);

  // Color
  QTreeWidgetItem *Grid_Color = new QTreeWidgetItem(QStringList() << "Color");
  Grid->addChild(Grid_Color);
  Grid_Color_Box = new QComboBox();
  Grid_Color_Box->addItem("160;160;160");
  Grid_Color_Box->setEditable(true);
  Grid_Color_Box->setMaximumWidth(150);
  ui->treeWidget->setItemWidget(Grid_Color, 1, Grid_Color_Box);

  // TF ui
  QTreeWidgetItem *TF = new QTreeWidgetItem(QStringList() << "TF");
  TF->setIcon(0, QIcon(":/image/data/images/classes/TF.png"));
  // checkbox
  TF_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(TF);
  ui->treeWidget->setItemWidget(TF, 1, TF_Check);

  // LaserScan
  QTreeWidgetItem *LaserScan =
      new QTreeWidgetItem(QStringList() << "LaserScan");
  LaserScan->setIcon(0, QIcon(":/image/data/images/classes/LaserScan.png"));
  // checkbox
  Laser_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(LaserScan);
  ui->treeWidget->setItemWidget(LaserScan, 1, Laser_Check);

  // laser topic
  QTreeWidgetItem *LaserTopic = new QTreeWidgetItem(QStringList() << "Topic");
  Laser_Topic_box = new QComboBox();
  Laser_Topic_box->addItem("/scan");
  Laser_Topic_box->setEditable(true);
  Laser_Topic_box->setMaximumWidth(150);
  LaserScan->addChild(LaserTopic);
  ui->treeWidget->setItemWidget(LaserTopic, 1, Laser_Topic_box);

  // polygon
  // checkbox
  QTreeWidgetItem *Polygon = new QTreeWidgetItem(QStringList() << "Polygon");
  Polygon->setIcon(0, QIcon(":/image/data/images/classes/Polygon.png"));
  Polygon_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(Polygon);
  ui->treeWidget->setItemWidget(Polygon, 1, Polygon_Check);

  // laser topic
  QTreeWidgetItem *PolygonTopic = new QTreeWidgetItem(QStringList() << "Topic");
  Polygon_Topic_box = new QComboBox();
  Polygon_Topic_box->addItem("/move_base/local_costmap/footprint");
  Polygon_Topic_box->setEditable(true);
  Polygon_Topic_box->setMaximumWidth(150);
  Polygon->addChild(PolygonTopic);
  ui->treeWidget->setItemWidget(PolygonTopic, 1, Polygon_Topic_box);

  // RobotModel
  QTreeWidgetItem *RobotModel =
      new QTreeWidgetItem(QStringList() << "RobotModel");
  RobotModel->setIcon(0, QIcon(":/image/data/images/classes/RobotModel.png"));
  // checkbox
  RobotModel_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(RobotModel);
  ui->treeWidget->setItemWidget(RobotModel, 1, RobotModel_Check);

  // Map
  QTreeWidgetItem *Map = new QTreeWidgetItem(QStringList() << "Map");
  Map->setIcon(0, QIcon(":/image/data/images/classes/Map.png"));
  // checkbox
  Map_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(Map);
  ui->treeWidget->setItemWidget(Map, 1, Map_Check);

  // Map topic
  QTreeWidgetItem *MapTopic = new QTreeWidgetItem(QStringList() << "Topic");
  Map_Topic_box = new QComboBox();
  Map_Topic_box->addItem("/map");
  Map_Topic_box->setEditable(true);
  Map_Topic_box->setMaximumWidth(150);
  Map->addChild(MapTopic);
  ui->treeWidget->setItemWidget(MapTopic, 1, Map_Topic_box);

  // Map color scheme
  QTreeWidgetItem *MapColorScheme =
      new QTreeWidgetItem(QStringList() << "Color Scheme");
  Map_Color_Scheme_box = new QComboBox();
  Map_Color_Scheme_box->addItem("map");
  Map_Color_Scheme_box->addItem("costmap");
  Map_Color_Scheme_box->addItem("raw");
  Map_Color_Scheme_box->setMaximumWidth(150);
  Map->addChild(MapColorScheme);
  ui->treeWidget->setItemWidget(MapColorScheme, 1, Map_Color_Scheme_box);

  // Path
  QTreeWidgetItem *Path = new QTreeWidgetItem(QStringList() << "Path");
  Path->setIcon(0, QIcon(":/image/data/images/classes/Path.png"));
  // checkbox
  Path_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(Path);
  ui->treeWidget->setItemWidget(Path, 1, Path_Check);

  // Path topic
  QTreeWidgetItem *PathTopic = new QTreeWidgetItem(QStringList() << "Topic");
  Path_Topic_box = new QComboBox();
  Path_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
  Path_Topic_box->setEditable(true);
  Path_Topic_box->setMaximumWidth(150);
  Path->addChild(PathTopic);
  ui->treeWidget->setItemWidget(PathTopic, 1, Path_Topic_box);

  // Path color scheme
  QTreeWidgetItem *PathColorScheme =
      new QTreeWidgetItem(QStringList() << "Color");
  Path_Color_box = new QComboBox();
  Path_Color_box->addItem("0;12;255");
  Path_Color_box->setEditable(true);
  Path_Color_box->setMaximumWidth(150);
  Path->addChild(PathColorScheme);
  ui->treeWidget->setItemWidget(PathColorScheme, 1, Path_Color_box);

  // Global Map
  QTreeWidgetItem *GlobalMap =
      new QTreeWidgetItem(QStringList() << "Global Map");
  GlobalMap->setIcon(0, QIcon(":/image/data/images/default_package_icon.png"));
  GlobalMap_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(GlobalMap);
  ui->treeWidget->setItemWidget(GlobalMap, 1, GlobalMap_Check);

  // Global CostMap
  QTreeWidgetItem *Global_CostMap =
      new QTreeWidgetItem(QStringList() << "Costmap");
  Global_CostMap->setIcon(0, QIcon(":/image/data/images/classes/Map.png"));
  GlobalMap->addChild(Global_CostMap);

  // Map topic
  QTreeWidgetItem *Global_CostMap_Topic =
      new QTreeWidgetItem(QStringList() << "Topic");
  Global_CostMap_Topic_box = new QComboBox();
  Global_CostMap_Topic_box->addItem("/move_base/global_costmap/costmap");
  Global_CostMap_Topic_box->setEditable(true);
  Global_CostMap_Topic_box->setMaximumWidth(150);
  Global_CostMap->addChild(Global_CostMap_Topic);
  ui->treeWidget->setItemWidget(Global_CostMap_Topic, 1,
                                Global_CostMap_Topic_box);

  // Map color scheme
  QTreeWidgetItem *GlobalMapColorScheme =
      new QTreeWidgetItem(QStringList() << "Color Scheme");
  GlobalMapColorScheme_box = new QComboBox();
  GlobalMapColorScheme_box->addItem("costmap");
  GlobalMapColorScheme_box->addItem("map");
  GlobalMapColorScheme_box->addItem("raw");
  GlobalMapColorScheme_box->setMaximumWidth(150);
  Global_CostMap->addChild(GlobalMapColorScheme);
  ui->treeWidget->setItemWidget(GlobalMapColorScheme, 1,
                                GlobalMapColorScheme_box);

  // Global Planner
  QTreeWidgetItem *Global_Planner =
      new QTreeWidgetItem(QStringList() << "Planner");
  Global_Planner->setIcon(0, QIcon(":/image/data/images/classes/Path.png"));
  GlobalMap->addChild(Global_Planner);

  // Path topic
  QTreeWidgetItem *Global_Planner_Topic =
      new QTreeWidgetItem(QStringList() << "Topic");
  Global_Planner_Topic_box = new QComboBox();
  Global_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/global_plan");
  Global_Planner_Topic_box->setEditable(true);
  Global_Planner_Topic_box->setMaximumWidth(150);
  Global_Planner->addChild(Global_Planner_Topic);
  ui->treeWidget->setItemWidget(Global_Planner_Topic, 1,
                                Global_Planner_Topic_box);

  // Path color scheme
  QTreeWidgetItem *Global_Planner_Color_Scheme =
      new QTreeWidgetItem(QStringList() << "Color");
  Global_Planner_Color_box = new QComboBox();
  Global_Planner_Color_box->addItem("255;0;0");
  Global_Planner_Color_box->setEditable(true);
  Global_Planner_Color_box->setMaximumWidth(150);
  Global_Planner->addChild(Global_Planner_Color_Scheme);
  ui->treeWidget->setItemWidget(Global_Planner_Color_Scheme, 1,
                                Global_Planner_Color_box);

  // Local Map
  QTreeWidgetItem *LocalMap = new QTreeWidgetItem(QStringList() << "Local Map");
  LocalMap->setIcon(0, QIcon(":/image/data/images/default_package_icon.png"));
  LocalMap_Check = new QCheckBox();
  ui->treeWidget->addTopLevelItem(LocalMap);
  ui->treeWidget->setItemWidget(LocalMap, 1, LocalMap_Check);

  // Local CostMap
  QTreeWidgetItem *Local_CostMap =
      new QTreeWidgetItem(QStringList() << "Costmap");
  Local_CostMap->setIcon(0, QIcon(":/image/data/images/classes/Map.png"));
  LocalMap->addChild(Local_CostMap);

  // Map topic
  QTreeWidgetItem *Local_CostMap_Topic =
      new QTreeWidgetItem(QStringList() << "Topic");
  Local_CostMap_Topic_box = new QComboBox();
  Local_CostMap_Topic_box->addItem("/move_base/local_costmap/costmap");
  Local_CostMap_Topic_box->setEditable(true);
  Local_CostMap_Topic_box->setMaximumWidth(150);
  Local_CostMap->addChild(Local_CostMap_Topic);
  ui->treeWidget->setItemWidget(Local_CostMap_Topic, 1,
                                Local_CostMap_Topic_box);

  // Map color scheme
  QTreeWidgetItem *LocalMapColorScheme =
      new QTreeWidgetItem(QStringList() << "Color Scheme");
  LocalMapColorScheme_box = new QComboBox();
  LocalMapColorScheme_box->addItem("costmap");
  LocalMapColorScheme_box->addItem("map");
  LocalMapColorScheme_box->addItem("raw");
  LocalMapColorScheme_box->setMaximumWidth(150);
  Local_CostMap->addChild(LocalMapColorScheme);
  ui->treeWidget->setItemWidget(LocalMapColorScheme, 1,
                                LocalMapColorScheme_box);

  // Local Planner
  QTreeWidgetItem *Local_Planner =
      new QTreeWidgetItem(QStringList() << "Planner");
  Local_Planner->setIcon(0, QIcon(":/image/data/images/classes/Path.png"));
  LocalMap->addChild(Local_Planner);

  // Path topic
  QTreeWidgetItem *Local_Planner_Topic =
      new QTreeWidgetItem(QStringList() << "Topic");
  Local_Planner_Topic_box = new QComboBox();
  Local_Planner_Topic_box->addItem("/move_base/DWAPlannerROS/local_plan");
  Local_Planner_Topic_box->setEditable(true);
  Local_Planner_Topic_box->setMaximumWidth(150);
  Local_Planner->addChild(Local_Planner_Topic);
  ui->treeWidget->setItemWidget(Local_Planner_Topic, 1,
                                Local_Planner_Topic_box);

  // Path color scheme
  QTreeWidgetItem *Local_Planner_Color_Scheme =
      new QTreeWidgetItem(QStringList() << "Color");
  Local_Planner_Color_box = new QComboBox();
  Local_Planner_Color_box->addItem("0;12;255");
  Local_Planner_Color_box->setEditable(true);
  Local_Planner_Color_box->setMaximumWidth(150);
  Local_Planner->addChild(Local_Planner_Color_Scheme);
  ui->treeWidget->setItemWidget(Local_Planner_Color_Scheme, 1,
                                Local_Planner_Color_box);

  // Signal and slot for topic setting widget
  connect(ui->lineEditRosIp, SIGNAL(editingFinished()), this,
          SLOT(SltEditFinished()));
  connect(ui->lineEditRosIp, SIGNAL(returnPressed()), this,
          SLOT(on_btnLogin_clicked()));
  connect(ui->lineEditMasterIp, SIGNAL(returnPressed()), this,
          SLOT(on_btnLogin_clicked()));
}

void QLoginWidget::paintEvent(QPaintEvent *) {
  QPainter painter;
  painter.fillRect(this->rect(), Qt::transparent);
}

void QLoginWidget::on_checkBoxAutoLogin_clicked(bool checked) {
  // pending
}


void QLoginWidget::on_btnLogout_clicked()
{
  if (ui->btnLogout->text() == "LOGOUT") {
    this->close();
  }
}
