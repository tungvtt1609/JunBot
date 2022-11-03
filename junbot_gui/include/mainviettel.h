#ifndef MAINVIETTEL_H
#define MAINVIETTEL_H

#include <QWidget>
#include <QMainWindow>
#include <sensor_msgs/BatteryState.h>

#include <QComboBox>
#include <QDesktopWidget>
#include <QHBoxLayout>
#include <QQueue>
#include <QSoundEffect>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QTimer>
#include <QTreeWidgetItem>
#include <QVBoxLayout>
#include <QVariant>
#include <map>
#include <QProcess>
#include <QPushButton>

#include "QRobotUltis.h"
#include "QRobotItem.h"
#include "QJoyStick.h"
#include "QNode.h"

QT_BEGIN_NAMESPACE
namespace Ui
{
  class MainViettel;
}
QT_END_NAMESPACE


class MainViettel : public QWidget
{
  Q_OBJECT

public:
  explicit MainViettel(int argc, char** argv, QWidget *parent = nullptr);
  ~MainViettel();

  bool connectMaster(QString master_ip, QString ros_ip, bool use_envirment = false);

private:
  Ui::MainViettel *ui;
  bool isPressedWidget;
  QPoint m_lastPos;
  QNode m_qnode;
  QStandardItemModel *treeView_rviz_model = nullptr;

  // Store the address of the control currently displayed
  // by the rviz treewidget and the parent of the control
  QMap<QWidget *, QTreeWidgetItem *> widget_to_parentItem_map;

  // Store the corresponding relationship of the status bar display name status item
  QMap<QString, QTreeWidgetItem *> tree_rviz_stues;

  // Store the current value of display item name, parameter name and value
  QMap<QTreeWidgetItem *, QMap<QString, QString>> tree_rviz_values;

  bool m_useEnviorment = false;
  bool m_autoConnect = false;
  AppEnums::QDisplayMode m_showMode;
  QString m_masterUrl;
  QString m_hostUrl;
  double m_turnLightThre = 0.1;
  QGraphicsScene *m_qgraphicsScene = nullptr;
  QRobotItem *m_roboItem = nullptr;
  QVariantList m_sendVelList, m_recvVelList, m_timeList;

  int line_max = 10;

  QTimer *m_timerChart;
  QTimer *m_timerPubImageMap;
  QTimer *m_timerCurrentTime;
};

#endif // MAINVIETTEL_H
