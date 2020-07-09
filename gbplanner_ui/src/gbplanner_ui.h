#ifndef GBPLANNER_UI_H
#define GBPLANNER_UI_H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <stdio.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <planner_msgs/pci_global.h>

#ifndef Q_MOC_RUN
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <rviz/panel.h>
#endif

class QLineEdit;
class QPushButton;

namespace gbplanner_ui {
class gbplanner_panel : public rviz::Panel {
  Q_OBJECT
 public:
  gbplanner_panel(QWidget* parent = 0);
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

 public Q_SLOTS:
  void on_start_planner_click();
  void on_stop_planner_click();
  void on_homing_click();
  void on_global_planner_click();
 protected Q_SLOTS:

 protected:
  QPushButton* button_start_planner;
  ros::ServiceClient planner_client_start_planner;

  QPushButton* button_stop_planner;
  ros::ServiceClient planner_client_stop_planner;

  QPushButton* button_homing;
  ros::ServiceClient planner_client_homing;

  QPushButton* button_global_planner;
  QLineEdit* global_id_line_edit;
  ros::ServiceClient planner_client_global_planner;

  ros::NodeHandle nh;
};

}  // namespace gbplanner_ui

#endif  // GBPLANNER_UI_H
