#ifndef PACKML_NAVIGATION_WIDGET_H
#define PACKML_NAVIGATION_WIDGET_H

#include <QWidget>
#include <ros/ros.h>
#include "packml_msgs/Transition.h"
#include "packml_msgs/State.h"
#include "packml_msgs/Status.h"
#include "packml_msgs/AllTimes.h"
#include "packml_msgs/AllStatus.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Bool.h"

namespace Ui
{
class NavigationWidget;
}

namespace packml_gui
{

class NavigationWidget : public QWidget
{
  Q_OBJECT
public:
  NavigationWidget(QWidget* parent = 0);

  virtual ~NavigationWidget();

public Q_SLOTS:
  void onStartButton();
  void onAbortButton();
  void onClearButton();
  void onHoldButton();
  void onResetButton();
  void onUnsuspendButton();
  void onUnholdButton();
  void onSuspendButton();
  void onStopButton();

  
public:
  // UI
  Ui::NavigationWidget* ui_;
  void setMessage(const std::string text);
  //void updateButtons(packml_msgs::State state);
  void callbackTime(const packml_msgs::AllTimes::ConstPtr& msg);
  void callbackMode(const std_msgs::Int8::ConstPtr& msg);
  //void updateButtonState(const packml_msgs::State &state);
  void updateButtonState(const packml_msgs::AllStatus::ConstPtr& msg);
  void updateStatusField(const packml_msgs::Status &msg);
  //void updateStateField(const packml_msgs::State &state);
  //void statusCallBack(const packml_msgs::Status &msg);
  void statusCallBack(const packml_msgs::AllStatus::ConstPtr& msg);
  void disableAllButtons();
  //void updateModeField(const packml_msgs::Mode &mode);

  ros::ServiceClient transition_client_;
  ros::Subscriber status_sub_, time_sub_, mode_sub_; 
  //ros::Subscriber status_subscriber_;

};
}

#endif
