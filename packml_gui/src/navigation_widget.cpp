#include <QString>
#include <ros/console.h>
#include "packml_gui/navigation_widget.h"
//#include "ui_packml_navigation.h"
#include "ui_packml.h"
#include <string> 
#include <iomanip> // setprecision
#include <sstream> // stringstream

packml_gui::NavigationWidget::NavigationWidget(QWidget* parent)
    : QWidget(parent)
{
  ros::NodeHandle nh_;
  transition_client_ = nh_.serviceClient<packml_msgs::Transition>("/packml_node/packml/transition");
  //status_subscriber_ = nh.subscribe("/packml_node/packml/status", 1, &packml_gui::NavigationWidget::statusCallBack, this);

  // UI setup
  ui_ = new Ui::NavigationWidget;
  ui_->setupUi(this);

  connect(ui_->start_button, SIGNAL(clicked()), this, SLOT(onStartButton()));
  connect(ui_->abort_button, SIGNAL(clicked()), this, SLOT(onAbortButton()));
  connect(ui_->clear_button, SIGNAL(clicked()), this, SLOT(onClearButton()));
  connect(ui_->hold_button, SIGNAL(clicked()), this, SLOT(onHoldButton()));
  connect(ui_->reset_button, SIGNAL(clicked()), this, SLOT(onResetButton()));
  connect(ui_->unsuspend_button, SIGNAL(clicked()), this, SLOT(onUnsuspendButton()));
  connect(ui_->unhold_button, SIGNAL(clicked()), this, SLOT(onUnholdButton()));
  connect(ui_->suspend_button, SIGNAL(clicked()), this, SLOT(onSuspendButton()));
  connect(ui_->stop_button, SIGNAL(clicked()), this, SLOT(onStopButton()));

  status_sub_= nh_.subscribe("allStatus", 1000, &packml_gui::NavigationWidget::statusCallBack, this); //type is AllStatus
  time_sub_ = nh_.subscribe("allTimes", 1000, &packml_gui::NavigationWidget::callbackTime, this); //type is AllTimes
  mode_sub_ = nh_.subscribe("plcMode", 1000, &packml_gui::NavigationWidget::callbackMode, this); //type is int8

}

void packml_gui::NavigationWidget::statusCallBack(const packml_msgs::AllStatus::ConstPtr& msg){
  updateButtonState(msg);
  //updateStatusField(msg);
}

void packml_gui::NavigationWidget::updateStatusField(const packml_msgs::Status &msg){
  //updateStateField(msg.state);
  //ui_->substate->setText(msg.sub_state));
  //updateModeField(msg.mode);
  //ui_->error_code->setText(QString::fromStdString(msg.error));
  //ui_->suberror_code->setText(QString::fromStdString(msg.sub_error));
}

/*
void packml_gui::NavigationWidget::updateStateField(const packml_msgs::State &state){
  if (state.val == packml_msgs::State::UNDEFINED){
    ui_->state_name->setText(QString::fromStdString("UNDEFINED"));
  }
  else if (state.val == packml_msgs::State::OFF){
    ui_->state_name->setText(QString::fromStdString("OFF"));
  }
  else if (state.val == packml_msgs::State::STOPPED){
    ui_->state_name->setText(QString::fromStdString("STOPPED"));
  }
  else if (state.val == packml_msgs::State::STARTING){
    ui_->state_name->setText(QString::fromStdString("STARTING"));
  }
  else if (state.val == packml_msgs::State::IDLE){
    ui_->state_name->setText(QString::fromStdString("IDLE"));
  }
  else if (state.val == packml_msgs::State::SUSPENDED){
    ui_->state_name->setText(QString::fromStdString("SUSPENDED"));
  }
  else if (state.val == packml_msgs::State::EXECUTE){
    ui_->state_name->setText(QString::fromStdString("EXECUTE"));
  }
  else if (state.val == packml_msgs::State::STOPPING){
    ui_->state_name->setText(QString::fromStdString("STOPPING"));
  }
  else if (state.val == packml_msgs::State::ABORTING){
    ui_->state_name->setText(QString::fromStdString("ABORTING"));
  }
  else if (state.val == packml_msgs::State::ABORTED){
    ui_->state_name->setText(QString::fromStdString("ABORTED"));
  }
  else if (state.val == packml_msgs::State::HOLDING){
    ui_->state_name->setText(QString::fromStdString("HOLDING"));
  }
  else if (state.val == packml_msgs::State::HELD){
    ui_->state_name->setText(QString::fromStdString("HELD"));
  }
  else if (state.val == packml_msgs::State::RESETTING){
    ui_->state_name->setText(QString::fromStdString("RESETTING"));
  }
  else if (state.val == packml_msgs::State::SUSPENDING){
    ui_->state_name->setText(QString::fromStdString("SUSPENDING"));
  }
  else if (state.val == packml_msgs::State::UNSUSPENDING){
    ui_->state_name->setText(QString::fromStdString("UNSUSPENDING"));
  }
  else if (state.val == packml_msgs::State::CLEARING){
    ui_->state_name->setText(QString::fromStdString("CLEARING"));
  }
  else if (state.val == packml_msgs::State::UNHOLDING){
    ui_->state_name->setText(QString::fromStdString("UNHOLDING"));
  }
  else if (state.val == packml_msgs::State::COMPLETING){
    ui_->state_name->setText(QString::fromStdString("COMPLETING"));
  }
  else if (state.val == packml_msgs::State::COMPLETE){
    ui_->state_name->setText(QString::fromStdString("COMPLETE"));
  }
  else if (state.val == packml_msgs::State::UNDEFINED){
    ui_->state_name->setText(QString::fromStdString("UNDEFINED"));
  }
  else{
    ui_->state_name->setText(QString::fromStdString("UNKNOWN"));
  }

}


void packml_gui::NavigationWidget::updateModeField(const packml_msgs::Mode &mode){
  if (mode.val == packml_msgs::Mode::UNDEFINED){
    ui_->mode_name->setText(QString::fromStdString("UNDEFINED"));
  }
  else if (mode.val == packml_msgs::Mode::AUTOMATIC){
    ui_->mode_name->setText(QString::fromStdString("AUTOMATIC"));
  }
  else if (mode.val == packml_msgs::Mode::SEMI_AUTOMATIC){
    ui_->mode_name->setText(QString::fromStdString("SEMI-AUTOMATIC"));
  }
  else if (mode.val == packml_msgs::Mode::MANUAL){
    ui_->mode_name->setText(QString::fromStdString("MANUAL"));
  }
  else if (mode.val == packml_msgs::Mode::IDLE){
    ui_->mode_name->setText(QString::fromStdString("IDLE"));
  }
  else if (mode.val == packml_msgs::Mode::SETUP){
    ui_->mode_name->setText(QString::fromStdString("SETUP"));
  }
  else{
    ui_->mode_name->setText(QString::fromStdString("UNKNOWN"));
  }
}



*/


packml_gui::NavigationWidget::~NavigationWidget(){}

void packml_gui::NavigationWidget::onStartButton(){
  ROS_INFO_STREAM("GUI Start Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::START;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onAbortButton(){
  ROS_INFO_STREAM("GUI Abort Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::ABORT;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onClearButton(){
  ROS_INFO_STREAM("GUI Clear Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::CLEAR;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onHoldButton(){
  ROS_INFO_STREAM("GUI Hold Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::HOLD;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onResetButton(){
  ROS_INFO_STREAM("GUI Reset Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::RESET;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onUnsuspendButton(){
  ROS_INFO_STREAM("GUI Unsuspend Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::UNSUSPEND;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onUnholdButton(){
  ROS_INFO_STREAM("GUI Unhold Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::UNHOLD;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onSuspendButton(){
  ROS_INFO_STREAM("GUI Suspend Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::SUSPEND;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::onStopButton(){
  ROS_INFO_STREAM("GUI Stop Button Pressed");
  packml_msgs::Transition trans;
  trans.request.command = packml_msgs::TransitionRequest::STOP;
  ROS_INFO_STREAM(transition_client_.call(trans));
}

void packml_gui::NavigationWidget::updateButtonState(const packml_msgs::AllStatus::ConstPtr& msg){
	disableAllButtons();
	
	if (msg->stopped_state == true){
		
		ui_->stopped_state->setStyleSheet("QLabel { color : red; }");
		ui_->abort_button->setEnabled(true);
		ui_->reset_button->setEnabled(true);
	}
	else {
		ui_->stopped_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->idle_state == true){
		
		ui_->idle_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
		ui_->start_button->setEnabled(true);
	}
	else {
		ui_->idle_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->starting_state == true){
		
		ui_->starting_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->starting_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->execute_state == true){
		
		ui_->execute_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
		ui_->hold_button->setEnabled(true);
    		ui_->suspend_button->setEnabled(true);
	}
	else {
		ui_->execute_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->completing_state == true){
		
		ui_->completing_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->completing_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->complete_state == true){
		
		ui_->complete_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
		ui_->reset_button->setEnabled(true);
	}
	else {
		ui_->complete_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->suspended_state == true){
		
		ui_->suspended_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
		ui_->unsuspend_button->setEnabled(true);
	}
	else {
		ui_->suspended_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->aborting_state == true){
		
		ui_->aborting_state->setStyleSheet("QLabel { color : red; }");
	}
	else {
		ui_->aborting_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->aborted_state == true){
		
		ui_->aborted_state->setStyleSheet("QLabel { color : red; }");
		ui_->clear_button->setEnabled(true);
	}
	else {
		ui_->aborted_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->holding_state == true){
		
		ui_->holding_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->holding_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->held_state == true){
		
		ui_->held_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
		ui_->unhold_button->setEnabled(true);
	}
	else {
		ui_->held_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->unholding_state == true){
		
		ui_->unholding_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->unholding_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->suspending_state == true){
		
		ui_->suspending_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->suspending_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->unsuspending_state == true){
		
		ui_->unsuspending_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->unsuspending_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->resetting_state == true){
		
		ui_->resetting_state->setStyleSheet("QLabel { color : red; }");
		ui_->stop_button->setEnabled(true);
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->resetting_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->clearing_state == true){
		
		ui_->clearing_state->setStyleSheet("QLabel { color : red; }");
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->clearing_state->setStyleSheet("QLabel { color : black; }");
	}
	if (msg->stopping_state == true){
		
		ui_->stopping_state->setStyleSheet("QLabel { color : red; }");
		ui_->abort_button->setEnabled(true);
	}
	else {
		ui_->stopping_state->setStyleSheet("QLabel { color : black; }");
	}
}

void packml_gui::NavigationWidget::setMessage(const std::string text){
  ui_->message_box->setText(QString::fromStdString(text));
}

void packml_gui::NavigationWidget::disableAllButtons(){
  ui_->abort_button->setEnabled(false);
  ui_->clear_button->setEnabled(false);
  ui_->hold_button->setEnabled(false);
  ui_->reset_button->setEnabled(false);
  ui_->start_button->setEnabled(false);
  ui_->stop_button->setEnabled(false);
  ui_->suspend_button->setEnabled(false);
  ui_->unhold_button->setEnabled(false);
  ui_->unsuspend_button->setEnabled(false);
}



void packml_gui::NavigationWidget::callbackTime(const packml_msgs::AllTimes::ConstPtr& msg){
	std::stringstream stream0;
	stream0 << std::fixed << std::setprecision(2) << msg->stopped_state;
	std::string str_stopped = stream0.str();
	ui_->stopped_state->setText(QString::fromStdString("Stopped: " + str_stopped +"s")); 
	std::stringstream stream1;
	stream1 << std::fixed << std::setprecision(2) << msg->idle_state;
	std::string str_idle = stream1.str();
	ui_->idle_state->setText(QString::fromStdString("Idle: " + str_idle +"s")); 
	std::stringstream stream2;
	stream2 << std::fixed << std::setprecision(2) << msg->starting_state;
	std::string str_starting = stream2.str();
	ui_->starting_state->setText(QString::fromStdString("Starting: " + str_starting +"s")); 
 	std::stringstream stream3;
	stream3 << std::fixed << std::setprecision(2) << msg->execute_state;
	std::string str_execute = stream3.str();
	ui_->execute_state->setText(QString::fromStdString("Executing: " + str_execute +"s")); 
	std::stringstream stream4;
	stream4 << std::fixed << std::setprecision(2) << msg->completing_state;
	std::string str_completing = stream4.str();
	ui_->completing_state->setText(QString::fromStdString("Completing: " + str_completing +"s")); 
	std::stringstream stream5;
	stream5 << std::fixed << std::setprecision(2) << msg->complete_state;
	std::string str_complete = stream5.str();
	ui_->complete_state->setText(QString::fromStdString("Complete: " + str_complete +"s")); 
	std::stringstream stream6;
	stream6 << std::fixed << std::setprecision(2) << msg->suspended_state;
	std::string str_suspended = stream6.str();
	ui_->suspended_state->setText(QString::fromStdString("Suspended: " + str_suspended +"s")); 
	std::stringstream stream7;
	stream7 << std::fixed << std::setprecision(2) << msg->aborting_state;
	std::string str_aborting = stream7.str();
	ui_->aborting_state->setText(QString::fromStdString("Aborting: " + str_aborting +"s")); 
	std::stringstream stream8;
	stream8 << std::fixed << std::setprecision(2) << msg->aborted_state;
	std::string str_aborted= stream8.str();
	ui_->aborted_state->setText(QString::fromStdString("Aborted: " + str_aborted +"s")); 
	std::stringstream stream9;
	stream9 << std::fixed << std::setprecision(2) << msg->holding_state;
	std::string str_holding= stream9.str();
	ui_->holding_state->setText(QString::fromStdString("Holding: " + str_holding +"s"));
	std::stringstream stream10;
	stream10 << std::fixed << std::setprecision(2) << msg->held_state;
	std::string str_held = stream10.str();
	ui_->held_state->setText(QString::fromStdString("Held: " + str_held +"s"));  
	std::stringstream stream11;
	stream11 << std::fixed << std::setprecision(2) << msg->unholding_state;
	std::string str_unholding = stream11.str();
	ui_->unholding_state->setText(QString::fromStdString("Unholding: " + str_unholding +"s")); 
	std::stringstream stream12;
	stream12 << std::fixed << std::setprecision(2) << msg->suspending_state;
	std::string str_suspending = stream12.str();;
	ui_->suspending_state->setText(QString::fromStdString("Suspending: " + str_suspending +"s")); 
	std::stringstream stream13;
	stream13 << std::fixed << std::setprecision(2) << msg->unsuspending_state;
	std::string str_unsuspending = stream13.str();
	ui_->unsuspending_state->setText(QString::fromStdString("Unsuspending: " + str_unsuspending +"s")); 
	std::stringstream stream14;
	stream14 << std::fixed << std::setprecision(2) << msg->resetting_state;
	std::string str_resetting = stream14.str();
	ui_->resetting_state->setText(QString::fromStdString("Resetting: " + str_resetting +"s")); 
	std::stringstream stream15;
	stream15 << std::fixed << std::setprecision(2) << msg->clearing_state;
	std::string str_clearing = stream15.str();
	ui_->clearing_state->setText(QString::fromStdString("Clearing: " + str_clearing +"s")); 
	std::stringstream stream16;
	stream16 << std::fixed << std::setprecision(2) << msg->stopping_state;
	std::string str_stopping = stream16.str();
	ui_->stopping_state->setText(QString::fromStdString("Stopping: " + str_stopping +"s")); 
	
}

void packml_gui::NavigationWidget::callbackMode(const std_msgs::Int8::ConstPtr& msg){
	std::string str_mode = std::to_string(msg->data+1);
	ui_->mode_state->setText(QString::fromStdString("Mode: "+ str_mode));


}


