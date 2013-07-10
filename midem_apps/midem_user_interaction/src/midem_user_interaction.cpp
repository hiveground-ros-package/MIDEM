#include <midem_user_interaction/midem_user_interaction.h>
#include "ui_user_interaction.h"
#include <QtGui>

MidemUserInteraction::MidemUserInteraction(QMainWindow *parent) :
  QMainWindow(parent),
  ui(new Ui::MidemUserInteraction),
  quit_thread_(false),
  nhp_("~")
{
  ui->setupUi(this);
  dynamic_reconfigure::Server<midem_user_interaction::MidemUserInteractionConfig>::CallbackType f;
  f = boost::bind(&MidemUserInteraction::callbackConfig, this, _1, _2);
  reconfigure_server_.setCallback(f);

}

MidemUserInteraction::~MidemUserInteraction()
{
  delete ui;
}

void MidemUserInteraction::callbackConfig(midem_user_interaction::MidemUserInteractionConfig &config, uint32_t level)
{
  ROS_INFO("Get config: %f", config.min_x);
}

void MidemUserInteraction::closeEvent(QCloseEvent *event)
{
  quit_thread_ = true;
  event->accept();
}


