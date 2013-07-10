#ifndef MIDEM_USER_INTERACTION_H
#define MIDEM_USER_INTERACTION_H

#include <QMainWindow>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <midem_user_interaction/midem_user_interaction.h>
#include <midem_user_interaction/MidemUserInteractionConfig.h>

namespace Ui {
  class MidemUserInteraction;
}

class MidemUserInteraction : public QMainWindow
{
  Q_OBJECT

public:
  explicit MidemUserInteraction(QMainWindow *parent = 0);
  ~MidemUserInteraction();
  bool isQuit() { return quit_thread_; }



protected: //Local
  void callbackConfig(midem_user_interaction::MidemUserInteractionConfig &config, uint32_t level);

protected: //Qt
  void closeEvent(QCloseEvent *evencurrentItemt);

private:
  Ui::MidemUserInteraction *ui;
  bool quit_thread_;
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  dynamic_reconfigure::Server<midem_user_interaction::MidemUserInteractionConfig> reconfigure_server_;
};

#endif // MIDEM_USER_INTERACTION_H
