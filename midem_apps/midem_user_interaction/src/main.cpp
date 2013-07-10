#include <ros/ros.h>
#include <QtGui/QApplication>
#include <midem_user_interaction/midem_user_interaction.h>
#include <boost/thread.hpp>


MidemUserInteraction* g_app = NULL;
bool g_initialized = false;

void spin_function()
{
  ros::WallRate r(100.0);
  while (ros::ok() && !g_initialized)
  {
    r.sleep();
    ros::spinOnce();
  }
  while (ros::ok() && !g_app->isQuit())
  {
    r.sleep();
    ros::spinOnce();
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "midem_user_interaction", ros::init_options::NoSigintHandler);
  boost::thread spin_thread(boost::bind(&spin_function));

  QApplication a(argc, argv);
  MidemUserInteraction app;
  g_app = & app;
  app.show();

  g_initialized = true;
  int ret = a.exec();

  spin_thread.join();

  return ret;
}
