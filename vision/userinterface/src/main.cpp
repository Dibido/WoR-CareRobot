#include "mainwindow.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <QApplication>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "userinterface");

  QApplication application(argc, argv);
  MainWindow homewindow;
  homewindow.show();

  return application.exec();
}
