#include "mainwindow.hpp"
#include <QApplication>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "userinterface");

  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}
