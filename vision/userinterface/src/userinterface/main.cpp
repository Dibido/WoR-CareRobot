#include "ros/ros.h"
#include "userinterface/mainwindow.hpp"
#include <QApplication>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "userinterface");

  // Initialize QApplication for the GUI.
  QApplication application(argc, argv);

  // Create homewindow.
  MainWindow homewindow;

  // Show homewindow.
  homewindow.show();
  return application.exec();
}
