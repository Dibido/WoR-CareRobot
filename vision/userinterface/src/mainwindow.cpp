#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QProcess>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_water_btn_clicked()
{
  /* This creates a new QProcess, which allows launching shell commands. Then a
   * command for Gazebo is launched in an detached state, so that the GUI is not
   * blocked when the simulation is running. This is a temporary solution for
   * the demo, as the GUI hasn't the highest priority in sprint 1.
   */
  QProcess process;
  process.startDetached(
      "roslaunch src/wor-18-19-s2/simulation/sim_world/launch/world.launch "
      "world:=current_world paused:=false");
  // process.waitForFinished();
}
