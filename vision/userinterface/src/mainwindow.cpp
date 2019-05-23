#include "mainwindow.hpp"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QProcess>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_water_btn_clicked()
{
  QProcess process;
  process.startDetached("roslaunch src/wor-18-19-s2/simulation/sim_world/launch/world.launch world:=current_world paused:=false");
  process.waitForFinished();
}
