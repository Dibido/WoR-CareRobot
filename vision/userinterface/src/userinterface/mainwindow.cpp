#include "userinterface/mainwindow.hpp"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow),
      mGoalPublisher("goal"),
      mDropLocationPublisher("droplocation")
{
  ui->setupUi(this);

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this,
          QOverload<>::of(&MainWindow::returnToMain));
  timer->start(100);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_water_btn_clicked()
{
  onWaterBtnClicked(false);
}

void MainWindow::on_static_glass_btn_clicked()
{
  onWaterBtnClicked(true);
}

void MainWindow::on_return_btn_clicked()
{
  mDropLocationPublisher.selectGoalPosition(
      userinterface::goal_constants::mDemoPos, false);
  QTimer::singleShot(3000, this,
                     QOverload<>::of(&MainWindow::sendReturnLocation));
}

void MainWindow::sendReturnLocation()
{
  mGoalPublisher.selectGoalPosition(
      userinterface::goal_constants::mDemoReturnPos, false);
  mReleaseTimePublisher.selectReleaseTime(
      userinterface::goal_constants::cReleaseTime_s);
}

void MainWindow::onWaterBtnClicked(bool aStaticCup)
{
  // Show ProgressScreen
  progressWindow.setActive(true);
  progressWindow.show();
  hide();

  // Create and send the message
  mGoalPublisher.selectGoalPosition(userinterface::goal_constants::mDemoPos,
                                    aStaticCup);

  // Mark msg as sent and change button to next step (see below)
  mGoalPublisher.mMsgSent = true;
  return;
}

void MainWindow::returnToMain()
{
  if (!isVisible() && !progressWindow.isVisible())
  {
    show();
  }
}
