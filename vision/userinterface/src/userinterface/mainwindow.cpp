#include "userinterface/mainwindow.hpp"
#include "ui_mainwindow.h"

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
  onWaterBtnClicked(false);
}

void MainWindow::on_static_glass_btn_clicked()
{
  onWaterBtnClicked(true);
}

void MainWindow::onWaterBtnClicked(bool aStaticCup)
{
  // Show ProgressScreen
  progressWindow.setActive(true);
  progressWindow.show();
  hide();

  // If the goal msg hasn't been sent, do so to initiate the pickup sequence.
  if (mGoalPublisher.mMsgSent == false)
  {
    // Create and send the message
    mGoalPublisher.selectGoalPosition(userinterface::goal_constants::mDemoPos,
                                      aStaticCup);

    // Mark msg as sent and change button to next step (see below)
    mGoalPublisher.mMsgSent = true;
    return;
  }

  // If the goal msg has been sent but the release time msg hasn't, follow this
  // step.
  if (mGoalPublisher.mMsgSent == true)
  {
    // Start an QElapsedTimer, to check the elapsed time.
    QElapsedTimer lTimer;
    lTimer.start();

    // Send the release time msg.
    mReleaseTimePublisher.selectReleaseTime(
        userinterface::goal_constants::cReleaseTime_s);

    // Reset mMsgSent variable to reset button state and allow consecutive
    // executions.
    mGoalPublisher.mMsgSent = false;
  }
}
