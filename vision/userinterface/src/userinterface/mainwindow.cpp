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

  // Show ProgressScreen

  progressWindow.setActive(true);
  progressWindow.show();
  hide();

  // If the goal msg hasn't been sent, do so to initiate the pickup sequence.
  if (mGoalPublisher.mMsgSent == false)
  {
    // Create and send the message
    mGoalPublisher.selectGoalPosition(userinterface::goal_constants::mDemoPos);

    // Mark msg as sent and change button to next step (see below)
    mGoalPublisher.mMsgSent = true;
    //    QPixmap lPix(":/new/icons/Cross.png");
    //    QIcon lIcon(lPix);
    //    ui->water_btn->setIcon(lIcon);
    //    ui->water_btn->setIconSize(QSize(Ui::ui_constants::c_CrossBtnSize_m,
    //                                     Ui::ui_constants::c_CrossBtnSize_m));
    //    ui->water_btn->setText(QString("Laat beker los"));
    //    qApp->processEvents();
    return;
  }
  //
  // If the goal msg has been sent but the release time msg hasn't, follow this
  // step.
  if (mGoalPublisher.mMsgSent == true)
  {
    //
    //    // Make QFont to store font settings used in this scope.
    //    QFont lStandardFontSetting("Ubuntu", 24);
    //
    // Start an QElapsedTimer, to check the elapsed time.
    QElapsedTimer lTimer;
    lTimer.start();
    //
    // Send the release time msg.
    mReleaseTimePublisher.selectReleaseTime(
        userinterface::goal_constants::cReleaseTime_s);
    //
    //
    //    // Reset mMsgSent variable to reset button state and allow consecutive
    //    // executions.
    mGoalPublisher.mMsgSent = false;
    //
    //    ui->label->setFont(lStandardFontSetting);
    //    ui->label->setText(
    //        "Maak een keuze doormiddel van de knoppen hieronder. Dit object
    //        zal " "voor u worden opgepakt.");
    //
    //    // Change button components back to standard.
    //    QPixmap lPix(":/new/icons/Glas_water.png");
    //    QIcon lIcon(lPix);
    //    ui->water_btn->setIcon(lIcon);
    //    ui->water_btn->setIconSize(QSize(Ui::ui_constants::c_WaterBtnSize_m,
    //                                     Ui::ui_constants::c_WaterBtnSize_m));
    //    ui->water_btn->setText(QString("Beker water"));
  }
}
