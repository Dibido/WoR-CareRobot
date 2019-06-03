#include "userinterface/mainwindow.hpp"
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
  uint8_t timeout = 10;
  // If the goal msg hasn't been sent, do so to initiate the pickup sequence.
  if (mGoalPublisher.mMsgSent == false)
  {
    // Create and send the message
    environment_controller::Position mPos(-0.30, 0.30, 0.00);
    mGoalPublisher.selectGoalPosition(mPos);

    // Mark msg as sent and change button to next step (see below)
    mGoalPublisher.mMsgSent = true;
    QPixmap lPix(":/new/icons/Cross.png");
    QIcon lIcon(lPix);
    ui->water_btn->setIcon(lIcon);
    ui->water_btn->setIconSize(QSize(130, 130));
    ui->water_btn->setText(QString("Laat beker los"));
    qApp->processEvents();
    return;
  }

  // If the goal msg has been sent but the release time msg hasn't, follow this
  // step.
  if (mGoalPublisher.mMsgSent == true &&
      mReleaseTimePublisher.mMsgSent == false)
  {
    // Start an QElapsedTimer, to check the elapsed time.
    QElapsedTimer lTimer;
    lTimer.start();

    // Send the release time msg.
    mReleaseTimePublisher.selectReleaseTime(timeout);

    // Countdown from timeout to zero in the GUI, so the user has visual
    // feedback.
    while (lTimer.elapsed() / 1000 <= timeout - 1)
    {
      // Update time each second using the standard application font, font
      // management has to be refactored.
      QFont standardFontSetting("Ubuntu", 24);
      ui->label->setFont(standardFontSetting);
      std::string lLabelString =
          "De gripper laat los in: " +
          std::to_string(timeout - lTimer.elapsed() / 1000) + " seconden";
      ui->label->setText(lLabelString.c_str());

      // Required in blocking functions or while loops. This allows for the
      // redrawing of the GUI dynamically.
      qApp->processEvents();
    }

    // Reset mMsgSent variable to reset button state and allow consecutive
    // executions.
    mGoalPublisher.mMsgSent = false;

    // Reset label to standard text, this is a temporary solution and needs to
    // be refactored.
    QFont standardFontSetting("Ubuntu", 24);
    ui->label->setFont(standardFontSetting);
    ui->label->setText(
        "Maak een keuze doormiddel van de knoppen hieronder. Dit object zal "
        "voor u worden opgepakt.");

    // Change button components back to standard.
    QPixmap lPix(":/new/icons/Glas_water.png");
    QIcon lIcon(lPix);
    ui->water_btn->setIcon(lIcon);
    ui->water_btn->setIconSize(QSize(130, 130));
    ui->water_btn->setText(QString("Beker water"));
  }
}
