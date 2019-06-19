#include "userinterface/ProgressScreen.hpp"
#include "ui_ProgressScreen.h"
#include "userinterface/GoalPublisher.hpp"

ProgressScreen::ProgressScreen(QWidget* parent)
    : QWidget(parent), ui(new Ui::ProgressScreen), gripperOpenCommandTime(0, 0)
{
  ui->setupUi(this);

  ui->ReleaseBtn->hide();

  QPixmap lPix(":/new/icons/Noodstop.png");
  QIcon lIcon(lPix);
  ui->EmergencyBtn->setIcon(lIcon);
  ui->EmergencyBtn->setIconSize(QSize(130, 130));
  ui->EmergencyBtn->setText(QString());
  ui->StatusLabel->setText(QString("Uw bestelling is onderweg"));

  QTimer* timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this,
          QOverload<>::of(&ProgressScreen::updateProgress));
  timer->start(100);
}

ProgressScreen::~ProgressScreen()
{
  delete ui;
}

void ProgressScreen::setActive(bool aValue)
{
  active = aValue;
  if (active)
  {
    mCupSubscriber.setEnabled(true);
    gripperOpenCommandTime = ros::Time::now();
  }
}

bool ProgressScreen::getActive()
{
  return active;
}

void ProgressScreen::updateProgress()
{
  if (mCupSubscriber.getArrivalTime() == 0)
  {
    ui->ProgressBar->setValue(0);
  }
  else
  {
    ui->ProgressBar->setValue(
        (ros::Time::now().toSec() - mCupSubscriber.getStartingTime()) /
        (mCupSubscriber.getArrivalTime() - mCupSubscriber.getStartingTime()) *
        100.0);

    std::cout << "Starting Time: "
              << std::to_string(mCupSubscriber.getStartingTime()) << std::endl;
    std::cout << "Current Time: " << std::to_string(ros::Time::now().toSec())
              << std::endl;
    std::cout << "Arrival Time: "
              << std::to_string(mCupSubscriber.getArrivalTime()) << std::endl;
  }

  if ((ros::Time::now() - gripperOpenCommandTime).toSec() < 0 ||
      (ros::Time::now() - gripperOpenCommandTime).toSec() >
          userinterface::goal_constants::cReleaseTime_s)
  {
    ui->StatusLabel->setText(QString("Uw bestelling is onderweg"));
  }
  else
  {
    double lSecondsLeft = userinterface::goal_constants::cReleaseTime_s +
                          (gripperOpenCommandTime - ros::Time::now()).toSec();
    ui->StatusLabel->setText(QString::fromStdString(
        std::string("De gripper laat los over ") +
        std::to_string(static_cast<int>(lSecondsLeft) + 1) +
        std::string(" seconden")));
  }
  if (ui->ProgressBar->value() == 100)
  {
    ui->ReleaseBtn->show();
    ui->StatusLabel->setText(QString(
        "Uw beker staat klaar, druk op \n de knop loslaten om verder te gaan"));
  }
}
