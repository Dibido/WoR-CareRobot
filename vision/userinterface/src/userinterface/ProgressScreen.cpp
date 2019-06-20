#include "userinterface/ProgressScreen.hpp"
#include "ui_ProgressScreen.h"
#include "userinterface/GoalPublisher.hpp"

ProgressScreen::ProgressScreen(QWidget* parent)
    : QWidget(parent),
      ui(new Ui::ProgressScreen),
      gripperOpenCommandTime(INT_MAX - 1, 0)
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

void ProgressScreen::on_ReleaseBtn_clicked()
{
  mReleaseTimePublisher.selectReleaseTime(
      userinterface::goal_constants::cReleaseTime_s);

  gripperOpenCommandTime = ros::Time::now();
}

void ProgressScreen::setActive(bool aValue)
{
  active = aValue;
  if (active)
  {
    mCupSubscriber.setEnabled(true);
  }
}

bool ProgressScreen::getActive()
{
  return active;
}

void ProgressScreen::updateProgress()
{
  if (mCupSubscriber.getArrivalTime() < ros::Time::now())
  {
    ui->ProgressBar->setValue(0);
  }
  else
  {
    double lProgressPercentage =
        (ros::Time::now() - mCupSubscriber.getStartingTime()).toSec() /
        (mCupSubscriber.getArrivalTime() - mCupSubscriber.getStartingTime())
            .toSec() *
        100.0;
    ui->ProgressBar->setValue(static_cast<int>(lProgressPercentage));
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
    const double cSecondsLeftToHide = 0.15;
    if (lSecondsLeft < cSecondsLeftToHide)
    {
      // mCupSubscriber.resetAll();
      ui->ProgressBar->show();
      ui->TimeLabel->show();
      hide();
    }
  }

  // If the progress bar is full, show that the cup is ready.
  const int cProgressBarFullValue = 95;
  if (ui->ProgressBar->value() >= cProgressBarFullValue)
  {
    mCupSubscriber.resetArrivalTime();
    ui->ReleaseBtn->show();
    ui->StatusLabel->setText(QString(
        "Uw beker staat klaar, druk op \n de knop loslaten om verder te gaan"));
    ui->ProgressBar->hide();
    ui->TimeLabel->hide();
  }
}
