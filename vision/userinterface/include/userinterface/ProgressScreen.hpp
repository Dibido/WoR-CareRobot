#ifndef PROGRESSSCREEN_HPP
#define PROGRESSSCREEN_HPP

#include "CupSubscriber.hpp"
#include <QTimer>
#include <QWidget>
#include <ros/ros.h>

namespace Ui
{
  class ProgressScreen;
}

class ProgressScreen : public QWidget
{
  Q_OBJECT

    public:
  explicit ProgressScreen(QWidget* parent = nullptr);
  ~ProgressScreen();
  void on_ReleaseBtn_clicked();
  void setActive(bool aValue);

    private:
  Ui::ProgressScreen* ui;
  const std::string mCupTopic = "cup";
  userinterface::CupSubscriber mCupSubscriber;
  bool active = 0;
  ros::Time gripperOpenCommandTime;
  bool getActive();
  void updateProgress();
};

//} // namespace Ui

#endif // PROGRESSSCREEN_HPP
