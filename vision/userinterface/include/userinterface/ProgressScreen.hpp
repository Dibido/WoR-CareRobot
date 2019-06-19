#ifndef PROGRESSSCREEN_HPP
#define PROGRESSSCREEN_HPP

#include "userinterface/CupSubscriber.hpp"
#include "userinterface/ReleaseTimePublisher.hpp"
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
  void setActive(bool aValue);

    private slots:
  void on_ReleaseBtn_clicked();

    private:
  Ui::ProgressScreen* ui;
  const std::string mCupTopic = "cup";
  userinterface::CupSubscriber mCupSubscriber;
  /**
   * @brief ReleaseTimePublisher used in this class.
   */
  userinterface::ReleaseTimePublisher mReleaseTimePublisher;
  bool active = 0;
  ros::Time gripperOpenCommandTime;
  bool getActive();
  void updateProgress();
};

//} // namespace Ui

#endif // PROGRESSSCREEN_HPP
