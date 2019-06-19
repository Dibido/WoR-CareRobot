#ifndef PROGRESSSCREEN_HPP
#define PROGRESSSCREEN_HPP

#include "CupSubscriber.hpp"
#include "ros/ros.h"
#include <QTimer>
#include <QWidget>

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

    private:
  Ui::ProgressScreen* ui;
  const std::string mCupTopic = "cup";
  userinterface::CupSubscriber mCupSubscriber;
  bool active = 0;
  bool getActive();
  void updateProgress();
};

//} // namespace Ui

#endif // PROGRESSSCREEN_HPP
