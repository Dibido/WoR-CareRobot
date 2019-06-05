#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "environment_controller/Position.hpp"
#include "userinterface/GoalPublisher.hpp"
#include "userinterface/ReleaseTimePublisher.hpp"
#include <QApplication>
#include <QElapsedTimer>
#include <QMainWindow>

namespace Ui
{
  class MainWindow;

  namespace ui_constants
  {
    // Water button icon size
    const uint8_t c_WaterBtnSize_m = 130;

    // Cross button icon size
    const uint8_t c_CrossBtnSize_m = 130;

  } // namespace ui_constants

} // namespace Ui

/**
 * @brief MainWindow class which features the functionality for the MainWindow
 * screen.
 *
 * This class features the handling of all elements on the MainWindow screen,
 * including button and label parameters.
 */

class MainWindow : public QMainWindow
{
  Q_OBJECT

    public:
  /**
   * @brief Default QT Constructor
   *
   * @param parent
   * Ctor calls setupUi() which creates the GUI screen(s).
   */
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

    private slots:

  /**
   * @brief This method is called when the water button is clicked. For now, it
   * launches Gazebo to show the demo.
   */
  void on_water_btn_clicked();

    private:
  /**
   * @brief The UI object for QT. See: https://doc.qt.io/qt-5/qmainwindow.html.
   */
  Ui::MainWindow* ui;

  /**
   * @brief GoalPublisher used in this class.
   */
  userinterface::GoalPublisher mGoalPublisher;

  /**
   * @brief ReleaseTimePublisher used in this class.
   */
  userinterface::ReleaseTimePublisher mReleaseTimePublisher;
};

#endif // MAINWINDOW_HPP
