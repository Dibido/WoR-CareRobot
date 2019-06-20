#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include "environment_controller/Position.hpp"
#include "userinterface/CupSubscriber.hpp"
#include "userinterface/GoalPublisher.hpp"
#include "userinterface/ProgressScreen.hpp"
#include "userinterface/ReleaseTimePublisher.hpp"
#include <QApplication>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QToolButton>

namespace Ui
{
  class MainWindow;

  namespace ui_constants
  {
    // Water button icon size
    const uint8_t c_WaterBtnSize_m = 130;

    // Cross button icon size
    const uint8_t c_CrossBtnSize_m = 130;

    const uint8_t cStaticGlassBtnSize_pixels = 130;

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
   * @brief This method is called when the water button is clicked.
   */
  void on_water_btn_clicked();
  /**
   * @brief This method is called when the static water button is clicked.
   */
  void on_static_glass_btn_clicked();
  /**
   * @brief This method is called when the return water button is clicked.
   */
  void on_return_btn_clicked();

    private:
  ProgressScreen progressWindow;
  /**
   * @brief The UI object for QT. See: https://doc.qt.io/qt-5/qmainwindow.html.
   */
  Ui::MainWindow* ui;

  /**
   * @brief GoalPublisher used in this class.
   */
  userinterface::GoalPublisher mGoalPublisher;

  /**
   * @brief The dropLocation GoalPublisher used in this class.
   */
  userinterface::GoalPublisher mDropLocationPublisher;

  /**
   * @brief ReleaseTimePublisher used in this class.
   */
  userinterface::ReleaseTimePublisher mReleaseTimePublisher;

  /**
   * @brief Initiates the sequence of picking up the cup.
   *
   * @param staticCup Whether to detect a static cup or a moving cup from the
   * AGV.
   */
  void onWaterBtnClicked(bool staticCup);

  /**
   * @brief Checks whether the progress screen is hidden, and if so, un-hides
   * the main window.
   */
  void returnToMain();

  /**
   * @brief Initiates the returning of the cup sequence.
   */
  void sendReturnLocation();
};

#endif // MAINWINDOW_HPP
