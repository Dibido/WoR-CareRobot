#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>

namespace Ui
{
  class MainWindow;
}

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

  /**
   * @brief Destructor
   */
  ~MainWindow();

    private slots:

  /**
   * @brief This method is called when the water button is clicked. For now, it
   * launches Gazebo to show the demo.
   */
  void on_water_btn_clicked();

    private:
  Ui::MainWindow* ui;
};

#endif // MAINWINDOW_HPP
