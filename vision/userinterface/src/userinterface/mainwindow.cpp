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
  // If the goal msg hasn't been sent, do so to initiate the pickup sequence.
  if (mGoalPublisher.mMsgSent == false)
  {
    // Create and send the message
    mGoalPublisher.selectGoalPosition(userinterface::goal_constants::mDemoPos,
                                      false);

    // Mark msg as sent and change button to next step (see below)
    mGoalPublisher.mMsgSent = true;
    changeToReleaseButton(ui->water_btn);
    qApp->processEvents();
    return;
  }
  else
  {
    releaseGlass();
  }

  // Reset mMsgSent variable to reset button state and allow consecutive
  // executions.
  mGoalPublisher.mMsgSent = false;
  resetMainText();
  // Change button components back to standard.
  QPixmap lPix(":/new/icons/Glas_water.png");
  QIcon lIcon(lPix);
  ui->water_btn->setIcon(lIcon);
  ui->water_btn->setIconSize(QSize(Ui::ui_constants::c_WaterBtnSize_m,
                                   Ui::ui_constants::c_WaterBtnSize_m));
  ui->water_btn->setText(QString("Beker water"));
}

void MainWindow::on_static_glass_btn_clicked()
{
  if (mGoalPublisher.mMsgSent == false)
  {
    // Create and send the message
    mGoalPublisher.selectGoalPosition(userinterface::goal_constants::mDemoPos,
                                      true);

    // Mark msg as sent and change button to next step (see below)
    mGoalPublisher.mMsgSent = true;
    changeToReleaseButton(ui->static_glass_btn);
    qApp->processEvents();
    return;
  }
  else
  {
    releaseGlass();
    // Reset mMsgSent variable to reset button state and allow consecutive
    // executions.
    mGoalPublisher.mMsgSent = false;
    resetMainText();
    // Change button components back to standard.
    QPixmap lPix(":/new/icons/Glas_cola.png");
    QIcon lIcon(lPix);
    ui->static_glass_btn->setIcon(lIcon);
    ui->static_glass_btn->setIconSize(
        QSize(Ui::ui_constants::cStaticGlassBtnSize_pixels,
              Ui::ui_constants::cStaticGlassBtnSize_pixels));
    ui->static_glass_btn->setText(QString("Beker Statisch"));
  }
}

void MainWindow::on_return_btn_clicked()
{
  std::cout << "RETURN" << std::endl;
}

void MainWindow::changeToReleaseButton(QToolButton*& lButton)
{
  QPixmap lPix(":/new/icons/Cross.png");
  QIcon lIcon(lPix);
  lButton->setIcon(lIcon);
  lButton->setIconSize(QSize(Ui::ui_constants::c_CrossBtnSize_m,
                             Ui::ui_constants::c_CrossBtnSize_m));
  lButton->setText(QString("Laat beker los"));
  qApp->processEvents();
}

void MainWindow::releaseGlass()
{
  // Make QFont to store font settings used in this scope.
  QFont lStandardFontSetting("Ubuntu", 24);

  // Start an QElapsedTimer, to check the elapsed time.
  QElapsedTimer lTimer;
  lTimer.start();

  // Send the release time msg.
  mReleaseTimePublisher.selectReleaseTime(
      userinterface::goal_constants::cReleaseTime_s);

  // Countdown from timeout to zero in the GUI, so the user has visual
  // feedback.
  while (lTimer.elapsed() / 1000 <=
         userinterface::goal_constants::cReleaseTime_s - 1)
  {
    // Update time each second using the standardFontSetting defined above.
    ui->label->setFont(lStandardFontSetting);
    std::string lLabelString =
        "De gripper laat los in: " +
        std::to_string(userinterface::goal_constants::cReleaseTime_s -
                       lTimer.elapsed() / 1000) +
        " seconden";
    ui->label->setText(lLabelString.c_str());

    // Required in blocking functions or while loops. This allows for the
    // redrawing of the GUI dynamically.
    qApp->processEvents();
  }
}

void MainWindow::resetMainText()
{
  QFont lStandardFontSetting("Ubuntu", 24);

  ui->label->setFont(lStandardFontSetting);
  ui->label->setText(
      "Maak een keuze doormiddel van de knoppen hieronder. Dit object zal "
      "voor u worden opgepakt.");
}