#include "CupScanner.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

namespace
{
  /** Global Variables */
  const bool USE_MAX_WEBCAM_RESOLUTION = true;
  cv::Mat img_original, img_result;
  CupScanner scanner;

  void applyScan()
  {
    scanner.scan(img_original, img_result);
  }

  void on_linear_transform_trackbar(int, void*)
  {
  }
} // namespace

int main(int argc, char** argv)
{
  std::string imageName("-0"); // by default
  int webcamId = -1;
  if (argc == 1 ||
      (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h"))
  {
    std::cout << "Usage: DetectCup -<webcam id>" << std::endl;
    return 0;
  }
  else
  {
    webcamId = std::atoi(argv[1] + 1);
  }

  cv::VideoCapture webcam(webcamId);
  if (!webcam.isOpened())
  {
    std::cerr << "Failed to open webcam with id " << webcamId << std::endl;
    return 1;
  }

  if (USE_MAX_WEBCAM_RESOLUTION)
  {
    webcam.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 720);
  }

  const std::string mainWinName = "DetectCup";
  cv::namedWindow(mainWinName, cv::WINDOW_AUTOSIZE);
  /* cv::createTrackbar("BarcodeType (0=QrCode, 1=DataMatrix, 2=Aruco)", */
  /*                    mainWinName, &barcode_type_id, 2, */
  /*                    on_linear_transform_trackbar); */

  while (true)
  {
    webcam.read(img_original);
    applyScan();
    cv::imshow(mainWinName, img_result);

    int c = cv::waitKey(30);
    // if escape is pressed
    if (c == 27)
      break;
  }

  return 0;
}
