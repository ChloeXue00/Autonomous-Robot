/*
 * Copyright (C) 2024 OpenDLV
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cluon-complete.hpp"
#include "opendlv-message-standard.hpp"
int32_t detectcones()
int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto cmd = cluon::getCommandlineArguments(argc, argv);
  if ((0 == cmd.count("cid")) || (0 == cmd.count("name")) ||
      (0 == cmd.count("width")) || (0 == cmd.count("height")))
  {
    std::cout << argv[0]
              << " attaches to a shared memory area containing an ARGB image."
              << std::endl;
    std::cout << "Usage:   " << argv[0] << " "
              << "--cid=<OD4 session> --name=<name of shared memory area> "
                 "--width=<width of the video> --height=<height of the video> "
                 "[--verbose]"
              << std::endl;
    std::cout << "Example: " << argv[0] << " "
              << "--cid=112 --name=img.argb --width=640 --height=480 --verbose"
              << std::endl;
  }
  else
  {
    std::string const name{cmd["name"]};
    uint32_t const width{static_cast<uint32_t>(std::stoi(cmd["width"]))};
    uint32_t const height{static_cast<uint32_t>(std::stoi(cmd["height"]))};

    // uint32_t const hsvLowBlue_H{static_cast<uint32_t>(std::stoi(cmd["hsvLowBlue_H"]))};
    // uint32_t const hsvLowBlue_S{static_cast<uint32_t>(std::stoi(cmd["hsvLowBlue_S"]))};
    // uint32_t const hsvLowBlue_V{static_cast<uint32_t>(std::stoi(cmd["hsvLowBlue_V"]))};

    // uint32_t const hsvHighBlue_H{static_cast<uint32_t>(std::stoi(cmd["hsvHighBlue_H"]))};
    // uint32_t const hsvHighBlue_S{static_cast<uint32_t>(std::stoi(cmd["hsvHighBlue_S"]))};
    // uint32_t const hsvHighBlue_V{static_cast<uint32_t>(std::stoi(cmd["hsvHighBlue_V"]))};

    uint32_t const heightFactor{static_cast<uint32_t>(std::stoi(cmd["heightFactor"]))};

    cv::Scalar lower_yellow = cv::Scalar(20, 100, 100); // HSV yellow lower bound
    cv::Scalar upper_yellow = cv::Scalar(30, 255, 255); // HSV yellow upper bound
    cv::Scalar lower_blue = cv::Scalar(100, 150, 0);    // HSV blue lower bound
    cv::Scalar upper_blue = cv::Scalar(140, 255, 255);  // HSV blue upper bound

    bool const verbose{cmd.count("verbose") != 0};

    // Attach to the shared memory.
    std::unique_ptr<cluon::SharedMemory> sharedMemory{
        new cluon::SharedMemory{name}};
    if (sharedMemory && sharedMemory->valid())
    {
      std::clog << argv[0] << ": Attached to shared memory '"
                << sharedMemory->name() << " (" << sharedMemory->size()
                << " bytes)." << std::endl;

      // Interface to a running OD4 session; here, you can send and
      // receive messages.
      cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(cmd["cid"]))};

      // Endless loop; end the program by pressing Ctrl-C.
      while (od4.isRunning())
      {
        cv::Mat img;

        // Wait for a notification of a new frame.
        sharedMemory->wait();

        // Lock the shared memory.
        sharedMemory->lock();
        {
          // Copy image into cvMat structure.
          // Be aware of that any code between lock/unlock is blocking
          // the camera to provide the next frame. Thus, any
          // computationally heavy algorithms should be placed outside
          // lock/unlock
          cv::Mat wrapped(height, width, CV_8UC3, sharedMemory->data());
          wrapped = wrapped(cv::Range(height * heightFactor / 10, height), (cv::Range(0, width)));
          img = wrapped.clone();
        }
        sharedMemory->unlock();
      
        // OpenCV remove middle 300 px * 90 px
        cv::rectangle(img, cv::Point(width / 2 - 150, height * (10 - heightFactor) / 10 - 90), cv::Point(width / 2 + 130, height * (10 - heightFactor) / 10), cv::Scalar(0, 0, 0), -1);
        

        
        
        // cv::Mat hsv;
        // cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);


        cv::Scalar lower_yellow = cv::Scalar(20, 100, 100); // HSV yellow lower bound
        cv::Scalar upper_yellow = cv::Scalar(30, 255, 255); // HSV yellow upper bound
        cv::Scalar lower_blue = cv::Scalar(100, 150, 0);    // HSV blue lower bound
        cv::Scalar upper_blue = cv::Scalar(140, 255, 255);  // HSV blue upper bound
        cv::dilate(img, img, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
        
        int detectCones(cv::Mat& img) {
            cv::Mat hsv;
            cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

            cv::Mat mask_yellow, mask_blue;
            cv::inRange(hsv, lower_yellow, upper_yellow, mask_yellow);
            cv::inRange(hsv, lower_blue, upper_blue, mask_blue);
            cv::dilate(mask_blue, mask_blue, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
            cv::dilate(mask_yellow, mask_yellow, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
            cv::erode(mask_yellow, mask_yellow, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);
            cv::erode(mask_blue, mask_blue, cv::Mat(), cv::Point(-1, -1), 5, 1, 1);

            std::vector<std::vector<cv::Point>> contours_yellow, contours_blue;
            cv::findContours(mask_yellow, contours_yellow, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            cv::findContours(mask_blue, contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& contour : contours_yellow) {
                cv::Rect boundingRect = cv::boundingRect(contour);
                cv::rectangle(img, boundingRect, cv::Scalar(0, 255, 255), 2);
                int centerX_yellow = boundingRect.x + boundingRect.width / 2;
                int centerY_yellow = boundingRect.y + boundingRect.height / 2;
                int numberOfYellowCones = contours_yellow.size();
            }

            for (const auto& contour : contours_blue) {
                cv::Rect boundingRect = cv::boundingRect(contour);
                cv::rectangle(img, boundingRect, cv::Scalar(255, 0, 0), 2);
                int centerX_blue = boundingRect.x + boundingRect.width / 2;
                int centerY_blue = boundingRect.y + boundingRect.height / 2;
                int numberOfBlueCones = contours_blue.size();
            }
            cv::imshow("cones",img);
            waitkey(0);
            std::cout << "Number of yellow cones: " << numberOfYellowCones << std::endl;
            std::cout << "Number of blue cones: " << numberOfBlueCones << std::endl;
        }
         
        
        // Display image.
        if (verbose)
        {
          // print coordinates of the center of the cones
          std::cout << "X_yellow: " << centerX_blue << " Y_yellow: " << centerY_yellow << std::endl;
          std::cout << "X_blue: " << centerX_yellow << " Y_blue: " << centerY_blue << std::endl;
          cv::imshow(sharedMemory->name().c_str(), img);
          cv::waitKey(1);
        }

        opendlv::logic::perception::DetectionProperty colorBlue;
        opendlv::logic::perception::DetectionProperty colorYellow;
        colorBlue.sampleId(0);
        colorYellow.sampleId(1);

        // // encode x and y as one string
        // std::string xCoordStr = std::to_string(xCoord);
        // std::string yCoordStr = std::to_string(yCoord);
        // std::string xyCoordStr = xCoordStr + "," + yCoordStr;
        // colorBlue.property(xyCoordStr);

        

        // if paper is detected, send the coordinates, if it's big enough
        if (maxArea > 100.0)
        {
          od4.send(colorBlue);
        }
      }
    }
    retCode = 0;
  }
  return retCode;
}
