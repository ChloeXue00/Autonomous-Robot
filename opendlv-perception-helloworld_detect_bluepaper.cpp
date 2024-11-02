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

    uint32_t const hsvLowBlue_H{static_cast<uint32_t>(std::stoi(cmd["hsvLowBlue_H"]))};
    uint32_t const hsvLowBlue_S{static_cast<uint32_t>(std::stoi(cmd["hsvLowBlue_S"]))};
    uint32_t const hsvLowBlue_V{static_cast<uint32_t>(std::stoi(cmd["hsvLowBlue_V"]))};

    uint32_t const hsvHighBlue_H{static_cast<uint32_t>(std::stoi(cmd["hsvHighBlue_H"]))};
    uint32_t const hsvHighBlue_S{static_cast<uint32_t>(std::stoi(cmd["hsvHighBlue_S"]))};
    uint32_t const hsvHighBlue_V{static_cast<uint32_t>(std::stoi(cmd["hsvHighBlue_V"]))};
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
          wrapped = wrapped(cv::Range(height / 2, height), (cv::Range(0, width)));
          img = wrapped.clone();
        }
        sharedMemory->unlock();

        // OpenCV remove middle 100 px * 50 px
        cv::rectangle(img, cv::Point(width / 2 - 200, height / 2 - 50), cv::Point(width / 2 + 200, height / 2), cv::Scalar(0, 0, 0), -1);

        cv::Mat hsv;
        cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

        // Detect and draw blue papers
        cv::Scalar hsvLowBlue(hsvLowBlue_H, hsvLowBlue_S, hsvLowBlue_V);
        cv::Scalar hsvHighBlue(hsvHighBlue_H, hsvHighBlue_S, hsvHighBlue_V);


        cv::Mat papers;
        cv::inRange(hsv, hsvLowBlue, hsvHighBlue, papers);

        if (verbose)
        {
          cv::imshow("Blue Papers", papers);
        }

        // cv::erode(papers, papers, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        cv::dilate(papers, papers, cv::Mat(), cv::Point(-1, -1), 20, 1, 1);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(papers, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // Find the contour with the largest area.
        std::vector<cv::Point> largestContour;
        double maxArea = 0.0;
        for (const auto &contour : contours)
        {
          double area = cv::contourArea(contour);
          if (area > maxArea)
          {
            maxArea = area;
            largestContour = contour;
          }
        }

        // Draw the bounding rectangle for the largest contour.
        cv::Rect boundingRect = cv::boundingRect(largestContour);
        cv::rectangle(img, boundingRect, cv::Scalar(0, 255, 0), 2);

        // Get X and Y coordinates of the center of the bounding box.
        int32_t xCoord = (boundingRect.x + boundingRect.width / 2) - width / 2;
        int32_t yCoord = (boundingRect.y + boundingRect.height / 2) - height / 2;

        // Display image.
        if (verbose)
        {
          // print coordinates of the center of the bounding box
          std::cout << "X: " << xCoord << " Y: " << yCoord << std::endl;

          cv::imshow(sharedMemory->name().c_str(), img);
          cv::waitKey(1);
        }

        opendlv::logic::perception::DetectionProperty colorBlue;

        colorBlue.sampleId(0);

        // encode x and y as one string
        std::string xCoordStr = std::to_string(xCoord);
        std::string yCoordStr = std::to_string(yCoord);
        std::string xyCoordStr = xCoordStr + "," + yCoordStr;
        colorBlue.property(xyCoordStr);

        // if paper is detected, send the coordinates
        if (maxArea > 0.0)
        {
          od4.send(colorBlue);
        }
      }
    }
    retCode = 0;
  }
  return retCode;
}
