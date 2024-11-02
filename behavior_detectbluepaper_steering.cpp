/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "behavior.hpp"
#include <chrono>
#include <iostream>

Behavior::Behavior(float steeringFactor) noexcept : m_frontUltrasonicReading{},
                                                    m_rearUltrasonicReading{},
                                                    m_leftIrReading{},
                                                    m_rightIrReading{},
                                                    m_groundSteeringAngleRequest{},
                                                    m_pedalPositionRequest{},
                                                    m_xCoord{},
                                                    m_yCoord{},
                                                    m_lastDetectionPropertyTime{},
                                                    m_steeringFactor{steeringFactor},
                                                    m_frontUltrasonicReadingMutex{},
                                                    m_rearUltrasonicReadingMutex{},
                                                    m_leftIrReadingMutex{},
                                                    m_rightIrReadingMutex{},
                                                    m_groundSteeringAngleRequestMutex{},
                                                    m_pedalPositionRequestMutex{}
{
}

opendlv::proxy::GroundSteeringRequest Behavior::getGroundSteeringAngle() noexcept
{
  std::lock_guard<std::mutex> lock(m_groundSteeringAngleRequestMutex);
  return m_groundSteeringAngleRequest;
}

opendlv::proxy::PedalPositionRequest Behavior::getPedalPositionRequest() noexcept
{
  std::lock_guard<std::mutex> lock(m_pedalPositionRequestMutex);
  return m_pedalPositionRequest;
}

void Behavior::setFrontUltrasonic(opendlv::proxy::DistanceReading const &frontUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_frontUltrasonicReadingMutex);
  m_frontUltrasonicReading = frontUltrasonicReading;
}

void Behavior::setRearUltrasonic(opendlv::proxy::DistanceReading const &rearUltrasonicReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rearUltrasonicReadingMutex);
  m_rearUltrasonicReading = rearUltrasonicReading;
}

void Behavior::setLeftIr(opendlv::proxy::DistanceReading const &leftIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_leftIrReadingMutex);
  m_leftIrReading = leftIrReading;
}

void Behavior::setRightIr(opendlv::proxy::DistanceReading const &rightIrReading) noexcept
{
  std::lock_guard<std::mutex> lock(m_rightIrReadingMutex);
  m_rightIrReading = rightIrReading;
}

void Behavior::setDetectionProperty(opendlv::logic::perception::DetectionProperty const &detectionProperty) noexcept
{
  // the DetectionProperty.property is a strong of: xCoordStr + "," + yCoordStr; decode it and store the values
  std::string property = detectionProperty.property();
  // std::cout << "property: " << property << std::endl;
  std::string xCoordStr = property.substr(0, property.find(","));
  std::string yCoordStr = property.substr(property.find(",") + 1);
  m_xCoord = std::stoi(xCoordStr);
  m_yCoord = std::stoi(yCoordStr);
  // print the values
  m_lastDetectionPropertyTime = std::chrono::steady_clock::now();
}

void Behavior::step() noexcept
{
  opendlv::proxy::DistanceReading frontUltrasonicReading;
  opendlv::proxy::DistanceReading rearUltrasonicReading;
  opendlv::proxy::DistanceReading leftIrReading;
  opendlv::proxy::DistanceReading rightIrReading;
  {
    std::lock_guard<std::mutex> lock1(m_frontUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock2(m_rearUltrasonicReadingMutex);
    std::lock_guard<std::mutex> lock3(m_leftIrReadingMutex);
    std::lock_guard<std::mutex> lock4(m_rightIrReadingMutex);

    frontUltrasonicReading = m_frontUltrasonicReading;
    rearUltrasonicReading = m_rearUltrasonicReading;
    leftIrReading = m_leftIrReading;
    rightIrReading = m_rightIrReading;
  }

  float frontDistance = frontUltrasonicReading.distance();
  float rearDistance = rearUltrasonicReading.distance();
  double leftDistance = leftIrReading.distance();
  double rightDistance = rightIrReading.distance();

  auto now = std::chrono::steady_clock::now();
  auto timeSinceLastDetectionProperty = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastDetectionPropertyTime).count();

  float pedalPosition = (m_yCoord + 240) * 0.02f;
  if (pedalPosition > 0.7f)
  {
    pedalPosition = 0.7f;
  }

  float groundSteeringAngle = 0.3f;

  groundSteeringAngle = m_xCoord * m_steeringFactor;

  // print m_xCoord and m_yCoord
  // std::cout << "xCoord: " << m_xCoord << " yCoord: " << m_yCoord << std::endl;

  // if any distance is less than 0.1m, stop the vehicle
  if (frontDistance < 0.05f || rearDistance > 0.1f || leftDistance < 0.1f || rightDistance > 0.1f)
  {
    // print all the distances on one line
    std::cout << "frontDistance: " << frontDistance << " rearDistance: " << rearDistance << " leftDistance: " << leftDistance << " rightDistance: " << rightDistance << std::endl;
  }

  if (timeSinceLastDetectionProperty > 110)
  {
    if (timeSinceLastDetectionProperty < 1000)
    {
      pedalPosition = -0.7f;
      // wiggle wheels by seeting steering to 0.26 and -0.26
      if (timeSinceLastDetectionProperty % 200 < 100)
      {
        groundSteeringAngle = 0.26f;
      }
      else
      {
        groundSteeringAngle = -0.26f;
      }
    }
    else
    {
      pedalPosition = 0.0f;
      groundSteeringAngle = 0.0f;
    }
  }

  {
    std::lock_guard<std::mutex> lock1(m_groundSteeringAngleRequestMutex);
    std::lock_guard<std::mutex> lock2(m_pedalPositionRequestMutex);

    opendlv::proxy::GroundSteeringRequest groundSteeringAngleRequest;
    groundSteeringAngleRequest.groundSteering(groundSteeringAngle);
    m_groundSteeringAngleRequest = groundSteeringAngleRequest;

    opendlv::proxy::PedalPositionRequest pedalPositionRequest;
    pedalPositionRequest.position(pedalPosition);
    m_pedalPositionRequest = pedalPositionRequest;
  }
}
