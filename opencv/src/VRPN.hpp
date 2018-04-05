#pragma once

#include "IConfigurable.hpp"

#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <string>
#include <iostream>

#include <vrpn_Tracker.h>


class VRPN : public IConfigurable {
public:
  struct TrackerData {
    TrackerData();

    friend std::ostream& operator<<(std::ostream& os, const TrackerData& td);

    float x, y, z;
    float pitch, yaw, roll;
  };

  VRPN(std::vector<IConfigurable::Param>&& params);
  ~VRPN();

  TrackerData getUpdate() const;

private:
  void vrpnLoop();
  void callback(const vrpn_TRACKERCB t);

  std::string m_vrpnHost;
  vrpn_Tracker_Remote m_remote;

  TrackerData m_trackerData;
  mutable std::mutex m_mutex;
  
  std::atomic<bool> m_runFlag;
  std::thread m_vrpnThread;
};
