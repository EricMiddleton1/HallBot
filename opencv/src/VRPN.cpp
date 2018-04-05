#include "VRPN.hpp"

#include <quat.h>

VRPN::TrackerData::TrackerData()
  : x{0.f}
  , y{0.f}
  , z{0.f}
  , pitch{0.f}
  , yaw{0.f}
  , roll{0.f} {
}

std::ostream& operator<<(std::ostream& os, const VRPN::TrackerData& td) {
  os << td.x << ", " << td.y << ", " << td.z << ", "
    << td.pitch << ", " << td.yaw << ", " << td.roll;

  return os;
}

VRPN::VRPN(std::vector<IConfigurable::Param>&& params)
  : IConfigurable{ {"name"}, std::move(params) }
  , m_vrpnHost{getParam("name")}
  , m_remote{m_vrpnHost.c_str()}
  , m_trackerData{}
  , m_runFlag{true}
  , m_vrpnThread{[this](){ vrpnLoop(); }} {
  
  std::cout << "[Info] VRPN: Using remote host \"" << m_vrpnHost << "\"" << std::endl;

  m_remote.register_change_handler(this, [](void* param, const vrpn_TRACKERCB t) {
    reinterpret_cast<VRPN*>(param)->callback(t);
    });
}

VRPN::~VRPN() {
  m_runFlag.store(false);
  m_vrpnThread.join();
}

VRPN::TrackerData VRPN::getUpdate() const {
  std::lock_guard<std::mutex> lock(m_mutex);

  return m_trackerData;
}

void VRPN::vrpnLoop() {
  while(m_runFlag.load()) {
    m_remote.mainloop();
  }
}

void VRPN::callback(const vrpn_TRACKERCB t) {
  q_vec_type euler;
  q_to_euler(euler, t.quat);
  
  std::lock_guard<std::mutex> lock(m_mutex);

  m_trackerData.x = t.pos[0];
  m_trackerData.y = t.pos[1];
  m_trackerData.z = t.pos[2];
  
  m_trackerData.pitch = euler[1];
  m_trackerData.yaw = euler[0];
  m_trackerData.roll = euler[2];
}
