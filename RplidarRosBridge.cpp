
#include "RplidarRosBridge.hpp"

#include "boost/next_prior.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/LaserScan.h"

#include "messages/math.hpp"

namespace isaac 
{ 
    namespace rosbridge 
    {
namespace {

constexpr int kNumberOfRays = 270 * 4 + 1;

class CallbackFunctor {
 public:
  explicit CallbackFunctor(RplidarRosBridge* bridge)
      : bridge_(bridge) {}
  CallbackFunctor(const CallbackFunctor &) = default;
  ~CallbackFunctor() = default;

  void operator() (const sensor_msgs::LaserScan::ConstPtr &msg) {
    if (!msg) {
      std::cout << "Empty LaserScan Message!" << std::endl;
      return;
    }
    auto _msg = bridge_->tx_scan().initProto();

    do {  // translate sensor_msgs::LaserScan to isaac FlatscanProto
#if 0
      _msg.setInvalidRangeThreshold(msg->range_min);
      _msg.setOutOfRangeThreshold(msg->range_max);
#else
      _msg.setInvalidRangeThreshold(0.5);
      _msg.setOutOfRangeThreshold(20.0);
#endif

      auto _size = msg->ranges.size();
      auto _ranges = _msg.initRanges(_size);
      auto _angles = _msg.initAngles(_size);
      for (auto i=0; i < kNumberOfRays; ++i) {
        if (msg->intensities[i] > 10) {  // intensities filter
          _ranges.set(i, msg->ranges[i]);
        } else
        {
          _ranges.set(i, 0);
        }
        _angles.set(i, (msg->angle_min + i*msg->angle_increment));

        auto _x = _ranges[i] * std::cos(_angles[i]);
        auto _y = _ranges[i] * std::sin(_angles[i]);

        bridge_->position_[i] = Vector2f(_x, _y);
      }
    } while (0);

    // publish scan data
    bridge_->tx_scan().publish(ros::Time(msg->header.stamp).toNSec());
  }

 private:
  RplidarRosBridge* bridge_;
};
}  // namespace

struct RplidarRosBridge::RosRplidarData {
  ros::NodeHandle node_;
  ros::Subscriber sub_;
  ros::CallbackQueue callbackQueue_;
};

RplidarRosBridge::RplidarRosBridge() {
  position_.resize(kNumberOfRays);
}

RplidarRosBridge::~RplidarRosBridge() {
  position_.clear();
}

void RplidarRosBridge::start() {
  ros::M_string _args;
  if (!ros::isInitialized()) {
    ros::init(_args, "isaacRosBridge", ros::init_options::NoSigintHandler);
  }

  scan_data_ = std::make_unique<RosRplidarData>();
  scan_data_->node_.setCallbackQueue(&(scan_data_->callbackQueue_));
  scan_data_->sub_ = scan_data_->node_.subscribe<sensor_msgs::LaserScan>(
      get_subscriber_channel_name(), get_subscriber_queue_size(),
      CallbackFunctor(this));

  tickPeriodically();
}

void RplidarRosBridge::tick() {
  if (ros::ok()) {
    scan_data_->callbackQueue_.callAvailable();
  }
}

void RplidarRosBridge::stop() {
  scan_data_->sub_.shutdown();
  scan_data_.reset();
}

} // namespace rosbridge
}  // namespace isaac