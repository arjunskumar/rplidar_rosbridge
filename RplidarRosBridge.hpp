#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"

#include "engine/core/assert.hpp"
#include "engine/core/constants.hpp"
#include "engine/core/logger.hpp" 

#include "messages/messages.hpp"


namespace isaac
{
    namespace rosbridge{
        class RplidarRosBridge: public alice::Codelet 
        {
            public:
            RplidarRosBridge();
            virtual ~RplidarRosBridge();

            void start() override;
            void tick() override;
            void stop() override;

            ISAAC_PROTO_TX(FlatscanProto, scan);

            // ROS subscriber queue depth
            ISAAC_PARAM(int, subscriber_queue_size, 1000);
            // ROS subscriber channel
            ISAAC_PARAM(std::string, subscriber_channel_name, "scan");

            ISAAC_POSE2(robot, laser);

            std::vector<Vector2f> position_;

            private:
            struct RosRplidarData;
            std::unique_ptr<RosRplidarData> scan_data_;

        };
    } // namespace rosbridge 
} // namespace isaac


ISAAC_ALICE_REGISTER_CODELET(isaac::rosbridge::RplidarRosBridge);
