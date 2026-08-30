#ifndef FRANKA_OPCUA_BRIDGE__FRANKA_ROBOT_HPP_
#define FRANKA_OPCUA_BRIDGE__FRANKA_ROBOT_HPP_

#include<memory>
#include<string>
#include<vector>

#include "franka_opcua_bridge/i_robot.hpp"
#include "franka_opcua_bridge/i_protocol_client.hpp"

namespace franka_opcua_bridge{

    class FrankaRobot : public IRobot {

        public:

            explicit FrankaRobot(std::unique_ptr<IProtocolClient> client,
                                 std::string endpoint,
                                 std::string user,
                                 std::string password);

            bool connect() override;
            void disconnect() override;

            bool requestControl() override;
            bool releaseControl() override;

            bool openBrakes() override;
            bool closeBrakes() override;
            bool areBrakesOpen();

            bool stop() override;


            bool executeNameTask(const std::string & task_id) override;
            bool moveToNamedPose(const std::string & pose_id) override;

            RobotStatus readStatus() override;
            std::vector<double> readJointAngles() override;
            geometry_msgs::msg::Pose readCartesianPose() override;

        protected:

            std::unique_ptr<IProtocolClient> client_;
        
        private:

            static const std::vector<std::string> kExecutionControlPath;
            std::string endpoint_;
            std::string user_;
            std::string password_;



    };



}

#endif