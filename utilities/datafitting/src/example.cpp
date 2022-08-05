// HSDF, Philipp Berthold, philipp.berthold@unibw.de
#include <iostream>
#include <DataFitting/Service.h>

enum Pose2Index
{
    x,
    y,
    yaw
};

enum ScalarIndex
{
    value
};

enum DynamicsIndex
{
    speed,
    yawrate
};

typedef DataFitting::Type<Pose2Index,3> Pose2Type;
typedef DataFitting::Type<ScalarIndex,1> SType;
typedef DataFitting::Type<DynamicsIndex,2> DynamicsType;

DataFitting::Service<Pose2Type> ego_pose_service_;
DataFitting::Service<DynamicsType> target_dynamics_service_;

std::string computationStatusToString(DataFitting::ComputationStatus status)
{
    switch (status)
    {
        case DataFitting::ComputationStatus::none:
            return "none";
            break;

        case DataFitting::ComputationStatus::approximated:
            return "approximated";
            break;

        case DataFitting::ComputationStatus::interpolation:
            return "interpolation";
            break;

        case DataFitting::ComputationStatus::extrapolation:
            return "extrapolation";
            break;

        default:
            return "invalid";
            break;
    }
}

int main(int argc, char **argv)
{
    // Configure buffer sizes (only "trace to past" - buffers use ring buffers internally and reuse outdated sample slots)
    ego_pose_service_.setConfiguration(10);
    target_dynamics_service_.setConfiguration(2000);

    // Fill buffers (data, time)
    ego_pose_service_.registerValue(Pose2Type({0,0,0}), 0);
    ego_pose_service_.registerValue(Pose2Type({0.5,0,0}), 1);
    ego_pose_service_.registerValue(Pose2Type({1.5,0,0}), 2);
    ego_pose_service_.registerValue(Pose2Type({2,0,0}), 2.5);

    target_dynamics_service_.registerValue(DynamicsType({0,0}), 0);
    target_dynamics_service_.registerValue(DynamicsType({1,0}), 1);
    target_dynamics_service_.registerValue(DynamicsType({2,0}), 2);
    target_dynamics_service_.registerValue(DynamicsType({2,0.1}), 3);
    target_dynamics_service_.registerValue(DynamicsType({2,0.2}), 4);

    // Access data at arbitrary time
    DataFitting::ComputationStatus computation_status;
    DynamicsType target_dynamics;
    Pose2Type ego_pose;

    // 1) t = 0.5
    target_dynamics = target_dynamics_service_.getValueAtTime(0.5, &computation_status);
    std::cout << "1) " << target_dynamics.get(DynamicsIndex::speed) << " " << target_dynamics[DynamicsIndex::yawrate] << " (" << computationStatusToString(computation_status) << ")" << std::endl;

    // 2) t = 0.75
    target_dynamics = target_dynamics_service_.getValueAtTime(0.75, &computation_status);
    std::cout << "2) " << target_dynamics.get(DynamicsIndex::speed) << " " << target_dynamics[DynamicsIndex::yawrate] << " (" << computationStatusToString(computation_status) << ")" << std::endl;

    // 3) t = 2.5, no extrapolation
    ego_pose = ego_pose_service_.getValueAtTime(3.5, &computation_status, DataFitting::ExtrapolationMode::none);
    std::cout << "3) " << ego_pose.get(Pose2Index::x) << " " << ego_pose.get(Pose2Index::y) << " " << ego_pose[Pose2Index::yaw] << " (" << computationStatusToString(computation_status) << ")" << std::endl;

    // 4) t = 2.5, linear extrapolation
    ego_pose = ego_pose_service_.getValueAtTime(3.5, &computation_status, DataFitting::ExtrapolationMode::linear);
    std::cout << "4) " << ego_pose.get(Pose2Index::x) << " " << ego_pose[Pose2Index::y] << " " << ego_pose[Pose2Index::yaw] << " (" << computationStatusToString(computation_status) << ")" << std::endl;

    // Clear data
    target_dynamics_service_.flush();

    // Access data again
    // 5) t = 0.5 - no data
    target_dynamics = target_dynamics_service_.getValueAtTime(0.5, &computation_status, DataFitting::ExtrapolationMode::none);
    std::cout << "5) " << target_dynamics.get(DynamicsIndex::speed) << " " << target_dynamics[DynamicsIndex::yawrate] << " (" << computationStatusToString(computation_status) << ")" << std::endl;

    // Add one single sample
    target_dynamics_service_.registerValue(DynamicsType({2,0}), 2);

    // 6) t = 0.5 - one sample only
    target_dynamics = target_dynamics_service_.getValueAtTime(0.5, &computation_status, DataFitting::ExtrapolationMode::none);
    std::cout << "6) " << target_dynamics.get(DynamicsIndex::speed) << " " << target_dynamics[DynamicsIndex::yawrate] << " (" << computationStatusToString(computation_status) << ")" << std::endl;

    return 0;
}
