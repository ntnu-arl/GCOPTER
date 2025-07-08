#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string pathTopic;
    std::string targetTopic;
    std::string odomTopic;
    std::string outTrajTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MapTopic", mapTopic);
        nh_priv.getParam("PathTopic", pathTopic);
        nh_priv.getParam("TargetTopic", targetTopic);
        nh_priv.getParam("outTrajTopic", outTrajTopic);
        nh_priv.getParam("odomTopic", odomTopic);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("VoxelWidth", voxelWidth);
        nh_priv.getParam("MapBound", mapBound);
        nh_priv.getParam("TimeoutRRT", timeoutRRT);
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
    }
};

class TrajSmoother
{
private:
    Config config;

    ros::NodeHandle nh;
    ros::Subscriber pathSub;
    ros::Subscriber odomSub;

    ros::Publisher multidof_traj_pub;

    Visualizer visualizer;

    Trajectory<5> traj;
    double trajStamp;

    nav_msgs::Odometry odomMsg;

public:
    TrajSmoother(const Config &conf,
                  ros::NodeHandle &nh_)
        : config(conf),
          nh(nh_),
          visualizer(nh)
    {
        const Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                                  (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                                  (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        const Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        pathSub = nh.subscribe(config.pathTopic, 1, &TrajSmoother::pathCallback, this,
            ros::TransportHints().tcpNoDelay());
        odomSub = nh.subscribe(config.odomTopic, 1, &TrajSmoother::odomCallback, this,
            ros::TransportHints().tcpNoDelay());
        
        multidof_traj_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
            config.outTrajTopic, 1);
    }

    inline void odomCallback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        odomMsg = *odom;
    }

    inline void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if(msg->poses.size() < 2)
        {
            /* TODO: Handle this case */
            ROS_WARN("Too short path.");
            return;
        }

        std::vector<Eigen::Vector3d> route;
        // convert msg to route
        std::cout << "Received path with " << msg->poses.size() << " poses." << std::endl;
        route.push_back(Eigen::Vector3d(
            odomMsg.pose.pose.position.x,
            odomMsg.pose.pose.position.y,
            odomMsg.pose.pose.position.z));
        
            for (const auto &pose : msg->poses)
        {
            std::cout << pose.pose.position.x << ", "
                      << pose.pose.position.y << ", "
                      << pose.pose.position.z << std::endl;
            Eigen::Vector3d point(pose.pose.position.x,
                                 pose.pose.position.y,
                                 pose.pose.position.z);
            if (std::isnan(point(0)) || std::isnan(point(1)) || std::isnan(point(2)))
            {
                if((point - route.back()).norm() < 1e-3)
                {
                    continue; // Skip consecutive NaN points
                }
                else
                {
                    ROS_WARN("Encountered NaN point in path, skipping.");
                    continue;
                }
            }
            route.push_back(point);
        }

        if(route.size() < 2)
        {
            ROS_WARN("Path contains no valid points after filtering.");
            return;
        }

        std::vector<Eigen::MatrixX4d> hPolys;
        // Generate box shaped polyhedrons along each segment of the route
        const int n = route.size();
        const double range = config.dilateRadius; // Example range for the box size

        for (int i = 1; i < n; ++i)
        {
            Eigen::Vector3d a = route[i - 1];
            Eigen::Vector3d b = route[i];

            // Compute the direction vector of the segment [a, b]
            Eigen::Vector3d direction = (b - a).normalized();

            // Create a rotation matrix to align the bounding box with the path segment
            Eigen::Matrix3d rotation;
            rotation.col(0) = direction; // Align x-axis with the segment direction
            rotation.col(1) = Eigen::Vector3d(-direction(1), direction(0), 0).normalized(); // Perpendicular in xy-plane
            rotation.col(2) = direction.cross(rotation.col(1)).normalized(); // Perpendicular to both

            // Define the bounding planes in the local coordinate system
            Eigen::Matrix<double, 6, 4> local_bd;
            // local_bd << 1, 0, 0, -range,  // x+
            //             -1, 0, 0, -range, // x-
            //             0, 1, 0, -range,  // y+
            //             0, -1, 0, -range, // y-
            //             0, 0, 1, -range,  // z+
            //             0, 0, -1, -range; // z-
            double segment_length = (b - a).norm(); // Compute the length of the segment
            local_bd << 1, 0, 0, -(segment_length / 2.0 + range),  // x+
                        -1, 0, 0, -(segment_length / 2.0 + range), // x-
                        0, 1, 0, -range,                          // y+
                        0, -1, 0, -range,                         // y-
                        0, 0, 1, -range,                          // z+
                        0, 0, -1, -range;                         // z-

            // Transform the bounding planes to the global coordinate system
            Eigen::Matrix<double, 6, 4> global_bd = local_bd;
            // for (int j = 0; j < 6; ++j)
            // {
            //     Eigen::Vector3d normal = local_bd.row(j).head<3>(); // Extract the normal vector
            //     global_bd.row(j).head<3>() = rotation * normal;     // Transform the normal vector
            //     global_bd(j, 3) += -(global_bd.row(j).head<3>().dot(a)); // Update the offset
            // }
            Eigen::Vector3d center = (a + b) / 2.0; // Compute the center of the segment
            for (int j = 0; j < 6; ++j)
            {
                Eigen::Vector3d normal = local_bd.row(j).head<3>(); // Extract the normal vector
                global_bd.row(j).head<3>() = rotation * normal;     // Transform the normal vector
                global_bd(j, 3) += -(global_bd.row(j).head<3>().dot(center)); // Update the offset to center the box
            }

            hPolys.emplace_back(global_bd);
        }

        // Visualize the bounding boxes
        visualizer.visualizePolytope(hPolys);

        Eigen::Matrix3d iniState;
        Eigen::Matrix3d finState;
        Eigen::Vector3d init_vel = Eigen::Vector3d(odomMsg.twist.twist.linear.x,
                                                odomMsg.twist.twist.linear.y,
                                                odomMsg.twist.twist.linear.z);
        iniState << route.front(), init_vel, Eigen::Vector3d::Zero();
        finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

        gcopter::GCOPTER_PolytopeSFC gcopter;

        // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
        // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
        // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
        //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
        // initialize some constraint parameters
        Eigen::VectorXd magnitudeBounds(5);
        Eigen::VectorXd penaltyWeights(5);
        Eigen::VectorXd physicalParams(6);
        magnitudeBounds(0) = config.maxVelMag;
        magnitudeBounds(1) = config.maxBdrMag;
        magnitudeBounds(2) = config.maxTiltAngle;
        magnitudeBounds(3) = config.minThrust;
        magnitudeBounds(4) = config.maxThrust;
        penaltyWeights(0) = (config.chiVec)[0];
        penaltyWeights(1) = (config.chiVec)[1];
        penaltyWeights(2) = (config.chiVec)[2];
        penaltyWeights(3) = (config.chiVec)[3];
        penaltyWeights(4) = (config.chiVec)[4];
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;
        const int quadratureRes = config.integralIntervs;

        traj.clear();

        if (!gcopter.setup(config.weightT,
                            iniState, finState,
                            hPolys, INFINITY,
                            config.smoothingEps,
                            quadratureRes,
                            magnitudeBounds,
                            penaltyWeights,
                            physicalParams))
        {
            return;
        }

        if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
        {
            return;
        }

        if (traj.getPieceNum() > 0)
        {
            trajStamp = ros::Time::now().toSec();
            visualizer.visualize(traj, route);

            // publish the trajectory as a MultiDOFJointTrajectory
            trajectory_msgs::MultiDOFJointTrajectory trajMsg;
            trajMsg.header.stamp = ros::Time::now();
            trajMsg.header.frame_id = "odom";
            // trajMsg.joint_names.push_back("uav_joint");
            // for (const auto &piece : traj)
            // {
            //     trajectory_msgs::MultiDOFJointTrajectoryPoint point;
            //     point.time_from_start = ros::Duration(piece.getDuration());
            //     Eigen::Vector3d pos = piece.getPos(0.0);
            //     Eigen::Quaterniond quat = Eigen::Quaterniond::Identity(); // Assuming no rotation
            //     point.transforms.emplace_back();
            //     point.transforms.back().translation.x = pos(0);
            //     point.transforms.back().translation.y = pos(1);
            //     point.transforms.back().translation.z = pos(2);
            //     point.transforms.back().rotation.x = quat.x();
            //     point.transforms.back().rotation.y = quat.y();
            //     point.transforms.back().rotation.z = quat.z();
            //     point.transforms.back().rotation.w = quat.w();
            //     trajMsg.points.push_back(point);
            // }
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = 0.0; t < traj.getTotalDuration(); t += T)
            {
                // geometry_msgs::Point point;
                // Eigen::Vector3d X = traj.getPos(t);
                // point.x = lastX(0);
                // point.y = lastX(1);
                // point.z = lastX(2);
                // trajMarker.points.push_back(point);
                // point.x = X(0);
                // point.y = X(1);
                // point.z = X(2);
                // trajMarker.points.push_back(point);
                // lastX = X;

                trajectory_msgs::MultiDOFJointTrajectoryPoint point;
                point.time_from_start = ros::Duration(t);
                Eigen::Vector3d pos = traj.getPos(t);
                Eigen::Quaterniond quat = Eigen::Quaterniond::Identity(); // Assuming no rotation
                point.transforms.emplace_back();
                point.transforms.back().translation.x = pos(0);
                point.transforms.back().translation.y = pos(1);
                point.transforms.back().translation.z = pos(2);
                point.transforms.back().rotation.x = quat.x();
                point.transforms.back().rotation.y = quat.y();
                point.transforms.back().rotation.z = quat.z();
                point.transforms.back().rotation.w = quat.w();

                point.velocities.emplace_back();
                point.velocities.back().linear.x = traj.getVel(t)(0);
                point.velocities.back().linear.y = traj.getVel(t)(1);
                point.velocities.back().linear.z = traj.getVel(t)(2);

                trajMsg.points.push_back(point);
            }
            multidof_traj_pub.publish(trajMsg);
            ROS_INFO("Published trajectory with %zu points.", trajMsg.points.size());

        }
    }

//     inline void plan()
//     {
//         if (startGoal.size() == 2)
//         {
//             std::vector<Eigen::Vector3d> route;
//             sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0],
//                                                    startGoal[1],
//                                                    voxelMap.getOrigin(),
//                                                    voxelMap.getCorner(),
//                                                    &voxelMap, 0.01,
//                                                    route);
//             std::vector<Eigen::MatrixX4d> hPolys;
//             std::vector<Eigen::Vector3d> pc;
//             voxelMap.getSurf(pc);

//             sfc_gen::convexCover(route,
//                                  pc,
//                                  voxelMap.getOrigin(),
//                                  voxelMap.getCorner(),
//                                  7.0,
//                                  3.0,
//                                  hPolys);
//             sfc_gen::shortCut(hPolys);

//             if (route.size() > 1)
//             {
//                 visualizer.visualizePolytope(hPolys);

//                 Eigen::Matrix3d iniState;
//                 Eigen::Matrix3d finState;
//                 iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
//                 finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

//                 gcopter::GCOPTER_PolytopeSFC gcopter;

//                 // magnitudeBounds = [v_max, omg_max, theta_max, thrust_min, thrust_max]^T
//                 // penaltyWeights = [pos_weight, vel_weight, omg_weight, theta_weight, thrust_weight]^T
//                 // physicalParams = [vehicle_mass, gravitational_acceleration, horitonral_drag_coeff,
//                 //                   vertical_drag_coeff, parasitic_drag_coeff, speed_smooth_factor]^T
//                 // initialize some constraint parameters
//                 Eigen::VectorXd magnitudeBounds(5);
//                 Eigen::VectorXd penaltyWeights(5);
//                 Eigen::VectorXd physicalParams(6);
//                 magnitudeBounds(0) = config.maxVelMag;
//                 magnitudeBounds(1) = config.maxBdrMag;
//                 magnitudeBounds(2) = config.maxTiltAngle;
//                 magnitudeBounds(3) = config.minThrust;
//                 magnitudeBounds(4) = config.maxThrust;
//                 penaltyWeights(0) = (config.chiVec)[0];
//                 penaltyWeights(1) = (config.chiVec)[1];
//                 penaltyWeights(2) = (config.chiVec)[2];
//                 penaltyWeights(3) = (config.chiVec)[3];
//                 penaltyWeights(4) = (config.chiVec)[4];
//                 physicalParams(0) = config.vehicleMass;
//                 physicalParams(1) = config.gravAcc;
//                 physicalParams(2) = config.horizDrag;
//                 physicalParams(3) = config.vertDrag;
//                 physicalParams(4) = config.parasDrag;
//                 physicalParams(5) = config.speedEps;
//                 const int quadratureRes = config.integralIntervs;

//                 traj.clear();

//                 if (!gcopter.setup(config.weightT,
//                                    iniState, finState,
//                                    hPolys, INFINITY,
//                                    config.smoothingEps,
//                                    quadratureRes,
//                                    magnitudeBounds,
//                                    penaltyWeights,
//                                    physicalParams))
//                 {
//                     return;
//                 }

//                 if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
//                 {
//                     return;
//                 }

//                 if (traj.getPieceNum() > 0)
//                 {
//                     trajStamp = ros::Time::now().toSec();
//                     visualizer.visualize(traj, route);
//                 }
//             }
//         }
//     }

//     inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
//     {
//         if (mapInitialized)
//         {
//             if (startGoal.size() >= 2)
//             {
//                 startGoal.clear();
//             }
//             const double zGoal = config.mapBound[4] + config.dilateRadius +
//                                  fabs(msg->pose.orientation.z) *
//                                      (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
//             const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);
//             if (voxelMap.query(goal) == 0)
//             {
//                 visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
//                 startGoal.emplace_back(goal);
//             }
//             else
//             {
//                 ROS_WARN("Infeasible Position Selected !!!\n");
//             }

//             plan();
//         }
//         return;
//     }

//     inline void process()
//     {
//         Eigen::VectorXd physicalParams(6);
//         physicalParams(0) = config.vehicleMass;
//         physicalParams(1) = config.gravAcc;
//         physicalParams(2) = config.horizDrag;
//         physicalParams(3) = config.vertDrag;
//         physicalParams(4) = config.parasDrag;
//         physicalParams(5) = config.speedEps;

//         flatness::FlatnessMap flatmap;
//         flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2),
//                       physicalParams(3), physicalParams(4), physicalParams(5));

//         if (traj.getPieceNum() > 0)
//         {
//             const double delta = ros::Time::now().toSec() - trajStamp;
//             if (delta > 0.0 && delta < traj.getTotalDuration())
//             {
//                 double thr;
//                 Eigen::Vector4d quat;
//                 Eigen::Vector3d omg;

//                 flatmap.forward(traj.getVel(delta),
//                                 traj.getAcc(delta),
//                                 traj.getJer(delta),
//                                 0.0, 0.0,
//                                 thr, quat, omg);
//                 double speed = traj.getVel(delta).norm();
//                 double bodyratemag = omg.norm();
//                 double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));
//                 std_msgs::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
//                 speedMsg.data = speed;
//                 thrMsg.data = thr;
//                 tiltMsg.data = tiltangle;
//                 bdrMsg.data = bodyratemag;
//                 visualizer.speedPub.publish(speedMsg);
//                 visualizer.thrPub.publish(thrMsg);
//                 visualizer.tiltPub.publish(tiltMsg);
//                 visualizer.bdrPub.publish(bdrMsg);

//                 visualizer.visualizeSphere(traj.getPos(delta),
//                                            config.dilateRadius);
//             }
//         }
//     }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_smoother_node");
    ros::NodeHandle nh_;

    TrajSmoother traj_smoother(Config(ros::NodeHandle("~")), nh_);

    ros::spin();

    // ros::Rate lr(1000);
    // while (ros::ok())
    // {
    //     global_planner.process();
    //     ros::spinOnce();
    //     lr.sleep();
    // }

    return 0;
}
