#include <algorithm>
#include <filesystem>
#include <iostream>
#include <map>
#include <numeric>
#include <optional>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/storage_options.hpp>

#include "common/ImuLoader.h"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/writer.hpp"

#include "LasLoader.h"
#include "ros2_utils.h"

void printHelp(char* entrypoint)
{
    std::cout << "Usage: " << entrypoint << " <directory> <output_bag>" << std::endl;
    std::cout << " Options are:" << std::endl;
    std::cout << "  --lines <number of lines> (default 8)" << std::endl;
    std::cout << "  --type <pointcloud2/livox> default is pointcloud2" << std::endl;
    std::cout << "  --points <number of points> (default 19968)" << std::endl;
}

void processImu(std::vector<std::string> files_imu, rosbag2_cpp::Writer* bag)
{
    for (const auto& imu_fn : files_imu)
    {
        auto data = mandeye::load_imu(imu_fn, 0);
        std::cout << "file '" << imu_fn << "'" << std::endl;
        for (const auto& [ts, ang, acc] : data)
        {
            if (ts == 0)
                continue;

            sensor_msgs::msg::Imu imu;

            imu.header.frame_id = "livox";
            imu.header.stamp = GetRosTimeSecond(ts);

            imu.angular_velocity.x = ang[0];
            imu.angular_velocity.y = ang[1];
            imu.angular_velocity.z = ang[2];

            imu.linear_acceleration.x = acc[0];
            imu.linear_acceleration.y = acc[1];
            imu.linear_acceleration.z = acc[2];

            bag->write(imu, "/livox/imu", imu.header.stamp);
        }
    }
}

void processPointcloud2()
{
}

void processLivox()
{
}

int main(int argc, char** argv)
{
    // parse command line arguments for directory to process
    if (argc < 3)
    {
        printHelp(argv[0]);
        return 1;
    }

    std::uint16_t number_of_lines = 40;
    std::uint16_t number_of_points = 20000;
    std::string messageType = "pointcloud2";
    std::string directory = argv[1];
    std::string output_bag = argv[2];

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg.size() < 2 || arg[0] != '-' || arg[1] != '-')
            continue;
        if (arg == "--lines")
        {
            number_of_lines = std::stoi(argv[i + 1]);
        }
        else if (arg == "--points")
        {
            number_of_points = std::stoi(argv[i + 1]);
        }
        else if (arg == "--type")
        {
            messageType = std::string(argv[i + 1]);
        }
        else
        {
            std::cout << "Unknown option: " << arg << std::endl;
            printHelp(argv[0]);
            return 1;
        }
    }

    std::cout << "Processing directory: " << directory << " creating bag " << output_bag << std::endl;
    std::cout << "Number of lines: " << number_of_lines << std::endl;
    std::cout << "Number of points: " << number_of_points << std::endl;

    // get list of files in directory
    std::vector<std::string> files_imu;
    std::vector<std::string> files_laz;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
    {
        auto extension = entry.path().extension();
        if (extension == ".csv")
        {
            files_imu.push_back(entry.path());
        }
        else if (extension == ".laz")
        {
            auto fn = entry.path().filename().string();
            if (fn.find("lidar") != std::string::npos)
                files_laz.push_back(entry.path());
        }
    }

    std::sort(files_imu.begin(), files_imu.end());
    std::sort(files_laz.begin(), files_laz.end());

    rosbag2_cpp::Writer bag;
    bag.open(output_bag);

    std::thread imu_thread(processImu, files_imu, &bag);

    if (messageType == "pointcloud2")
    {
        std::vector<mandeye::Point> points;
        std::optional<double> last_ts;
        for (auto& pcName : files_laz)
        {
            auto new_points = mandeye::load(pcName);
            for (auto p : new_points)
            {
                if (p.timestamp == 0)
                    continue;
                if (!last_ts)
                {
                    last_ts = p.timestamp;
                }
                points.push_back(p);
                if (points.size() > number_of_points && last_ts && last_ts > 0)
                {
                    sensor_msgs::msg::PointCloud2 pc2 = CreatePointcloudMessage(points);
                    pc2.header.stamp = GetRosTimeSecond(*last_ts);
                    pc2.header.frame_id = "livox";
                    bag.write(pc2, "/livox/pointcloud", pc2.header.stamp);
                    points.clear();
                    last_ts = nullopt;
                }
            }
        }
    }
    else if (messageType == "livox")
    {
        livox_ros_driver2::msg::CustomMsg custom_msg;
        custom_msg.lidar_id = 192;
        custom_msg.header.frame_id = "livox";
        custom_msg.rsvd[0] = 0;
        custom_msg.rsvd[1] = 0;
        custom_msg.rsvd[2] = 0;
        custom_msg.points.reserve(number_of_points);

        int line_id = 0;
        for (auto& pcName : files_laz)
        {
            auto points = mandeye::load(pcName);
            for (auto p : points)
            {
                if (!p.timestamp)
                    continue;

                if (!custom_msg.points.size())
                {
                    custom_msg.header.stamp = GetRosTimeSecond(p.timestamp);
                    custom_msg.timebase = p.timestamp * 1e9;
                }

                livox_ros_driver2::msg::CustomPoint cp;
                cp.tag = 0;
                cp.offset_time = p.timestamp * 1e9 - custom_msg.timebase;
                cp.reflectivity = p.intensity;
                cp.x = p.point.x();
                cp.y = p.point.y();
                cp.z = p.point.z();
                cp.line = line_id;

                custom_msg.points.push_back(cp);

                line_id = (line_id + 1) % number_of_lines;

                if (custom_msg.points.size() >= number_of_points)
                {
                    custom_msg.point_num = custom_msg.points.size();
                    bag.write(custom_msg, "/livox/lidar", custom_msg.header.stamp);
                    custom_msg.points.clear();
                }
            }
        }
    }

    imu_thread.join();

    bag.close();
    return 0;
}
