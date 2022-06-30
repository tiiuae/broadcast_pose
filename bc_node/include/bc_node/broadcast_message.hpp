#ifndef BC_NODE__BROADCAST_MESSAGE_HPP_
#define BC_NODE__BROADCAST_MESSAGE_HPP_

#include <cmath>
#include <cstring>
#include <fognav_msgs/msg/trajectory.hpp>
#include <fognav_msgs/msg/trajectory_pose.hpp>
#include <string>
#include <type_traits>

namespace bc_node {

struct GeoPoint // 3*8 bytes = 24 Bytes = 192 bits
{
    double lat;
    double lon;
    double alt;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &lat, sizeof(lat));
        serialized.append((const char*) &lon, sizeof(lon));
        serialized.append((const char*) &alt, sizeof(alt));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&lat, &message[0], sizeof(lat));
        memcpy(&lon, &message[sizeof(lat)], sizeof(lon));
        memcpy(&alt, &message[sizeof(lat) + sizeof(lon)], sizeof(alt));
    }
    static size_t size() { return sizeof(lat) + sizeof(lon) + sizeof(alt); }
    size_t length() const { return sizeof(lat) + sizeof(lon) + sizeof(alt); }
};

struct GeoPointMin // 2*4 + 2 = 10 Bytes = 80 bits
{
    std::int32_t lat; // lat deg * 1 000 000
    std::int32_t lon; // lon deg * 1 000 000
    std::int16_t alt; // alt_m * 10 -> 0.1 m accuracy

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &lat, sizeof(lat));
        serialized.append((const char*) &lon, sizeof(lon));
        serialized.append((const char*) &alt, sizeof(alt));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&lat, &message[0], sizeof(lat));
        memcpy(&lon, &message[sizeof(lat)], sizeof(lon));
        memcpy(&alt, &message[sizeof(lat) + sizeof(lon)], sizeof(alt));
    }
    static size_t size() { return sizeof(lat) + sizeof(lon) + sizeof(alt); }
    size_t length() const { return sizeof(lat) + sizeof(lon) + sizeof(alt); }
};

template<typename T>
struct Point
{
    T x;
    T y;
    T z;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(x));
        serialized.append((const char*) &y, sizeof(y));
        serialized.append((const char*) &z, sizeof(z));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(x));
        memcpy(&y, &message[sizeof(x)], sizeof(y));
        memcpy(&z, &message[sizeof(x) + sizeof(y)], sizeof(z));
    }
    static size_t size() { return sizeof(x) + sizeof(y) + sizeof(z); }
    size_t length() const { return sizeof(x) + sizeof(y) + sizeof(z); }
};

template<typename T>
struct Quaternion
{
    T x;
    T y;
    T z;
    T w;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(T));
        serialized.append((const char*) &y, sizeof(T));
        serialized.append((const char*) &z, sizeof(T));
        serialized.append((const char*) &w, sizeof(T));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(x));
        memcpy(&y, &message[sizeof(x)], sizeof(y));
        memcpy(&z, &message[sizeof(x) + sizeof(y)], sizeof(z));
        memcpy(&w, &message[sizeof(x) + sizeof(y) + sizeof(z)], sizeof(w));
    }
    static size_t size() { return sizeof(x) + sizeof(y) + sizeof(z) + sizeof(w); }
    size_t length() const { return sizeof(x) + sizeof(y) + sizeof(z) + sizeof(w); }
};

template<typename T>
struct Pose
{
    Point<T> point;
    Quaternion<T> orientation;
    std::string serialize() const
    {
        std::string serialized;
        serialized.append(point.serialize());
        serialized.append(orientation.serialize());
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        point.deserialize(message);
        orientation.deserialize(message.substr(Point<T>::size()));
    }
    static size_t size() { return Point<T>::size() + Quaternion<T>::size(); }
    size_t length() const { return point.length() + orientation.length(); }
};

template<typename T>
struct BroadcastMessagePose
{
    char droneid[20];
    std::uint16_t priority;
    std::uint32_t sec;
    std::uint32_t nsec;
    bc_node::GeoPoint datum;
    bc_node::Pose<T> path[10];

    std::string serialize() const
    {
        std::string serialized;
        for (int k = 0; k < 20; k++)
        {
            serialized += droneid[k];
        }
        serialized.append((const char*) &priority, sizeof(priority));
        serialized.append((const char*) &sec, sizeof(sec));
        serialized.append((const char*) &nsec, sizeof(nsec));
        serialized.append(datum.serialize());
        for (int i = 0; i < 10; ++i)
        {
            serialized.append(path[i].serialize());
        }
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        for (int j = 0; j < 20; j++)
        {
            droneid[j] = message[j];
        }
        size_t offset = 20;
        memcpy(&priority, &message[offset], sizeof(priority));
        offset += sizeof(priority);
        memcpy(&sec, &message[offset], sizeof(sec));
        offset += sizeof(sec);
        memcpy(&nsec, &message[offset], sizeof(nsec));
        offset += sizeof(nsec);
        datum.deserialize(message.substr(offset));
        offset += GeoPoint::size();
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(offset + Pose<T>::size() * i));
        }
    }
    static size_t size()
    {
        return 20 + sizeof(priority) + sizeof(sec) + sizeof(nsec) + GeoPoint::size()
               + 10 * Pose<T>::size();
    }
    size_t length() const
    {
        return 20 + sizeof(priority) + sizeof(sec) + sizeof(nsec) + datum.length()
               + 10 * path[0].length();
    }
    void to_rosmsg(fognav_msgs::msg::TrajectoryPose::UniquePtr& trajectory) const
    {
        trajectory->droneid = std::string(droneid);
        trajectory->priority = priority;
        trajectory->header.stamp.sec = sec;
        trajectory->header.stamp.nanosec = nsec;
        trajectory->datum.latitude = datum.lat;
        trajectory->datum.longitude = datum.lon;
        trajectory->datum.altitude = datum.alt;
        if (std::is_integral<T>::value)
        {
            for (int i = 0; i < 10; i++)
            {
                trajectory->poses[i].position.x = 0.1 * static_cast<double>(path[i].point.x);
                trajectory->poses[i].position.y = 0.1 * static_cast<double>(path[i].point.y);
                trajectory->poses[i].position.z = 0.1 * static_cast<double>(path[i].point.z);

                trajectory->poses[i].orientation.x = 0.0001
                                                     * static_cast<double>(path[i].orientation.x);
                trajectory->poses[i].orientation.y = 0.0001
                                                     * static_cast<double>(path[i].orientation.y);
                trajectory->poses[i].orientation.z = 0.0001
                                                     * static_cast<double>(path[i].orientation.z);
                trajectory->poses[i].orientation.w = 0.0001
                                                     * static_cast<double>(path[i].orientation.w);
            }
        }
        else
        {
            for (int i = 0; i < 10; i++)
            {
                trajectory->poses[i].position.x = path[i].point.x;
                trajectory->poses[i].position.y = path[i].point.y;
                trajectory->poses[i].position.z = path[i].point.z;
                trajectory->poses[i].orientation.x = path[i].orientation.x;
                trajectory->poses[i].orientation.y = path[i].orientation.y;
                trajectory->poses[i].orientation.z = path[i].orientation.z;
                trajectory->poses[i].orientation.w = path[i].orientation.w;
            }
        }
    }

    void from_rosmsg(const fognav_msgs::msg::TrajectoryPose& trajectory)
    {
        strncpy(droneid, trajectory.droneid.c_str(), 20);
        priority = trajectory.priority;
        sec = trajectory.header.stamp.sec;
        nsec = trajectory.header.stamp.nanosec;
        datum.lat = trajectory.datum.latitude;
        datum.lon = trajectory.datum.longitude;
        datum.alt = trajectory.datum.altitude;
        if (std::is_integral<T>::value)
        {
            for (int i = 0; i < 10; i++)
            {
                path[i].point.x = static_cast<T>(std::round(10. * trajectory.poses[i].position.x));
                path[i].point.y = static_cast<T>(std::round(10. * trajectory.poses[i].position.y));
                path[i].point.z = static_cast<T>(std::round(10. * trajectory.poses[i].position.z));

                path[i].orientation.x = static_cast<T>(
                    std::round(10000. * trajectory.poses[i].orientation.x));
                path[i].orientation.y = static_cast<T>(
                    std::round(10000. * trajectory.poses[i].orientation.y));
                path[i].orientation.z = static_cast<T>(
                    std::round(10000. * trajectory.poses[i].orientation.z));
                path[i].orientation.w = static_cast<T>(
                    std::round(10000. * trajectory.poses[i].orientation.w));
            }
        }
        else
        {
            for (int i = 0; i < 10; i++)
            {
                path[i].point.x = trajectory.poses[i].position.x;
                path[i].point.y = trajectory.poses[i].position.y;
                path[i].point.z = trajectory.poses[i].position.z;

                path[i].orientation.x = trajectory.poses[i].orientation.x;
                path[i].orientation.y = trajectory.poses[i].orientation.y;
                path[i].orientation.z = trajectory.poses[i].orientation.z;
                path[i].orientation.w = trajectory.poses[i].orientation.w;
            }
        }
    }
};

template<typename T>
struct BroadcastMessagePoint
{
    char droneid[20];
    std::uint32_t sec;
    std::uint32_t nsec;
    bc_node::GeoPoint datum;
    bc_node::Point<T> path[10];
    std::uint16_t priority;

    std::string serialize() const
    {
        std::string serialized;
        for (int k = 0; k < 20; k++)
        {
            serialized += droneid[k];
        }
        serialized.append((const char*) &priority, sizeof(priority));
        serialized.append((const char*) &sec, sizeof(sec));
        serialized.append((const char*) &nsec, sizeof(nsec));
        serialized.append(datum.serialize());
        for (int i = 0; i < 10; ++i)
        {
            serialized.append(path[i].serialize());
        }
        return serialized;
    }

    void deserialize(const std::string& message)
    {
        for (int j = 0; j < 20; j++)
        {
            droneid[j] = message[j];
        }
        size_t offset = 20;
        memcpy(&priority, &message[offset], sizeof(priority));
        offset += sizeof(priority);
        memcpy(&sec, &message[offset], sizeof(sec));
        offset += sizeof(sec);
        memcpy(&nsec, &message[offset], sizeof(nsec));
        offset += sizeof(nsec);
        datum.deserialize(message.substr(offset));
        offset += GeoPoint::size();
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(offset + Point<T>::size() * i));
        }
    }
    static size_t size()
    {
        return 20 + sizeof(priority) + sizeof(sec) + sizeof(nsec) + GeoPoint::size()
               + 10 * Point<T>::size();
    }
    size_t length() const
    {
        return 20 + sizeof(priority) + sizeof(sec) + sizeof(nsec) + datum.length()
               + 10 * path[0].length();
    }
    void to_rosmsg(fognav_msgs::msg::Trajectory::UniquePtr& trajectory) const
    {
        trajectory->droneid = std::string(droneid); //, 20);
        if (trajectory->droneid.length() > 20)
        {
            trajectory->droneid.resize(20);
        }
        trajectory->priority = priority;
        trajectory->header.stamp.sec = sec;
        trajectory->header.stamp.nanosec = nsec;
        trajectory->datum.latitude = datum.lat;
        trajectory->datum.longitude = datum.lon;
        trajectory->datum.altitude = datum.alt;
        if (std::is_integral<T>::value)
        {
            for (int i = 0; i < 10; i++)
            {
                trajectory->path[i].x = 0.1 * static_cast<double>(path[i].x);
                trajectory->path[i].y = 0.1 * static_cast<double>(path[i].y);
                trajectory->path[i].z = 0.1 * static_cast<double>(path[i].z);
            }
        }
        else
        {
            for (int i = 0; i < 10; i++)
            {
                trajectory->path[i].x = path[i].x;
                trajectory->path[i].y = path[i].y;
                trajectory->path[i].z = path[i].z;
            }
        }
    }

    void from_rosmsg(const fognav_msgs::msg::Trajectory& trajectory)
    {
        strncpy(droneid, trajectory.droneid.c_str(), 20);
        priority = trajectory.priority;
        sec = trajectory.header.stamp.sec;
        nsec = trajectory.header.stamp.nanosec;
        datum.lat = trajectory.datum.latitude;
        datum.lon = trajectory.datum.longitude;
        datum.alt = trajectory.datum.altitude;

        if (std::is_integral<T>::value)
        {
            for (int i = 0; i < 10; i++)
            {
                path[i].x = static_cast<T>(std::round(10. * trajectory.path[i].x));
                path[i].y = static_cast<T>(std::round(10. * trajectory.path[i].y));
                path[i].z = static_cast<T>(std::round(10. * trajectory.path[i].z));
            }
        }
        else
        {
            for (int i = 0; i < 10; i++)
            {
                path[i].x = trajectory.path[i].x;
                path[i].y = trajectory.path[i].y;
                path[i].z = trajectory.path[i].z;
            }
        }
    }
};

struct BroadcastMessageMin // 20 + 1 + 4 + 4 + 10 + 10*6 = 99 B
{
    char droneid[20];
    uint32_t sec;
    uint32_t nsec;
    bc_node::GeoPointMin datum;
    bc_node::Point<std::int16_t> path[10];
    uint8_t priority;
    std::string serialize() const
    {
        std::string serialized;
        for (int k = 0; k < 20; k++)
        {
            serialized += droneid[k];
        }
        serialized.append((const char*) &priority, sizeof(priority));
        serialized.append((const char*) &sec, sizeof(sec));
        serialized.append((const char*) &nsec, sizeof(nsec));
        serialized.append(datum.serialize());
        for (int i = 0; i < 10; ++i)
        {
            serialized.append(path[i].serialize());
        }
        return serialized;
    }

    void deserialize(const std::string& message)
    {
        for (int j = 0; j < 20; j++)
        {
            droneid[j] = message[j];
        }
        size_t offset = 20;
        memcpy(&priority, &message[offset], sizeof(priority));
        offset += sizeof(priority);
        memcpy(&sec, &message[offset], sizeof(sec));
        offset += sizeof(sec);
        memcpy(&nsec, &message[offset], sizeof(nsec));
        offset += sizeof(nsec);
        datum.deserialize(message.substr(offset));
        offset += GeoPointMin::size();
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(offset + Point<std::int16_t>::size() * i));
        }
    }
    static size_t size()
    {
        return 20 + sizeof(priority) + sizeof(sec) + sizeof(nsec) + GeoPointMin::size()
               + 10 * Point<std::int16_t>::size();
    }
    size_t length() const
    {
        return 20 + sizeof(priority) + sizeof(sec) + sizeof(nsec) + datum.length()
               + 10 * path[0].length();
    }
    void to_rosmsg(fognav_msgs::msg::Trajectory::UniquePtr& trajectory) const
    {
        trajectory->droneid = std::string(droneid);
        trajectory->priority = priority;
        trajectory->header.stamp.sec = sec;
        trajectory->header.stamp.nanosec = nsec;
        trajectory->datum.latitude = 0.000001 * static_cast<double>(datum.lat);
        trajectory->datum.longitude = 0.000001 * static_cast<double>(datum.lon);
        trajectory->datum.altitude = 0.1 * static_cast<double>(datum.alt);
        for (int i = 0; i < 10; i++)
        {
            trajectory->path[i].x = 0.1 * static_cast<double>(path[i].x);
            trajectory->path[i].y = 0.1 * static_cast<double>(path[i].y);
            trajectory->path[i].z = 0.1 * static_cast<double>(path[i].z);
        }
    }

    void from_rosmsg(const fognav_msgs::msg::Trajectory& trajectory)
    {
        strncpy(droneid, trajectory.droneid.c_str(), 20);
        priority = trajectory.priority;
        sec = trajectory.header.stamp.sec;
        nsec = trajectory.header.stamp.nanosec;
        datum.lat = static_cast<std::int32_t>(std::round(trajectory.datum.latitude * 1000000.));
        datum.lon = static_cast<std::int32_t>(std::round(trajectory.datum.longitude * 1000000.));
        datum.alt = static_cast<std::int16_t>(trajectory.datum.altitude * 10.);
        for (int i = 0; i < 10; i++)
        {
            path[i].x = static_cast<std::int16_t>(std::round(10. * trajectory.path[i].x));
            path[i].y = static_cast<std::int16_t>(std::round(10. * trajectory.path[i].y));
            path[i].z = static_cast<std::int16_t>(std::round(10. * trajectory.path[i].z));
        }
    }
};
} // namespace bc_node
#endif