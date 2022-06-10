#ifndef BC_NODE__BROADCAST_MESSAGE_HPP_
#define BC_NODE__BROADCAST_MESSAGE_HPP_

#include <cstring>
#include <string>
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
        memcpy(&lon, &message[8], sizeof(lon));
        memcpy(&alt, &message[16], sizeof(alt));
    }
};

struct GeoPointMin // 2*4 + 2 = 10 Bytes = 80 bits
{
    int32_t lat; // lat deg * 1 000 000
    int32_t lon; // lon deg * 1 000 000
    int16_t alt; // alt_m * 10 -> 0.1 m accuracy

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
        memcpy(&lon, &message[4], sizeof(lon));
        memcpy(&alt, &message[8], sizeof(alt));
    }
};

struct Point // 3*8 bytes = 24 Bytes = 192 bits
{
    double x;
    double y;
    double z;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(double));
        serialized.append((const char*) &y, sizeof(double));
        serialized.append((const char*) &z, sizeof(double));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(double));
        memcpy(&y, &message[8], sizeof(double));
        memcpy(&z, &message[16], sizeof(double));
    }
};

struct Point32 // 3*4 bytes = 12 Bytes = 96 bits
{
    float x;
    float y;
    float z;

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
        memcpy(&y, &message[4], sizeof(y));
        memcpy(&z, &message[8], sizeof(z));
    }
};

struct Point16 // 3*2 bytes = 6 Bytes = 48 bits
{
    int16_t x;
    int16_t y;
    int16_t z;

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
        memcpy(&y, &message[2], sizeof(y));
        memcpy(&z, &message[4], sizeof(z));
    }
};

struct Quaternion // 4*8 bytes = 32 Bytes = 256 bits
{
    double x;
    double y;
    double z;
    double w;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(double));
        serialized.append((const char*) &y, sizeof(double));
        serialized.append((const char*) &z, sizeof(double));
        serialized.append((const char*) &w, sizeof(double));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(double));
        memcpy(&y, &message[8], sizeof(double));
        memcpy(&z, &message[16], sizeof(double));
        memcpy(&w, &message[24], sizeof(double));
    }
};

struct Quaternion32 // 4*4 bytes = 16 Bytes = 128 bits
{
    float x;
    float y;
    float z;
    float w;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(x));
        serialized.append((const char*) &y, sizeof(y));
        serialized.append((const char*) &z, sizeof(z));
        serialized.append((const char*) &w, sizeof(w));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(x));
        memcpy(&y, &message[4], sizeof(y));
        memcpy(&z, &message[8], sizeof(z));
        memcpy(&w, &message[12], sizeof(w));
    }
};

struct Quaternion16 // 4*2 bytes = 8 Bytes = 128 bits
{
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t w;

    std::string serialize() const
    {
        std::string serialized;
        serialized.append((const char*) &x, sizeof(x));
        serialized.append((const char*) &y, sizeof(y));
        serialized.append((const char*) &z, sizeof(z));
        serialized.append((const char*) &w, sizeof(w));
        return serialized;
    }
    void deserialize(const std::string& message)
    {
        memcpy(&x, &message[0], sizeof(x));
        memcpy(&y, &message[2], sizeof(y));
        memcpy(&z, &message[4], sizeof(z));
        memcpy(&w, &message[6], sizeof(w));
    }
};

struct Pose // 56 Bytes = 448 bits
{
    Point point;
    Quaternion orientation;
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
        orientation.deserialize(message.substr(24));
    }
};

struct Pose32 //
{
    Point32 point;
    Quaternion32 orientation;
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
        orientation.deserialize(message.substr(12));
    }
};

struct Pose16 //
{
    Point16 point;
    Quaternion16 orientation;
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
        orientation.deserialize(message.substr(6));
    }
};

struct BroadcastMessage // 20 + 2 + 4 + 4 + 24 + 10*56 = 614 B
{
    char droneid[20];
    uint16_t priority;
    uint32_t sec;
    uint32_t nsec;
    bc_node::GeoPoint datum;
    bc_node::Pose path[10];

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
        // memcpy(&droneid, &message, 20);
        memcpy(&priority, &message[20], sizeof(priority));
        memcpy(&sec, &message[22], sizeof(sec));
        memcpy(&nsec, &message[26], sizeof(nsec));
        datum.deserialize(message.substr(30));
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(54 + 56 * i));
        }
    }
};

struct BroadcastMessage32 // 20 + 2 + 4 + 4 + 24 + 10*28 = 334 B
{
    char droneid[20];
    uint16_t priority;
    uint32_t sec;
    uint32_t nsec;
    bc_node::GeoPoint datum;
    bc_node::Pose32 path[10];

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
        // memcpy(&droneid, &message, 20);
        memcpy(&priority, &message[20], sizeof(priority));
        memcpy(&sec, &message[22], sizeof(sec));
        memcpy(&nsec, &message[26], sizeof(nsec));
        datum.deserialize(message.substr(30));
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(54 + 28 * i));
        }
    }
};

struct BroadcastMessage16 // 20 + 2 + 4 + 4 + 24 + 10*14 = 194 B
{
    char droneid[20];
    uint16_t priority;
    uint32_t sec;
    uint32_t nsec;
    bc_node::GeoPoint datum;
    bc_node::Pose16 path[10];

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
        // memcpy(&droneid, &message, 20);
        memcpy(&priority, &message[20], sizeof(priority));
        memcpy(&sec, &message[22], sizeof(sec));
        memcpy(&nsec, &message[26], sizeof(nsec));
        datum.deserialize(message.substr(30));
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(54 + 14 * i));
        }
    }
};

struct BroadcastMessageMin // 20 + 1 + 4 + 4 + 10 + 10*6 = 99 B
{
    char droneid[20];
    uint32_t sec;
    uint32_t nsec;
    bc_node::GeoPointMin datum;
    bc_node::Point16 path[10];
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
        // memcpy(&droneid, &message, 20);
        memcpy(&priority, &message[20], sizeof(priority));
        memcpy(&sec, &message[22], sizeof(sec));
        memcpy(&nsec, &message[26], sizeof(nsec));
        datum.deserialize(message.substr(30));
        for (int i = 0; i < 10; ++i)
        {
            path[i].deserialize(message.substr(39 + 6 * i));
        }
    }
};

} // namespace bc_node
#endif