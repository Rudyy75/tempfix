enum PacketID : uint8_t {
    BNOREADING = 1,
    LIDAR = 2,
    PWM = 3,
    MOTORSPEEDS  = 4,
};

#pragma pack(push, 1)
struct BnoReading {
    float current_angle;
};  
#pragma pack(pop)

#pragma pack(push, 1)   //is ts 1 or id ig 1??
struct Lidar {
    float distance;
};  
#pragma pack(pop)

#pragma pack(push, 1)
struct Pwm {
    float m1;
    float m2;
    float m3;
    float m4;
};  
#pragma pack(pop)

#pragma pack(push, 1)
struct MotorSpeeds {
    float vx;
    float vy;
    float vw;
    
};  
#pragma pack(pop)

size_t get_packet_size(uint8_t id) {
    switch (id) {
        case BNOREADING:    return sizeof(BnoReading);
        case LIDAR:    return sizeof(Lidar);
        case PWM:    return sizeof(Pwm);
        case MOTORSPEEDS:    return sizeof(MotorSpeeds);
        default:      return 0; // unknown
    }
}
