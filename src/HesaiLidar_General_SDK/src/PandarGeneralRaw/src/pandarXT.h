/**
 * Pandar XT
 */
// Head
#define HS_LIDAR_XT_HEAD_SIZE (12)
// Body
#define HS_LIDAR_XT_BLOCK_NUMBER (8)
#define HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH (2)
#define HS_LIDAR_XT_UNIT_NUM (32)
#define HS_LIDAR_XT_UNIT_SIZE (4)
#define HS_LIDAR_XT_BLOCK_SIZE (HS_LIDAR_XT_UNIT_SIZE * \
    HS_LIDAR_XT_UNIT_NUM + HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH)
#define HS_LIDAR_XT_BODY_SIZE (HS_LIDAR_XT_BLOCK_SIZE * \
    HS_LIDAR_XT_BLOCK_NUMBER)
//Tail
#define HS_LIDAR_XT_RESERVED_SIZE (10)
#define HS_LIDAR_XT_ENGINE_VELOCITY (2)
#define HS_LIDAR_XT_TIMESTAMP_SIZE (4)
#define HS_LIDAR_XT_ECHO_SIZE (1)
#define HS_LIDAR_XT_FACTORY_SIZE (1)
#define HS_LIDAR_XT_UTC_SIZE (6)
#define HS_LIDAR_XT_SEQUENCE_SIZE (4)
#define HS_LIDAR_XT_PACKET_TAIL_SIZE (28)
// All
#define HS_LIDAR_XT_PACKET_SIZE ( HS_LIDAR_XT_HEAD_SIZE + \
    HS_LIDAR_XT_BODY_SIZE + HS_LIDAR_XT_PACKET_TAIL_SIZE)
// XT16
#define HS_LIDAR_XT16_UNIT_NUM (16)
#define HS_LIDAR_XT16_PACKET_SIZE (568)
#define HS_LIDAR_XTM_PACKET_SIZE (820)

#define HS_LIDAR_XT_MAJOR_VERSION (6)


typedef struct HS_LIDAR_XT_Header_s{
    unsigned short sob;     // 0xFFEE 2bytes
    char chProtocolMajor;   // Protocol Version Major 1byte
    char chProtocolMinor;   // Protocol Version Minor 1byte
    char chLaserNumber;     // laser number 1byte
    char chBlockNumber;     // block number 1byte
    char chReturnType;      // return mode 1 byte  when dual return 0-Single Return 
    char chDisUnit;         // Distance unit, 4mm
    public:
    HS_LIDAR_XT_Header_s() {
        sob = 0;
        chProtocolMajor = 0;
        chProtocolMinor = 0;
        chLaserNumber = 0;
        chBlockNumber = 0;
        chReturnType = 0;
        chDisUnit = 0;
    }
} HS_LIDAR_XT_Header;

typedef struct HS_LIDAR_XT_Unit_s{
    double distance;
    unsigned short intensity;
    unsigned short confidence;
} HS_LIDAR_XT_Unit;

typedef struct HS_LIDAR_XT_Block_s{
    unsigned short azimuth; // packet angle  ,Azimuth = RealAzimuth * 100
    HS_LIDAR_XT_Unit units[HS_LIDAR_XT_UNIT_NUM];
} HS_LIDAR_XT_Block;

typedef struct HS_LIDAR_XT_Packet_s{
    HS_LIDAR_XT_Header header;
    HS_LIDAR_XT_Block blocks[HS_LIDAR_XT_BLOCK_NUMBER];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
    double timestamp_point;
    float spin_speed;
} HS_LIDAR_XT_Packet;


const float pandarXT_elev_angle_map[] = {
  15.0f, 14.0f, 13.0f, 12.0f, 11.0f, 10.0f, 9.0f, 8.0f, \
  7.0f,  6.0f,  5.0f,  4.0f,  3.0f,  2.0f,  1.0f, 0.0f,  \
  -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -7.0f, -8.0f, \
  -9.0f, -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -15.0f, -16.0f
};

const float pandarXT_horizatal_azimuth_offset_map[] = {
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

const float blockXTOffsetDual[HS_LIDAR_XT_BLOCK_NUMBER] = {
    5.632f - 50.0f * 3.0f,
    5.632f - 50.0f * 3.0f,
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f
};  
const float blockXTOffsetSingle[HS_LIDAR_XT_BLOCK_NUMBER] = {
    5.632f - 50.0f * 7.0f,
    5.632f - 50.0f * 6.0f,
    5.632f - 50.0f * 5.0f,
    5.632f - 50.0f * 4.0f,
    5.632f - 50.0f * 3.0f,
    5.632f - 50.0f * 2.0f, 
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f
};

const float laserXTOffset[HS_LIDAR_XT_UNIT_NUM] = {
    1.512f * 0.0f + 0.368f,
    1.512f * 1.0f + 0.368f,
    1.512f * 2.0f + 0.368f,
    1.512f * 3.0f + 0.368f,
    1.512f * 4.0f + 0.368f,
    1.512f * 5.0f + 0.368f,
    1.512f * 6.0f + 0.368f,
    1.512f * 7.0f + 0.368f,

    1.512f * 8.0f + 0.368f,
    1.512f * 9.0f + 0.368f,
    1.512f * 10.0f + 0.368f,
    1.512f * 11.0f + 0.368f,
    1.512f * 12.0f + 0.368f,
    1.512f * 13.0f + 0.368f,
    1.512f * 14.0f + 0.368f,
    1.512f * 15.0f + 0.368f,

    1.512f * 16.0f + 0.368f,
    1.512f * 17.0f + 0.368f,
    1.512f * 18.0f + 0.368f,
    1.512f * 19.0f + 0.368f,
    1.512f * 20.0f + 0.368f,
    1.512f * 21.0f + 0.368f,
    1.512f * 22.0f + 0.368f,
    1.512f * 23.0f + 0.368f,

    1.512f * 24.0f + 0.368f,
    1.512f * 25.0f + 0.368f,
    1.512f * 26.0f + 0.368f,
    1.512f * 27.0f + 0.368f,
    1.512f * 28.0f + 0.368f,
    1.512f * 29.0f + 0.368f,
    1.512f * 30.0f + 0.368f,
    1.512f * 31.0f + 0.368f
};

const float pandarXTM_elev_angle_map[] = {
  19.5f, 18.2f, 16.9f, 15.6f, 14.3f, 13.0f, 11.7f, 10.4f, \
  9.1f,  7.8f,  6.5f,  5.2f,  3.9f,  2.6f,  1.3f, 0.0f,  \
  -1.3f, -2.6f, -3.9f, -5.2f, -6.5f, -7.8f, -9.1f, -10.4f, \
  -11.7f, -13.0f, -14.3f, -15.6f, -16.9f, -18.2f, -19.5f, -20.8f
};

const float pandarXTM_horizatal_azimuth_offset_map[] = {
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, \
   0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
};

const float blockXTMOffsetTriple[HS_LIDAR_XT_BLOCK_NUMBER] = {
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f
}; 

const float blockXTMOffsetDual[HS_LIDAR_XT_BLOCK_NUMBER] = {
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f
};  
const float blockXTMOffsetSingle[HS_LIDAR_XT_BLOCK_NUMBER] = {
    5.632f - 50.0f * 5.0f,
    5.632f - 50.0f * 4.0f,
    5.632f - 50.0f * 3.0f,
    5.632f - 50.0f * 2.0f,
    5.632f - 50.0f * 1.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f,
    5.632f - 50.0f * 0.0f
};

const float laserXTMOffset[HS_LIDAR_XT_UNIT_NUM] = {
    2.856f * 0.0f + 0.368f,
    2.856f * 1.0f + 0.368f,
    2.856f * 2.0f + 0.368f,
    2.856f * 3.0f + 0.368f,
    2.856f * 4.0f + 0.368f,
    2.856f * 5.0f + 0.368f,
    2.856f * 6.0f + 0.368f,
    2.856f * 7.0f + 0.368f,

    2.856f * 8.0f + 0.368f,
    2.856f * 9.0f + 0.368f,
    2.856f * 10.0f + 0.368f,
    2.856f * 11.0f + 0.368f,
    2.856f * 12.0f + 0.368f,
    2.856f * 13.0f + 0.368f,
    2.856f * 14.0f + 0.368f,
    2.856f * 15.0f + 0.368f,

    2.856f * 0.0f + 0.368f,
    2.856f * 1.0f + 0.368f,
    2.856f * 2.0f + 0.368f,
    2.856f * 3.0f + 0.368f,
    2.856f * 4.0f + 0.368f,
    2.856f * 5.0f + 0.368f,
    2.856f * 6.0f + 0.368f,
    2.856f * 7.0f + 0.368f,

    2.856f * 8.0f + 0.368f,
    2.856f * 9.0f + 0.368f,
    2.856f * 10.0f + 0.368f,
    2.856f * 11.0f + 0.368f,
    2.856f * 12.0f + 0.368f,
    2.856f * 13.0f + 0.368f,
    2.856f * 14.0f + 0.368f,
    2.856f * 15.0f + 0.368f
};

