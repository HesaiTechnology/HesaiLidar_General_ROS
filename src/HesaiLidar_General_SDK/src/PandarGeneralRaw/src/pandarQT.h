/**
 * Pandar QT
 */
// Head
#define HS_LIDAR_QT_HEAD_SIZE (12)
#define HS_LIDAR_QT_PRE_HEADER_SIZE (6)
#define HS_LIDAR_QT_HEADER_SIZE (6)
// Body
#define HS_LIDAR_QT_BLOCK_NUMBER (4)
#define HS_LIDAR_QT_BLOCK_HEADER_AZIMUTH (2)
#define HS_LIDAR_QT_UNIT_NUM (64)
#define HS_LIDAR_QT_UNIT_SIZE (4)
#define HS_LIDAR_QT_BLOCK_SIZE (HS_LIDAR_QT_UNIT_SIZE * \
    HS_LIDAR_QT_UNIT_NUM + HS_LIDAR_QT_BLOCK_HEADER_AZIMUTH)
#define HS_LIDAR_QT_BODY_SIZE (HS_LIDAR_QT_BLOCK_SIZE * \
    HS_LIDAR_QT_BLOCK_NUMBER)
//Tail
#define HS_LIDAR_QT_RESERVED_SIZE (10)
#define HS_LIDAR_QT_ENGINE_VELOCITY (2)
#define HS_LIDAR_QT_TIMESTAMP_SIZE (4)
#define HS_LIDAR_QT_ECHO_SIZE (1)
#define HS_LIDAR_QT_FACTORY_SIZE (1)
#define HS_LIDAR_QT_UTC_SIZE (6)
#define HS_LIDAR_QT_SEQUENCE_SIZE (4)
#define HS_LIDAR_QT_PACKET_TAIL_SIZE (28)
#define HS_LIDAR_QT_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE (24)
// All
#define HS_LIDAR_QT_PACKET_SIZE ( HS_LIDAR_QT_HEAD_SIZE + \
    HS_LIDAR_QT_BODY_SIZE + HS_LIDAR_QT_PACKET_TAIL_SIZE)
#define HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE ( HS_LIDAR_QT_HEAD_SIZE + \
    HS_LIDAR_QT_BODY_SIZE + HS_LIDAR_QT_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)


typedef struct HS_LIDAR_QT_Header_s{
    unsigned short sob;     // 0xFFEE 2bytes
    char chProtocolMajor;   // Protocol Version Major 1byte
    char chProtocolMinor;   // Protocol Version Minor 1byte
    char chLaserNumber;     // laser number 1byte
    char chBlockNumber;     // block number 1byte
    char chReturnType;      // return mode 1 byte  when dual return 0-Single Return 
                            // 1-The first block is the 1 st return. 
                            // 2-The first block is the 2 nd return
    char chDisUnit;         // Distance unit, 4mm
    public:
    HS_LIDAR_QT_Header_s() {
        sob = 0;
        chProtocolMajor = 0;
        chProtocolMinor = 0;
        chLaserNumber = 0;
        chBlockNumber = 0;
        chReturnType = 0;
        chDisUnit = 0;
    }
} HS_LIDAR_QT_Header;

typedef struct HS_LIDAR_QT_Unit_s{
    double distance;
    unsigned short intensity;
    unsigned short confidence;
} HS_LIDAR_QT_Unit;

typedef struct HS_LIDAR_QT_Block_s{
    unsigned short azimuth; // packet angle  ,Azimuth = RealAzimuth * 100
    HS_LIDAR_QT_Unit units[HS_LIDAR_QT_UNIT_NUM];
} HS_LIDAR_QT_Block;

typedef struct HS_LIDAR_QT_Packet_s{
    HS_LIDAR_QT_Header header;
    HS_LIDAR_QT_Block blocks[HS_LIDAR_QT_BLOCK_NUMBER];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
    double timestamp_point;
    float spin_speed;
} HS_LIDAR_QT_Packet;


const float pandarQT_elev_angle_map[] = {
  -52.121f,-49.785f,-47.577f,-45.477f,-43.465f,-41.528f,-39.653f,-37.831f, \
  -36.055f,-34.320f,-32.619f,-30.950f,-29.308f,-27.690f,-26.094f,-24.517f, \
  -22.964f,-21.420f,-19.889f,-18.372f,-16.865f,-15.368f,-13.880f,-12.399f, \
  -10.925f, -9.457f, -7.994f, -6.535f, -5.079f, -3.626f, -2.175f, -0.725f, \
    0.725f,  2.175f,  3.626f,  5.079f,  6.534f,  7.993f,  9.456f, 10.923f, \
   12.397f, 13.877f, 15.365f, 16.861f, 18.368f, 19.885f, 21.415f, 22.959f, \
   24.524f, 26.101f, 27.697f, 29.315f, 30.957f, 32.627f, 34.328f, 36.064f, \
   37.840f, 39.662f, 41.537f, 43.475f, 45.487f, 47.587f, 49.795f, 52.133f
};

const float pandarQT_horizatal_azimuth_offset_map[] = {
   8.736f, 8.314f, 7.964f, 7.669f, 7.417f, 7.198f, 7.007f, 6.838f, \
   6.688f, 6.554f, 6.434f, 6.326f, 6.228f, 6.140f, 6.059f, 5.987f, \
  -5.270f,-5.216f,-5.167f,-5.123f,-5.083f,-5.047f,-5.016f,-4.988f, \
  -4.963f,-4.942f,-4.924f,-4.910f,-4.898f,-4.889f,-4.884f,-4.881f, \
   5.493f, 5.496f, 5.502f, 5.512f, 5.525f, 5.541f, 5.561f, 5.584f, \
   5.611f, 5.642f, 5.676f, 5.716f, 5.759f, 5.808f, 5.862f, 5.921f, \
  -5.330f,-5.396f,-5.469f,-5.550f,-5.640f,-5.740f,-5.850f,-5.974f, \
  -6.113f,-6.269f,-6.447f,-6.651f,-6.887f,-7.163f,-7.493f,-7.892f
};