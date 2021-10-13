#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include "pcap_reader.h"
#include "log.h"
#include <map>
#include "../util.h"

#define IPV4_PKT_HEADER_SIZE (42)
#define IPV6_PKT_HEADER_SIZE (62)
#define IPV4_TYPE (0X0800)
#define IPV6_TYPE (0X86dd)
#define MAC_PACKET_HEADER_LEN (12)

PcapReader::PcapReader(std::string path, std::string frame_id) {
  initTimeIndexMap();
  pcapPath   = path;
  m_sFrameId = frame_id;
  loop       = false;
  parse_thr_ = NULL;
  std::map<std::string, std::pair<int,int>>::iterator iter = m_timeIndexMap.find(m_sFrameId);
  if(iter != m_timeIndexMap.end()) {
     m_iTsIndex = iter->second.first;
    m_iUTCIndex = iter->second.second;
  
  }
  else{
    m_iTsIndex = 0;
    m_iUTCIndex = 0;
  }
}

PcapReader::~PcapReader() {
  stop();
}

void PcapReader::initTimeIndexMap() {
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("Pandar40P", std::pair<int,int>(1250,1256)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("Pandar40M", std::pair<int,int>(1250,1256)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("Pandar64", std::pair<int,int>(1182,1188)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("PandarQT", std::pair<int,int>(1056,1062)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("Pandar20A", std::pair<int,int>(1258,1264)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("Pandar20B", std::pair<int,int>(1258,1264)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("PandarXT-32", std::pair<int,int>(1071,1065)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("PandarXT-16", std::pair<int,int>(559,553)));
  m_timeIndexMap.insert(std::pair<string,std::pair<int,int>>("PandarXTM", std::pair<int,int>(811,805)));
}

void PcapReader::start(boost::function<void(const uint8_t*, const int, double timestamp)> callback) {
  // LOG_FUNC();
  stop();

  this->callback = callback;
  loop           = true;

  parse_thr_ = new boost::thread(boost::bind(&PcapReader::parsePcap, this));
}

void PcapReader::stop() {
  loop = false;

  if (parse_thr_) {
    parse_thr_->join();
    delete parse_thr_;
    parse_thr_ = NULL;
  }
}

void PcapReader::parsePcap() {
  // LOG_FUNC();
  pcap_t *pcapFile = NULL;
  char pcapBuf[PCAP_ERRBUF_SIZE];
  struct bpf_program filter;
  pcap_pkthdr *pktHeader;
  const unsigned char *packetBuf;
  struct tm t = {0};
  static int gap = 100;
  int64_t last_pkt_ts = 0;
  int count;
  int64_t last_time;
  int64_t current_time;
  int64_t pkt_ts = 0;

  if(pcapPath.empty()){return;}

  pcapFile = pcap_open_offline(pcapPath.c_str(), pcapBuf);

  if (NULL == pcapFile) {
    printf("open pcap file %s fail\n", pcapPath.c_str());
    return;
  }

  if (pcap_compile(pcapFile, &filter, "udp", 0, 0xffffffff) == -1) {
    printf("compile pcap file fail\n");
    return;
  }

  if (pcap_setfilter(pcapFile, &filter) == -1) {
    printf("pcap set filter fail\n");
    return;
  }

  if (NULL == callback) {
    printf("pcap read callback is null\n");
    return;
  }
  while (pcap_next_ex(pcapFile, &pktHeader, &packetBuf) >= 0 && loop) {
    uint16_t ip_type = htons(*(uint16_t *)(packetBuf + MAC_PACKET_HEADER_LEN));
    const uint8_t *packet = nullptr;
    int pktSize = 0;
    switch (ip_type)
    {
    case IPV4_TYPE:
      packet = packetBuf + IPV4_PKT_HEADER_SIZE;
      pktSize = pktHeader->len - IPV4_PKT_HEADER_SIZE;
      break;
    case IPV6_TYPE:
      packet = packetBuf + IPV6_PKT_HEADER_SIZE;
      pktSize = pktHeader->len - IPV6_PKT_HEADER_SIZE;
      break;
    default:
      printf("ip type error %x\n", ip_type);
      break;
    }
 
    double time = getNowTimeSec();
    // printf("Real time: %lf\n",time);
    callback(packet, pktSize, time);
    count++;

    if (count >= gap && m_iUTCIndex != 0) {
      count = 0;

      t.tm_year  = packet[m_iUTCIndex];
      t.tm_mon   = packet[m_iUTCIndex+1] - 1;
      t.tm_mday  = packet[m_iUTCIndex+2];
      t.tm_hour  = packet[m_iUTCIndex+3];
      t.tm_min   = packet[m_iUTCIndex+4];
      t.tm_sec   = packet[m_iUTCIndex+5];
      // LOG_D("[%d][%d][%d][%d][%d][%d]",t.tm_year,t.tm_mon,t.tm_mday,t.tm_hour,t.tm_min,t.tm_sec);
      t.tm_isdst = 0;


      pkt_ts = mktime(&t) * 1000000 + ((packet[m_iTsIndex]& 0xff) | \
          (packet[m_iTsIndex+1]& 0xff) << 8 | \
          ((packet[m_iTsIndex+2]& 0xff) << 16) | \
          ((packet[m_iTsIndex+3]& 0xff) << 24));
      struct timeval sys_time;
      gettimeofday(&sys_time, NULL);
      current_time = sys_time.tv_sec * 1000000 + sys_time.tv_usec;

      if (0 == last_pkt_ts) {
        last_pkt_ts = pkt_ts;
        last_time = current_time;
      } else {
        int64_t sleep_time = (pkt_ts - last_pkt_ts) - \
            (current_time - last_time);
        // LOG_D("[%lld],[%lld],[%lld],[%lld]",pkt_ts,last_pkt_ts,current_time,last_time);
        // LOG_D("sleep time is: [%lld]", sleep_time);

        if (sleep_time > 0) {
          struct timeval waitTime;
          waitTime.tv_sec = sleep_time / 1000000;
          waitTime.tv_usec = sleep_time % 1000000;

          int err;

          do {
            err = select(0, NULL, NULL, NULL, &waitTime);
          } while (err < 0 && errno != EINTR);
        }

        last_pkt_ts = pkt_ts;
        last_time = current_time;
        last_time += sleep_time;
      }
    }
  }
  // LOG_D("read pcap file done");

  if (pcapFile != NULL) {
    pcap_close(pcapFile);
    pcapFile = NULL;
  }
}
