
#include <errno.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>

#include <indradrive_hw_iface/cs_robot_hw.h>

extern "C"
{
    #include "ecrt.h"
}

namespace indradrive {

#define MEASURE_TIMING
#define CLOCK_TO_USE CLOCK_REALTIME
#define PERIOD_NS 1000000
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS 1000000
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
	(B).tv_nsec - (A).tv_nsec)
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

#define LOWBYTE16(T) ((uint8_t) ((T) & 0x00ff))
#define HIGHBYTE16(T) ((uint8_t) ((T) >> 8))
#define LENDIAN(T) { LOWBYTE16((T)), HIGHBYTE16((T)) }
uint16_t idn_to_hex(const std::string& idn);

struct PDOConfiguration
{
  std::string name;
  std::string idn;
  uint16_t idn_bin;
  uint8_t subindex;
  uint16_t bit_len;
  bool in_telegram_config;
  uint8_t* data_address;
  int data_offset;

  PDOConfiguration(const std::string& name_, const std::string idn_, 
      uint8_t subindex_, uint16_t bit_len_, bool in_telegram_config_)
    : name(name_), idn(idn_), idn_bin(idn_to_hex(idn_)), subindex(subindex_),
    bit_len(bit_len_), in_telegram_config(in_telegram_config_), 
    data_address(NULL), data_offset(-1)
  {
  }

  int configureEntry(ec_slave_config_t *sc, ec_domain_t *domain_)
  {
    data_offset = ecrt_slave_config_reg_pdo_entry(sc, idn_bin, 0x00, domain_, NULL);
    if (data_offset < 0) {
      printf("Failed to configure %s\n", name.c_str());
      return -1;
    }
    return 0;
  }

  void registerDataAddress(uint8_t *domain_pd_)
  {
    data_address = domain_pd_ + data_offset;
  }

  void writeEntryInfo(ec_pdo_entry_info_t& entry_info)
  {
    entry_info.index = idn_bin;
    entry_info.subindex = subindex;
    entry_info.bit_length = bit_len;
  }
};

typedef std::vector<PDOConfiguration*> PDOList;

class EthercatController : public IndradriveCSRobotHW
{
public:
  EthercatController(
    ros::NodeHandle& nh, std::string& joint_name) :
    IndradriveCSRobotHW(nh, joint_name),
    master_(NULL),
    domain_(NULL),
    domain_pd_(NULL),
    counter_(0),
#ifdef MEASURE_TIMING
    period_ns(0), exec_ns(0), latency_ns(0),
    latency_min_ns(0), latency_max_ns(0),
    period_min_ns(0), period_max_ns(0),
    exec_min_ns(0), exec_max_ns(0),
#endif
    sync_ref_counter(0)
  {
    cycletime.tv_sec = 0; cycletime.tv_nsec = PERIOD_NS;
  }
  virtual int init();
  virtual void read();
  virtual void write();

  ~EthercatController();

protected:
  virtual int configureIDNs() { return 0; }
  virtual void readTelegram() = 0;
  virtual void writeTelegram() = 0;

  // EtherCAT
  ec_master_t* master_;
  ec_master_state_t master_state_;

  ec_domain_t* domain_;
  ec_domain_state_t domain1_state_;

  ec_slave_info_t slave_info_;
  uint8_t* domain_pd_; // process data
  ec_slave_config_t *slv_cfg_;

  PDOList at_pdo_configs_;
  PDOList mdt_pdo_configs_;

  uint64_t counter_;

  unsigned int sync_ref_counter;
  struct timespec cycletime;
  struct timespec wakeupTime, time;
#ifdef MEASURE_TIMING
  struct timespec startTime, endTime, lastStartTime;
  uint32_t period_ns, exec_ns, latency_ns,
           latency_min_ns, latency_max_ns,
           period_min_ns, period_max_ns,
           exec_min_ns, exec_max_ns;
#endif
};

}
