
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

extern "C"
{
    #include "ecrt.h"
}

namespace indradrive {

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

uint16_t idn_to_hex(const std::string& idn);

class EthercatController : public IndradriveCSRobotHW
{
public:
  EthercatController(ros::NodeHandle& nh, std::string& joint_name) :
    IndradriveCSRobotHW(nh, joint_name),
    master_(NULL),
    domain_(NULL),
    domain_pd_(NULL)
  {
    configurePDOs();
  }
  virtual void init();
  virtual void read();
  virtual void write();

  ~EthercatController();

protected:
  virtual void configureIDNs() {}
  virtual void readTelegram() = 0;
  virtual void writeTelegram() = 0;

  // EtherCAT
  ec_master_t* master_;
  ec_master_state_t master_state_;

  ec_domain_t* domain_;
  ec_domain_state_t domain1_state_;

  uint8_t* domain_pd_; // process data
  ec_slave_config_t *slv_cfg_;

  PDOList at_pdo_configs_;
  PDOList mdt_pdo_configs_;
};

}
