
#include <indradrive_hw_iface/ethercat_ctrl.h>

uint16_t idn_to_hex(const std::string& idn)
{
  uint16_t hex;
  if(idn[0] == 'S') {
    hex = 0x0000;
  } else if(idn[0] == 'P') {
    hex = 0x8000;
  } else {
    printf("INVALID IDN CONVERSION (not S/P)\n");
    return 0x0000;
  }
  hex |= atoi(idn.substr(2,1).c_str()) << 3*4;
  hex |= atoi(idn.substr(4,4).c_str());
  return hex;
}
#define LOWBYTE16(T) ((uint8_t) ((T) & 0x00ff))
#define HIGHBYTE16(T) ((uint8_t) ((T) >> 8))
#define LENDIAN(T) { LOWBYTE16((T)), HIGHBYTE16((T)) }

uint16_t createTelegramList(
    PDOList& pdo_configs, uint16_t max_len, uint8_t** idn_bytes)
{
  uint16_t len = 0;
  for(PDOList::iterator iter = pdo_configs.begin(); iter != pdo_configs.end(); iter++) 
    if((iter*)->in_telegram_config)
      len++;

  (*idn_bytes) = new uint8_t[4+2*len];
  uint16_t cur_byte = 0;
  (*idn_bytes)[cur_byte++] = LOWBYTE16(2*len);
  (*idn_bytes)[cur_byte++] = HIGHBYTE16(2*len);
  (*idn_bytes)[cur_byte++] = LOWBYTE16(max_len);
  (*idn_bytes)[cur_byte++] = HIGHBYTE16(max_len);
  for(PDOList::iterator iter = pdo_configs.begin(); iter != pdo_configs.end(); iter++) {
    if((iter*)->in_telegram_config) {
      (*idn_bytes)[cur_byte++] = LOWBYTE16((iter*)->idn_bin);
      (*idn_bytes)[cur_byte++] = HIGHBYTE16((iter*)->idn_bin);
    }
  }
  return cur_byte;
}
/*****************************************************************************/

void EthercatController::init()
{

#ifdef VELCTRL
  uint8_t vel_mode_data[2] = LENDIAN(0x0002);
#else
  uint8_t vel_mode_data[2] = LENDIAN(0x000B);
#endif

  master_ = ecrt_request_master(0);
  if (!master_)
    return -1;

  domain_ = ecrt_master_create_domain(master_);
  if (!domain_) {
    printf("Domain not created\n");
    return -1;
  }
  printf("Domain created\n");

  ec_slave_info_t slave_info;
  if(ecrt_master_get_slave(master_, 0, &slave_info)) {
    printf("Couldn't get slave info\n");
    return -1;
  }
  printf("Got slave info for: 0x%x, 0x%x\n", 
      slave_info.vendor_id, slave_info.product_code);

  if (!(slv_cfg_ = ecrt_master_slave_config(master_, 0, 0, 
          slave_info.vendor_id, slave_info.product_code))) {
    fprintf(stderr, "Failed to get slave configuration.\n");
    return -1;
  }
  printf("Initialized slave config.\n");

  ecrt_slave_config_watchdog(slv_cfg_, 2498, 1000);

  printf("Configuring sync managers...\n");
  ec_pdo_entry_info_t* at_entry_infos = new ec_pdo_entry_info_t[at_pdo_configs_.size()];
  uint16_t at_info_idx = 0;
  for(PDOList::iterator iter = at_pdo_configs_.begin(); iter != at_pdo_configs_.end(); iter++) 
    (iter*)->writeEntryInfo(at_entry_infos[at_info_idx++]);
  uint16_t mdt_info_idx = 0;
  ec_pdo_entry_info_t* mdt_entry_infos = new ec_pdo_entry_info_t[mdt_pdo_configs_.size()];
  for(PDOList::iterator iter = mdt_pdo_configs_.begin(); iter != mdt_pdo_configs_.end(); iter++) 
    (iter*)->writeEntryInfo(mdt_entry_infos[mdt_info_idx++]);

  ec_pdo_info_t at_pdo_info[] = {
    {0x0010, at_pdo_configs_.size(), at_entry_infos}
  };
  ec_pdo_info_t mdt_pdo_info[] = {
    {0x0018, mdt_pdo_configs_.size(), mdt_entry_infos}
  };

  ec_sync_info_t sync_infos[] = {
    {0, EC_DIR_OUTPUT}, // <syncManager idx="0" dir="out">
    {1, EC_DIR_INPUT}, // <syncManager idx="1" dir="in">
    // <syncManager idx="2" dir="out">
    {2, EC_DIR_OUTPUT, 1, mdt_pdo_info, EC_WD_ENABLE}, 
    // <syncManager idx="3" dir="in">
    {3, EC_DIR_INPUT, 1, at_pdo_info, EC_WD_ENABLE}, 
    {0xff}
  };

  if(ecrt_slave_config_pdos(slv_cfg_, EC_END, sync_infos)) {
    printf("Failed to config sync managers\n");
    return -1;
  }
  printf("Configured sync managers\n");

  for(PDOList::iterator iter = at_pdo_configs_.begin(); iter != at_pdo_configs_.end(); iter++) 
    if((iter*)->configureEntry(slv_cfg_, domain_) < 0)
      return -1;
  for(PDOList::iterator iter = mdt_pdo_configs_.begin(); iter != mdt_pdo_configs_.end(); iter++) 
    if((iter*)->configureEntry(slv_cfg_, domain_) < 0)
      return -1;
  printf("Configured PDO Entries\n");

  printf("Configuring telegrams...\n");
  uint16_t error_code;

  uint8_t* at_list_bytes;
  uint16_t at_list_len = createTelegramList(at_pdo_configs_, 0x001e, &at_list_bytes);
  if(ecrt_master_write_idn(master_, slave_info.position, 0, 0x0010, 
        at_list_bytes, at_list_len, &error_code)) {
    printf("Failed to write AT config (%d)\n", error_code);
    return -1;
  }

  uint8_t* mdt_list_bytes;
  uint16_t mdt_list_len = createTelegramList(mdt_pdo_configs_, 0x001e, &mdt_list_bytes);
  if(ecrt_master_write_idn(master_, slave_info.position, 0, 0x0010, 
        mdt_list_bytes, mdt_list_len, &error_code)) {
    printf("Failed to write MDT config (%d)\n", error_code);
    return -1;
  }

  for(uint16_t i=0; i<at_list_len; i+=2)
    printf("0x%x 0x%x\n", at_list_bytes[i], at_list_bytes[i+1]);
  for(uint16_t i=0; i<mdt_list_len; i+=2)
    printf("0x%x 0x%x\n", mdt_list_bytes[i], mdt_list_bytes[i+1]);
  printf("Telegrams configured.\n");

  printf("Configuring IDNs...\n");
  configureIDNs();
  printf("Done configuring IDNs.\n");

  ecrt_slave_config_dc(slv_cfg_, 0x0500, 1000000, 250000, 0, 0);
  //<master idx="0" appTimePeriod="1000000" refClockSyncCycles="1">
  //<dcConf assignActivate="500" sync0Cycle="*1" sync0Shift="250000"/>

  printf("Activating master...\n");
  if (ecrt_master_activate(master_)) {
    printf("Failed to activate master\n");
    return -1;
  }

  if (!(domain_pd_ = ecrt_domain_data(domain_))) {
    printf("Failed to get domain data\n");
    return -1;
  }
  for(PDOList::iterator iter = at_pdo_configs_.begin(); iter != at_pdo_configs_.end(); iter++) 
    (iter*)->registerDataAddress(domain_pd_);
  for(PDOList::iterator iter = mdt_pdo_configs_.begin(); iter != mdt_pdo_configs_.end(); iter++) 
    (iter*)->registerDataAddress(domain_pd_);

  pid_t pid = getpid();
  if (setpriority(PRIO_PROCESS, pid, -19))
    fprintf(stderr, "Warning: Failed to set priority: %s\n",
        strerror(errno));
}

void EthercatController::~EthercatController()
{
  ecrt_master_deactivate(master_);
}

void EthercatController::read()
{
  // receive process data
  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);

  // check process data state (optional)
  // check_domain1_state();

  // internally process the telegram data
  readTelegram(); 
}

void EthercatController::write()
{
  // internally process the telegram data
  writeTelegram();

  ecrt_domain_queue(domain_);
  ecrt_master_send(master_);
}
