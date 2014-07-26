
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

/****************************************************************************/

extern "C"
{
    #include "ecrt.h"
}

/****************************************************************************/

#define FREQUENCY 1000
#define VELCTRL

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain = NULL;
static ec_domain_state_t domain1_state = {};

/****************************************************************************/

// process data
static uint8_t *domain_pd = NULL;

// offsets for PDO entries
static int off_dig_out;
static int off_counter_in;
static int off_counter_out;

static int drivecontrol_off;
static int poscommand_off;
static int velcommand_off;
static int drivestatus_off;
static int posfeedback_off;
static int velfeedback_off;
// static int diagmsg_off;

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

    int configureEntry(ec_slave_config_t *sc, ec_domain_t *domain)
    {
        data_offset = ecrt_slave_config_reg_pdo_entry(sc, idn_bin, 0x00, domain, NULL);
        if (data_offset < 0) {
            printf("Failed to configure %s\n", name.c_str());
            return -1;
        }
        return 0;
    }

    void registerDataAddress(uint8_t *domain_pd)
    {
        data_address = domain_pd + data_offset;
    }

    void writeEntryInfo(ec_pdo_entry_info_t& entry_info)
    {
        entry_info.index = idn_bin;
        entry_info.subindex = subindex;
        entry_info.bit_length = bit_len;
    }
};

typedef std::map<std::string, PDOConfiguration> PDOMap;
PDOMap at_pdo_configs;
PDOMap mdt_pdo_configs;

#define PERIOD_NS 1000000
#define CLOCK_TO_USE CLOCK_REALTIME
#define NSEC_PER_SEC (1000000000L)
#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
const struct timespec cycletime = {0, PERIOD_NS};
static unsigned int counter = 0;
static unsigned int inner_counter = 0;
static unsigned int freq_counter = 0;

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
        PDOMap& pdo_configs, uint16_t max_len, uint8_t** idn_bytes)
{
    uint16_t len = 0;
    for(PDOMap::iterator iter = pdo_configs.begin(); iter != pdo_configs.end(); iter++) 
        if(iter->second.in_telegram_config)
            len++;

    (*idn_bytes) = new uint8_t[4+2*len];
    uint16_t cur_byte = 0;
    (*idn_bytes)[cur_byte++] = LOWBYTE16(2*len);
    (*idn_bytes)[cur_byte++] = HIGHBYTE16(2*len);
    (*idn_bytes)[cur_byte++] = LOWBYTE16(max_len);
    (*idn_bytes)[cur_byte++] = HIGHBYTE16(max_len);
    for(PDOMap::iterator iter = pdo_configs.begin(); iter != pdo_configs.end(); iter++) {
        if(iter->second.in_telegram_config) {
            (*idn_bytes)[cur_byte++] = LOWBYTE16(iter->second.idn_bin);
            (*idn_bytes)[cur_byte++] = HIGHBYTE16(iter->second.idn_bin);
        }
    }
    return cur_byte;
}
/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
	struct timespec result;

	if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
		result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
	} else {
		result.tv_sec = time1.tv_sec + time2.tv_sec;
		result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
	}

	return result;
}

/*****************************************************************************/

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

	if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}

/****************************************************************************/

static uint64_t tick = 0;

/****************************************************************************/
void cyclic_task()
{
    struct timespec wakeupTime, time;
    int32_t orig_pos;
    clock_gettime(CLOCK_TO_USE, &wakeupTime);
    while(1) {
		wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

        // receive process data
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        // check process data state (optional)
        check_domain1_state();

        int32_t cur_pos = EC_READ_S32(domain_pd + posfeedback_off);
        int32_t cur_vel = EC_READ_S32(domain_pd + velfeedback_off);
        // uint32_t cur_diag_msg = EC_READ_U32(domain_pd + diagmsg_off);
		if (counter) {
			counter--;
		} else { // do this at 1 Hz
			counter = FREQUENCY;

            printf("Iteration %2d\n", inner_counter);

            // if(cur_pos != 0) {
            //     EC_WRITE_S32(domain_pd + poscommand_off, cur_pos);
            // }

			// check for master state (optional)
			check_master_state();
            printf("Drive status: 0x%x\n", 
                   EC_READ_U16(domain_pd + drivestatus_off));
            printf("Position: %d\n", cur_pos);
            printf("Velocity: %d\n", cur_vel);
            // printf("Diagnostic msg: %d\n", cur_diag_msg);
            printf("Drive control: 0x%x\n", 
                   EC_READ_U16(domain_pd + drivecontrol_off));
            printf("Position command: %d\n", 
                   EC_READ_S32(domain_pd + poscommand_off));
            printf("Velocity command: %d\n", 
                   EC_READ_S32(domain_pd + velcommand_off));

            //if(inner_counter == 6)
            //    EC_WRITE_U16(domain_pd + drivecontrol_off, 0xE400);

            inner_counter++;
        }
        if(inner_counter == 7) {
            orig_pos = cur_pos;
        }
        if(inner_counter < 7) {
            if(freq_counter == 0) {
                EC_WRITE_U16(domain_pd + drivecontrol_off, 0x0000);
                freq_counter = 1;
            }
            else {
                EC_WRITE_U16(domain_pd + drivecontrol_off, 0x0400);
                freq_counter = 0;
            }
        }
        if(inner_counter >= 10) {
            if(freq_counter == 0) {
                EC_WRITE_U16(domain_pd + drivecontrol_off, 0xE000);
                freq_counter = 1;
            }
            else {
                EC_WRITE_U16(domain_pd + drivecontrol_off, 0xE400);
                freq_counter = 0;
            }
        }
        if(inner_counter >= 10 && freq_counter == 1) {
            EC_WRITE_S32(domain_pd + velcommand_off, 0);
            EC_WRITE_S32(domain_pd + poscommand_off, orig_pos);
        }
        double frequency = 0.2;
        double vel_amplitude = -100000;
        double tick_sec = tick*0.002;
        double ang_freq = 2.0*M_PI*frequency;
        double pos_amplitude = 1.0*-vel_amplitude/ang_freq;
        if(inner_counter >= 15 && freq_counter == 1) {
            // EC_WRITE_S32(domain_pd + velcommand_off, -1000000*sin(160*0.001*tick*3.14/180));
            EC_WRITE_S32(domain_pd + poscommand_off, pos_amplitude*cos(tick_sec*ang_freq)+orig_pos-pos_amplitude);
            EC_WRITE_S32(domain_pd + velcommand_off, vel_amplitude*sin(tick_sec*ang_freq));
            // EC_WRITE_S32(domain_pd + poscommand_off, orig_pos+1000);
            tick++;
        }
        // if(inner_counter >= 18 && freq_counter == 1) {
        //     EC_WRITE_S32(domain_pd + velcommand_off, 0);
        //     // EC_WRITE_S32(domain_pd + poscommand_off, orig_pos);
        // }
        // if(inner_counter >= 10 && freq_counter == 1) {
        //     // EC_WRITE_S32(domain_pd + poscommand_off, orig_pos-500*(inner_counter-9));
        //     EC_WRITE_S32(domain_pd + poscommand_off, orig_pos-500*(FREQUENCY*(inner_counter-10)+(FREQUENCY-counter+1)));
        // }

		// write application time to master
		//clock_gettime(CLOCK_TO_USE, &time);
		//ecrt_master_application_time(master, TIMESPEC2NS(time));

        ecrt_domain_queue(domain);
        ecrt_master_send(master);
    }
}

int main(int argc, char **argv)
{
    uint64_t u;
    ec_slave_config_t *sc;

#if 0
    uint16_t hex = idn_to_hex("S-0-0135");
    printf("hex 0x%x, Bytes: low 0x%x, high 0x%x\n", hex, LOWBYTE16(hex), HIGHBYTE16(hex));
    std::vector<std::string> idn_list;
    idn_list.push_back("S-0-0040");
    idn_list.push_back("S-0-0051");
    uint8_t* idn_list_bytes;
    uint16_t idn_list_data_len = idn_list_to_bytes(idn_list, 0x001e, &idn_list_bytes);
    for(uint16_t i=0; i<idn_list_data_len; i+=2)
        printf("0x%x 0x%x\n", idn_list_bytes[i], idn_list_bytes[i+1]);
    uint8_t blah[2] = LENDIAN(0x0007);
    printf("0x%x 0x%x\n", blah[0], blah[1]);
    return 0;
#endif
    /*
	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
		perror("mlockall failed");
		return -1;
	}
    */
    // initialize signal handling
    // exitEvent = eventfd(0, 0);
    // if (exitEvent == -1) {
    //   fprintf(stderr, "%s: ERROR: unable to create exit event\n", modname);
    // }
    // signal(SIGINT, exitHandler);
    // signal(SIGTERM, exitHandler);

    // at_pdo_configs["status"] = PDOConfiguration("Drive status", "S-0-0135", 0, 16, false);
    // at_pdo_configs["pos_fb"] = PDOConfiguration("Position feedback", "S-0-0051", 0, 32, true);
    // at_pdo_configs["vel_fb"] = PDOConfiguration("Velocity feedback", "S-0-0040", 0, 32, true);

    // mdt_pdo_configs["ctrl"] = PDOConfiguration("Device control", "S-0-0134", 0, 16, false);
    // mdt_pdo_configs["pos_cmd"] = PDOConfiguration("Position command", "S-0-0047", 0, 32, true);
    // mdt_pdo_configs["vel_cmd"] = PDOConfiguration("Velocity command", "S-0-0036", 0, 32, true);

#ifdef VELCTRL
    uint8_t vel_mode_data[2] = LENDIAN(0x0002);
#else
    uint8_t vel_mode_data[2] = LENDIAN(0x000B);
#endif

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain = ecrt_master_create_domain(master);
    if (!domain) {
        printf("Domain not created\n");
        return -1;
    }
    printf("Domain created\n");

    ec_slave_info_t slave_info;
    if(ecrt_master_get_slave(master, 0, &slave_info)) {
        printf("Couldn't get slave info\n");
        return -1;
    }
    printf("Got slave info for: 0x%x, 0x%x\n", 
           slave_info.vendor_id, slave_info.product_code);

    if (!(sc = ecrt_master_slave_config(master, 0, 0, 
                    slave_info.vendor_id, slave_info.product_code))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
    printf("Initialized slave config.\n", 
           slave_info.vendor_id, slave_info.product_code);

    ecrt_slave_config_watchdog(sc, 2498, 1000);
    
    printf("Configuring sync managers...\n");
    ec_pdo_entry_info_t* at_entry_infos = new ec_pdo_entry_info_t[at_pdo_configs.size()];
    uint16_t at_info_idx = 0;
    for(PDOMap::iterator iter = at_pdo_configs.begin(); iter != at_pdo_configs.end(); iter++) 
        iter->second.writeEntryInfo(at_entry_infos[at_info_idx++]);
    uint16_t mdt_info_idx = 0;
    ec_pdo_entry_info_t* mdt_entry_infos = new ec_pdo_entry_info_t[mdt_pdo_configs.size()];
    for(PDOMap::iterator iter = mdt_pdo_configs.begin(); iter != mdt_pdo_configs.end(); iter++) 
        iter->second.writeEntryInfo(mdt_entry_infos[mdt_info_idx++]);

    ec_pdo_info_t at_pdo_info[] = {
        {0x0010, at_pdo_configs.size(), at_entry_infos}
    };
    ec_pdo_info_t mdt_pdo_info[] = {
        {0x0018, mdt_pdo_configs.size(), mdt_entry_infos}
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
    
    if(ecrt_slave_config_pdos(sc, EC_END, sync_infos)) {
        printf("Failed to config sync managers\n");
        return -1;
    }
    printf("Configured sync managers\n");

    for(PDOMap::iterator iter = at_pdo_configs.begin(); iter != at_pdo_configs.end(); iter++) 
        if(iter->second.configureEntry(sc, domain) < 0)
            return -1;
    for(PDOMap::iterator iter = mdt_pdo_configs.begin(); iter != mdt_pdo_configs.end(); iter++) 
        if(iter->second.configureEntry(sc, domain) < 0)
            return -1;
    printf("Configured PDO Entries\n");

    printf("Configuring telegrams...\n");
    uint16_t error_code;

    uint8_t* at_list_bytes;
    uint16_t at_list_len = createTelegramList(at_pdo_configs, 0x001e, &at_list_bytes);
    if(ecrt_master_write_idn(master, slave_info.position, 0, 0x0010, 
                             at_list_bytes, at_list_len, &error_code)) {
        printf("Failed to write AT config (%d)\n", error_code);
        return -1;
    }

    uint8_t* mdt_list_bytes;
    uint16_t mdt_list_len = createTelegramList(mdt_pdo_configs, 0x001e, &mdt_list_bytes);
    if(ecrt_master_write_idn(master, slave_info.position, 0, 0x0010, 
                             mdt_list_bytes, mdt_list_len, &error_code)) {
        printf("Failed to write MDT config (%d)\n", error_code);
        return -1;
    }

    for(uint16_t i=0; i<at_list_len; i+=2)
        printf("0x%x 0x%x\n", at_list_bytes[i], at_list_bytes[i+1]);
    for(uint16_t i=0; i<mdt_list_len; i+=2)
        printf("0x%x 0x%x\n", mdt_list_bytes[i], mdt_list_bytes[i+1]);
    printf("Telegrams configured.\n");

    uint8_t telegram_type_data[2] = LENDIAN(0x0007);
    if(ecrt_master_write_idn(master, slave_info.position, 0, 
                             idn_to_hex("S-0-0015"), telegram_type_data, 2, NULL)) {
        printf("Failed to write telegram type\n");
        return -1;
    }
    printf("Using configurable telegram.\n");

    // Primary operation mode
    if(ecrt_slave_config_idn(sc, 0, idn_to_hex("S-0-0032"), EC_AL_STATE_SAFEOP, vel_mode_data, 2)) {
        printf("Failed to write mode\n");
        return -1;
    }

	ecrt_slave_config_dc(sc, 0x0500, 1000000, 250000, 0, 0);
        //<master idx="0" appTimePeriod="1000000" refClockSyncCycles="1">
        //<dcConf assignActivate="500" sync0Cycle="*1" sync0Shift="250000"/>

    printf("Activating master...\n");
    if (ecrt_master_activate(master)) {
        printf("Failed to activate master\n");
        return -1;
    }

    if (!(domain_pd = ecrt_domain_data(domain))) {
        printf("Failed to get domain data\n");
        return -1;
    }
    for(PDOMap::iterator iter = at_pdo_configs.begin(); iter != at_pdo_configs.end(); iter++) 
        iter->second.registerDataAddress(domain_pd);
    for(PDOMap::iterator iter = mdt_pdo_configs.begin(); iter != mdt_pdo_configs.end(); iter++) 
        iter->second.registerDataAddress(domain_pd);

    pid_t pid = getpid();
    if (setpriority(PRIO_PROCESS, pid, -19))
        fprintf(stderr, "Warning: Failed to set priority: %s\n",
                strerror(errno));

    //while(1);
    cyclic_task();

    ecrt_master_deactivate(master);
    return 0;
    // wait for SIGTERM
    // if (read(exitEvent, &u, sizeof(uint64_t)) < 0) {
    //   fprintf(stderr, "%s: ERROR: error reading exit event\n", modname);
    // }
}

/****************************************************************************/
