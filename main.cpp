#include <iostream>
#include <yaml-cpp/yaml.h>
#include <hal.h>
#include <rtapi.h>
#include <ecrt.h>
#include <cstring>
#include <map>
#include <csignal>
#include <vector>
#include <iomanip>
#include <stdint.h>

// Structure to store data of different types of HAL variables
struct HALVariableData {
    hal_bit_t* bit;
    hal_u32_t* u32;
    hal_s32_t* s32;
    hal_float_t* flt;

    HALVariableData() : bit(nullptr), u32(nullptr), s32(nullptr), flt(nullptr) {}
};

// Structure to store SDO configuration and its associated HAL variable
struct SDOConfig {
    uint16_t index;
    uint8_t subIndex;
    size_t bitLen;
    std::string halType;
    std::string readWrite;
    std::string halPin;
    HALVariableData hal_var_data;
    uint16_t alias;
    uint16_t position;
    size_t bitOffset; // Offset for a specific bit in a complex structure
    float scale;      // Scale
    float offset;     // Offset

    SDOConfig() : scale(1.0f), offset(0.0f) {} // Default initialization
};

static int comp_id = -1;
static ec_master_t* master = nullptr;
static std::vector<SDOConfig> sdo_configs;
static std::vector<ec_slave_info_t> slave_infos;
static bool running = true;

void signalHandler(int signum) {
    running = false;
}

void initialize_slave_info(ec_master_t* master, ec_master_info_t& master_info) {
    slave_infos.resize(master_info.slave_count);
    for (unsigned int i = 0; i < master_info.slave_count; ++i) {
        if (ecrt_master_get_slave(master, i, &slave_infos[i]) == 0) {
            std::cout << "Slave " << i << ": Vendor ID: 0x" << std::hex << std::setw(8) << std::setfill('0') << slave_infos[i].vendor_id
                      << ", Product Code: 0x" << std::hex << std::setw(8) << std::setfill('0') << slave_infos[i].product_code
                      << ", Position: " << std::dec << slave_infos[i].position << std::endl;
        } else {
            std::cerr << "Failed to get info for slave " << i << std::endl;
        }
    }
}

// Function to read SDO data using the configuration
int read_sdo(uint16_t position, uint16_t index, uint8_t subindex, uint8_t* target, size_t size) {
    uint32_t abort_code;
    size_t result_size;

    std::cout << "Starting SDO upload for Index=0x" << std::hex << index << ", SubIndex=0x" << std::hex << size << std::endl;

    int ret = ecrt_master_sdo_upload(master, position, index, subindex, target, size, &result_size, &abort_code);

    if (ret < 0) {
        std::cerr << "SDO upload failed for SDO 0x" << std::hex << index << ":0x" << std::hex << static_cast<int>(subindex)
                  << ". Abort code: 0x" << std::hex << abort_code << std::endl;
        return -1;
    }

    // Display result information
    std::cout << "SDO upload successful for Index=0x" << std::hex << index << ", SubIndex=0x"
              << std::hex << static_cast<int>(subindex) << ". Data size: " << std::dec << result_size << " bytes." << std::endl;

    // Display data values
    std::cout << "SDO Data (hex):";
    for (size_t i = 0; i < result_size; ++i) {
        std::cout << " " << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(target[i]);
    }
    std::cout << std::endl;

    return 0;
}

// SDO configuration based on YAML
void configure_sdos_from_yaml(const YAML::Node& yaml_slave_config, uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code) {
    std::cout << "Starting to configure SDOs for slave at alias " << alias << ", position " << position << std::endl;

    if (yaml_slave_config["sdo"]) {
        for (const auto& sdo : yaml_slave_config["sdo"]) {
            try {
                uint16_t index = std::stoi(sdo["index"].as<std::string>(), nullptr, 16);
                uint8_t subIndex = std::stoi(sdo["subIndex"].as<std::string>(), nullptr, 16);
                size_t bitLen = sdo["bitLen"].as<size_t>();
                std::string halType = sdo["halType"].as<std::string>();
                std::string readWrite = sdo["readWrite"].as<std::string>();

                std::cout << "Processing SDO: Index=0x" << std::hex << std::setw(4) << std::setfill('0') << index
                          << ", SubIndex=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(subIndex)
                          << ", bitLen=" << std::dec << bitLen
                          << ", halType=" << halType
                          << ", readWrite=" << readWrite << std::endl;

                if (halType == "complex" && sdo["complexEntries"]) {
                    size_t bitOffset = 0;

                    for (const auto& entry : sdo["complexEntries"]) {
                        size_t entryBitLen = entry["bitLen"].as<size_t>();

                        if (entry["halTyp"]) {
                            std::string entryHalTyp = entry["halTyp"].as<std::string>();

                            if (entryHalTyp == "bit") {
                                std::string halPin = entry["halPin"].as<std::string>();
                                hal_bit_t** hal_var = (hal_bit_t**)hal_malloc(sizeof(hal_bit_t*));
                                if (hal_pin_bit_newf(HAL_OUT, hal_var, comp_id, "%s", halPin.c_str()) != 0) {
                                    std::cerr << "Failed to create HAL bit pin: " << halPin << std::endl;
                                } else {
                                    SDOConfig config;
                                    config.index = index;
                                    config.subIndex = subIndex;
                                    config.bitLen = bitLen; //entryBitLen;
                                    config.halType = entryHalTyp;
                                    config.readWrite = readWrite;
                                    config.alias = alias;
                                    config.position = position;
                                    config.halPin = halPin;
                                    config.hal_var_data.bit = *hal_var;
                                    config.bitOffset = bitOffset;

                                    sdo_configs.push_back(config);
                                    std::cout << "HAL bit pin created successfully: " << halPin << ", bitOffset=" << bitOffset << std::endl;
                                }
                            }
                        } else {
                            std::cerr << "Invalid or missing halType key in complex entry for SDO 0x" 
                                      << std::hex << std::setw(4) << std::setfill('0') << index 
                                      << ":0x" << std::setw(2) << std::setfill('0') << static_cast<int>(subIndex) << std::endl;
                        }

                        bitOffset += entryBitLen;
                    }
                } else if (halType == "U32") {
                    hal_u32_t** hal_var = (hal_u32_t**)hal_malloc(sizeof(hal_u32_t*));
                    std::string halPin = sdo["name"].as<std::string>();

                    if (hal_pin_u32_newf(HAL_OUT, hal_var, comp_id, "%s", halPin.c_str()) != 0) {
                        std::cerr << "Failed to create HAL U32 pin: " << halPin << std::endl;
                    } else {
                        SDOConfig config;
                        config.index = index;
                        config.subIndex = subIndex;
                        config.bitLen = bitLen;
                        config.halType = halType;
                        config.readWrite = readWrite;
                        config.alias = alias;
                        config.position = position;
                        config.halPin = halPin;
                        config.hal_var_data.u32 = *hal_var;

                        sdo_configs.push_back(config);
                        std::cout << "HAL U32 pin created successfully: " << halPin << std::endl;
                    }
                } else if (halType == "S32") {
                    hal_s32_t** hal_var = (hal_s32_t**)hal_malloc(sizeof(hal_s32_t*));
                    std::string halPin = sdo["name"].as<std::string>();

                    if (hal_pin_s32_newf(HAL_OUT, hal_var, comp_id, "%s", halPin.c_str()) != 0) {
                        std::cerr << "Failed to create HAL S32 pin: " << halPin << std::endl;
                    } else {
                        SDOConfig config;
                        config.index = index;
                        config.subIndex = subIndex;
                        config.bitLen = bitLen;
                        config.halType = halType;
                        config.readWrite = readWrite;
                        config.alias = alias;
                        config.position = position;
                        config.halPin = halPin;
                        config.hal_var_data.s32 = *hal_var;

                        sdo_configs.push_back(config);
                        std::cout << "HAL S32 pin created successfully: " << halPin << std::endl;
                    }
                } else if (halType == "float") {
                    hal_float_t** hal_var = (hal_float_t**)hal_malloc(sizeof(hal_float_t*));
                    std::string halPin = sdo["name"].as<std::string>();

                    if (hal_pin_float_newf(HAL_OUT, hal_var, comp_id, "%s", halPin.c_str()) != 0) {
                        std::cerr << "Failed to create HAL float pin: " << halPin << std::endl;
                    } else {
                        SDOConfig config;
                        config.index = index;
                        config.subIndex = subIndex;
                        config.bitLen = bitLen;
                        config.halType = halType;
                        config.readWrite = readWrite;
                        config.alias = alias;
                        config.position = position;
                        config.halPin = halPin;
                        config.hal_var_data.flt = *hal_var;

                        // Parsing scale and offset for float type
                        if (sdo["scale"]) {
                            config.scale = sdo["scale"].as<float>();
                        }
                        if (sdo["offset"]) {
                            config.offset = sdo["offset"].as<float>();
                        }

                        sdo_configs.push_back(config);
                        std::cout << "HAL float pin created successfully: " << halPin << std::endl;
                    }
                } else {
                    std::cerr << "Unknown HAL type: " << halType << " for SDO 0x" << std::hex << index << ":0x" << subIndex << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error parsing YAML for SDO: " << e.what() << std::endl;
            }
        }
    } else {
        std::cerr << "No SDOs found in YAML configuration." << std::endl;
    }
}

// Function to swap bytes in a 32-bit word (for byte order conversion)
uint32_t swap_uint32(uint32_t val) {
    return (val << 24) | ((val << 8) & 0x00FF0000) | ((val >> 8) & 0x0000FF00) | (val >> 24);
}

// In the update_hal_variables function:
void update_hal_variables() {
    // Group SDO configurations by index and subindex
    std::map<std::pair<uint16_t, uint8_t>, std::vector<SDOConfig>> sdo_groups;
    for (const auto& config : sdo_configs) {
        sdo_groups[{config.index, config.subIndex}].push_back(config);
    }

    // Iterate through each group and make one SDO request
    for (const auto& group : sdo_groups) {
        const auto& configs = group.second;
        if (configs.empty()) continue;

        // Determine the size of data to be read based on the configuration
        size_t data_size = (configs[0].bitLen + 7) / 8;  // Round up to get a whole number of bytes
        uint8_t* data = new uint8_t[data_size];
        memset(data, 0, data_size);

        // Make one SDO request
        const auto& first_config = configs[0];
        std::cout << "Updating HAL variables for SDO 0x" << std::hex << std::setw(4) << std::setfill('0') << first_config.index
                  << ":0x" << std::setw(2) << std::setfill('0') << static_cast<int>(first_config.subIndex) << std::endl;

        if (read_sdo(first_config.position, first_config.index, first_config.subIndex, data, data_size) == 0) {
            // Distribute the data across the variables
            for (const auto& config : configs) {
                if (config.halType == "bit") {
                    size_t byteIndex = config.bitOffset / 8;
                    size_t bitIndex = config.bitOffset % 8;
                    uint32_t value = (data[byteIndex] >> bitIndex) & 0x01;
                    *(config.hal_var_data.bit) = (hal_bit_t)value;
                    std::cout << "Updated HAL bit variable: " << *(config.hal_var_data.bit) << " for pin " << config.halPin << std::endl;
                } else if (config.halType == "U32") {
                    if (config.bitLen == 16) {
                        uint16_t raw_value = *(uint16_t*)data;
                        uint32_t extended_value = static_cast<uint32_t>(raw_value);
                        *(config.hal_var_data.u32) = extended_value;
                        std::cout << "Updated HAL U32 variable with extended value: " << std::hex << extended_value << " for pin " << config.halPin << std::endl;
                    } else {
                        uint32_t raw_value = *(uint32_t*)data;
                        *(config.hal_var_data.u32) = raw_value;
                        std::cout << "Updated HAL U32 variable: " << std::hex << raw_value << " for pin " << config.halPin << std::endl;
                    }
                } else if (config.halType == "S32") {
                    if (config.bitLen == 16) {
                        // If the SDO represents a 16-bit signed value
                        int16_t raw_value = *(int16_t*)data;
                        int32_t extended_value = static_cast<int32_t>(raw_value);  // Extend to 32 bits while preserving the sign
                        *(config.hal_var_data.s32) = extended_value;
                        std::cout << "Updated HAL S32 variable with extended value: " << std::hex << extended_value << " for pin " << config.halPin << std::endl;
                    } else {
                        // If the SDO represents a 32-bit value
                        int32_t raw_value = *(int32_t*)data;
                        *(config.hal_var_data.s32) = raw_value;
                        std::cout << "Updated HAL S32 variable: " << std::hex << raw_value << " for pin " << config.halPin << std::endl;
                    }
                } else  if (config.halType == "float") {
                    float value = 0.0f;

                    if (config.bitLen == 16) {
                        // If the data size is 16 bits
                        uint16_t raw_value = *(uint16_t*)data;

                        // Convert 16-bit value to float
                        value = static_cast<float>(raw_value);
                    } else if (config.bitLen == 32) {
                        // If the data size is 32 bits
                        uint32_t raw_value = *(uint32_t*)data;
                        raw_value = swap_uint32(raw_value); // Swap bytes for correct order
                        value = *(float*)&raw_value; // Convert 32-bit value to float
                    }

                    // Apply scaling and offset
                    value = value * config.scale + config.offset;

                    // Store the result in the HAL variable
                    *(config.hal_var_data.flt) = value;
                    std::cout << "Updated HAL float variable: " << value << " for pin " << config.halPin << std::endl;
                }
            }
        }

        delete[] data;
    }
}


int main(int argc, char* argv[]) {
    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Check if the path to the YAML file is provided as a command line argument
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_yaml_config>" << std::endl;
        return -1;
    }

    // Get the path to the YAML file
    std::string yaml_file_path = argv[1];

    // Initialize HAL component
    comp_id = hal_init("lcecsdo");
    if (comp_id < 0) {
        std::cerr << "HAL initialization failed with error " << comp_id << std::endl;
        return comp_id;
    }

    // Request EtherCAT master
    master = ecrt_request_master(0);
    if (!master) {
        std::cerr << "Error requesting EtherCAT master.\n";
        return -1;
    }

    // Get master information
    ec_master_info_t master_info;
    if (ecrt_master(master, &master_info) != 0) {
        std::cerr << "Failed to retrieve master info.\n";
        return -1;
    }

    std::cout << "Number of slaves detected: " << master_info.slave_count << std::endl;

    // Initialize slave information
    initialize_slave_info(master, master_info);

    try {
        // Load configuration from YAML file
        YAML::Node config = YAML::LoadFile(yaml_file_path);
        const auto& slave_configs = config["master"]["slaves"];
        if (slave_configs && slave_configs.size() > 0) {
            for (unsigned int i = 0; i < master_info.slave_count; ++i) {
                for (const auto& slave_config : slave_configs) {
                    uint32_t yaml_vid = std::stoul(slave_config["vid"].as<std::string>(), nullptr, 16);
                    uint32_t yaml_pid = std::stoul(slave_config["pid"].as<std::string>(), nullptr, 16);

                    if (yaml_vid == slave_infos[i].vendor_id && yaml_pid == slave_infos[i].product_code) {
                        configure_sdos_from_yaml(slave_config, 0, slave_infos[i].position, slave_infos[i].vendor_id, slave_infos[i].product_code);
                        break;
                    }
                }
            }
        } else {
            std::cerr << "No slaves found in the YAML configuration file.\n";
        }
    } catch (const YAML::Exception& e) {
        std::cerr << "Error parsing YAML: " << e.what() << std::endl;
        return -1;
    }

    hal_ready(comp_id);
    std::cout << "HAL component is ready.\n";
    
    int count = 0;
    while (running) {
        std::cerr << "count :" <<  count++ << std::endl;
        update_hal_variables();
        usleep(1000000); // Pause for cyclic updates
    }

    hal_exit(comp_id);
    ecrt_release_master(master);

    return 0;
}
