/*
 *  Title: RPlidar example code
 *  Author: Alfonso Toriz V.
 *  
 *  Description: Shows how to use the RPlidar SDK for the S1 Lidar sensor by SLAMTECH.
 *  Work in progress...
 *
 *
 *  typedef struct sl_lidar_response_measurement_node_hq_t
 *  {
 *      sl_u16   angle_z_q14;  // Fix-point angle description in z representation
 *      sl_u32   dist_mm_q2;   // Distance in millimiter of fixed point values
 *      sl_u8    quality;      // Measurement quality (0 \sim 255)
 *      sl_u8    flag;         // Flags, current only one bit used: SP_LIDAR_RESP_MEASUREMENT_SYNCBIT
 *  } __attribute__((packed)) sl_lidar_response_measurement_node_hq_t;
 *
 *  Common conversions:
 *  float angle_in_degrees = node.angle_z_q14 * 90.f / (1 << 14);
 *  float distance_in_meters = node.dist_mm_q2 / 1000.f / (1 << 2);
 *
 *  2. Try to set motor speed
 *  3. Read and comment both examples in the sdk
 */

#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <rplidar.h> // include the RPLidar SDK

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar; // RPLidar 

void writeCSV(const std::string& filename, const rplidar_response_measurement_node_hq_t nodes[8192], size_t nodeCount);

int main(int argc, char* argv[])
{
    RPlidarDriver* lidar = RPlidarDriver::CreateDriver();
    // Connect to the RPLidar S1
    const char * devicePort = "/dev/ttyUSB0"; // or "COM3" for Windows
    u_result result = lidar->connect(devicePort, 256000);

    if (IS_FAIL(result))
    {
        std::cerr << "Failed to connect to RPLidar S1." << std::endl;
        return 1;
    }

    if (lidar->isConnected())
        std::cout << "Connection established!\n" << std::endl;

    
    // Sensor health and specs
    rplidar_response_device_health_t healthInfo;
    result = lidar->getHealth(healthInfo);
    
    std::cout << "***** Health *****" << std::endl;
    std::cout << "Status: " << static_cast<int>(healthInfo.status) << std::endl;
    std::cout << "Error code: " << healthInfo.error_code << std::endl;

    rplidar_response_device_info_t deviceInfo;
    result = lidar->getDeviceInfo(deviceInfo);

    std::cout << "\n***** Sensor information *****" << std::endl;
    std::cout << "Model: " << deviceInfo.model << std::endl;
    std::cout << "Firmware Version: " << deviceInfo.firmware_version << std::endl;
    std::cout << "Hardware Version: " << static_cast<int>(deviceInfo.hardware_version) << "\n" << std::endl;

    // Get and show supported scan modes
    std::vector<RplidarScanMode> scanModes;
    lidar->getAllSupportedScanModes(scanModes);

    std::cout << "***** Supported scan modes: " << scanModes.size() << " *****" << std::endl;
    // List scan modes
    for (auto it = scanModes.begin(); it != scanModes.end(); ++it)
    {
            std::cout << "Mode name: " << it[0].scan_mode << std::endl;
            std::cout << "Sampling duration: " << it[0].us_per_sample << " microseconds per sample" << std::endl;
            std::cout << "Max distance: " << it[0].max_distance << std::endl;
            std::cout << "Ans Type: " << it[0].ans_type << std::endl;
            std::cout << std::endl;

    }

    // Check if motor control is enabled for this model
    bool support;
    result = lidar->checkMotorCtrlSupport(support);
    if (support)
    {
        std::cout << "Motor control is supported for this model!" << std::endl;
    }
    else
    {
        std::cout << "Motor control is not supported for this model!" << std::endl;
    }
    
    // Typical Scan Mode
    // I think it indicates the chosen mode number from the previous list
    _u16 outMode;  
    lidar->getTypicalScanMode(outMode);
    std::cout << "\n***** Typical Scan Mode: " << scanModes[outMode-1].scan_mode << " *****" << std::endl;
    
    // Let's start the motor
    std::cout << "\nStarting the motor..." << std::endl;
    lidar->startMotor();  // Not needed for S1 model, may add as safety net

    // Start scan with dense mode
    bool force = true;
    bool useTypicalScan = true;
    std::cout << "Starting dense scan mode" << std::endl;

    // lidar->startScan(force, useTypicalScan);
    lidar->startScanExpress(force, 1);
    
    // Wait and grab a complete 0-360 degree scan data previously received
    const int bufferSize =  8192; 
    rplidar_response_measurement_node_hq_t nodes[bufferSize];  // Buffer, 8192 is used in the examples
    size_t nodeCount = _countof(nodes);
    std::cout << "Saving scanned data..." << std::endl;
    result = lidar->grabScanDataHq(nodes, nodeCount); // Saves data

    if (IS_FAIL(result))
    {
        std::cerr << "Failed to get scan data" << std::endl;
        lidar->disconnect();
        delete lidar;  // Equal to RPlidarDriver::DisposeDriver();
        return 1;
    }
    std::cout << "Scanned data saved!" << std::endl;
    
    // Writing file
    std::cout << "Writing CSV file with data..." << std::endl;
    std::string filename = "lidar_data_converted.csv";
    //std::string filename = "lidar_data_original.csv";
    writeCSV(filename, nodes, nodeCount);

    // Get frequency
    float frequency;
    lidar->getFrequency(scanModes[1], nodeCount, frequency);
    std::cout << "\nFrequency: " << frequency << " kHz" << std::endl;  // kHz
    
    std::cout << "\nEnding default scan mode" << std::endl;

    std::cout << "Stopping motor...\n" << std::endl;
    lidar->stopMotor();

    std::cout << "\nProgram finished, closing connection!" <<std::endl;
    lidar->disconnect(); // Frees memory
    //RPlidarDriver::DisposeDriver(lidar); //Should be used to free the memory, but ends up in segmentation fault

    delete lidar;  // Equal to RPlidarDriver::DisposeDriver();

    return 0;
}

void writeCSV(const std::string& filename, const rplidar_response_measurement_node_hq_t nodes[8192], size_t nodeCount)
{
    std::ofstream file(filename);
    if (file.is_open())
    {
        // Write header row
        file << "Angle,Distance,Quality\n";

        // Write data rows
        for (int pos = 0; pos < (int)nodeCount; ++pos)
        {
            // Unit conversions
            float angle_in_degrees = static_cast<float>(nodes[pos].angle_z_q14) * 90.f / (1 << 14);
            float distance_in_meters = static_cast<float>(nodes[pos].dist_mm_q2) / 1000.f / (1 << 2);
            //float angle_in_degrees = static_cast<float>(nodes[pos].angle_z_q14);
            //float distance_in_meters = static_cast<float>(nodes[pos].dist_mm_q2);

            file << angle_in_degrees << "," << distance_in_meters << "," << static_cast<int>(nodes[pos].quality) << "\n";
        }

        file.close();
        std::cout << "CSV file " << filename << " written successfully!" << std::endl;
    }
    else
    {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }
}
