/*
Título: Filtro complementario para calcular la pose.
Autor: Alfonso Toriz Vázquez^[1]
Descripción: Calcula la pose a partir de las lecturas del IMU de un sensor
Realsense D435i.
*/

#include "complementary_filter.h"
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>

// Frecuencias de muestreo
const int ACCEL_FPS = 250; // 63 o 250 Hz
const int GYRO_FPS = 400;  // 200 o 400 Hz

const double ACCEL_DT = 1.0 / ACCEL_FPS;

int main()
try
{
    std::cout << "Estimación de pose" << std::endl;

    // Filtro complementario
    imu_tools::ComplementaryFilter CF;

    // Realsense IMU config
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, ACCEL_FPS); // Acelerómetro
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, GYRO_FPS);   // Giroscopio
    cfg.enable_stream(RS2_STREAM_DEPTH);

    rs2::pipeline pipe;

    std::mutex mutex;

    // RS2 Callback
    auto profile = pipe.start(cfg, [&](rs2::frame frame) {
        std::lock_guard<std::mutex> lock(mutex);
        // Convertir frame a tipo IMU
        auto motion = frame.as<rs2::motion_frame>();

        // Gyroscopio
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            rs2_vector gyro_data = motion.get_motion_data();
            std::cout << "Giroscopio: (" << gyro_data.x << ", " << gyro_data.y << ", " << gyro_data.z << ")" << std::endl;
        }
        // Acelerómetro
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            rs2_vector accel_data = motion.get_motion_data();
            std::cout << "Acelerómetro: (" << accel_data.x << ", " << accel_data.y << ", " << accel_data.z << ")" << std::endl;
        }
    });
    
    while (true)
    {
        //
    }
    
    // Paramos el pipeline RS2
    pipe.stop();

    return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
    std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
              << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}