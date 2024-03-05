/*
Título: Filtro complementario para calcular la pose.
Autor: Alfonso Toriz Vázquez^[1]
Descripción: Calcula la pose a partir de las lecturas del IMU de un sensor
Realsense D435i.
*/

#include "complementary_filter.h"
// #include <fstream>  // Descomentar lineas texto
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <thread>
#include <vector>
// TODO: Convertir a vectores de Eigen

// Frecuencias de muestreo
const int ACCEL_FPS = 250; // 63 o 250 Hz
const int GYRO_FPS = 400;  // 200 o 400 Hz

// TODO: Calcular DT durante cada iteración
const double GYRO_DT = 1.0 / GYRO_FPS; // Solo para fines de testeo, se necesita calcular con motion.get_timestamp()

// Gravedad
const float GRAVITY = 9.78718;

int main()
try
{
    std::cout << "Estimación de pose" << std::endl;

    // Archivo de texto
    // std::ofstream outfile;
    // outfile.open("../valores_cuaternion.txt");
    // bool writeHeader = true;

    // Filtro complementario
    imu_tools::ComplementaryFilter CF;
    rs2_vector gyro_data, accel_data;
    // bool firstIteration = true;
    // double dt_init = 0.0;

    // Cuaternión resultante
    double q0 = 0;
    double q1 = 0;
    double q2 = 0;
    double q3 = 0;
    double q0_inv = 0;
    double q1_inv = 0;
    double q2_inv = 0;
    double q3_inv = 0;

    // Cuaterniones
    Eigen::Quaternion<double> quaternionRotCF;
    Eigen::Quaternion<double> quaternionRotCFConj;
    Eigen::Quaternion<double> quaternionAccel;
    Eigen::Quaternion<double> quaternionPureAccel;
    Eigen::Quaternion<double> quaternionAccelRel;

    // Vectores
    Eigen::Vector3d vectorAccel(0.0, 0.0, 0.0);
    Eigen::Vector3d vectorGravity(0.0, 0.0, GRAVITY); // Verificar dónde actúa la gravedad en el sensor

    // Realsense IMU config
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, ACCEL_FPS); // Acelerómetro
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, GYRO_FPS);   // Giroscopio
    cfg.enable_stream(RS2_STREAM_DEPTH);

    rs2::pipeline pipe;

    std::mutex accel_mutex;
    std::mutex gyro_mutex;
    std::mutex filter_mutex;

    // RS2 Callback
    auto profile = pipe.start(cfg, [&](rs2::frame frame) {
        // Convertir frame a tipo IMU
        auto motion = frame.as<rs2::motion_frame>();

        // Giroscopio
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // double ts = motion.get_timestamp();  // Unix time

            // std::lock_guard<std::mutex> lock(gyro_mutex);
            gyro_data = motion.get_motion_data();
            // std::cout << "Giroscopio: (" << gyro_data.x << ", " << gyro_data.y << ", " << gyro_data.z << ")"
            //   << std::endl;
        }
        // Acelerómetro
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            // std::lock_guard<std::mutex> lock(accel_mutex);
            accel_data = motion.get_motion_data();
            // std::cout << "Acelerómetro: (" << accel_data.x << ", " << accel_data.y << ", " << accel_data.z << ")"
            //   << std::endl;
        }
    });

    int iteration = 0;
    while (iteration < 5)
    // while (true)
    {
        // TODO: Arreglar tiempo inicialización del sensor
        std::this_thread::sleep_for(std::chrono::seconds(5));
        // Actualización del filtro complementario
        std::lock_guard<std::mutex> lock(filter_mutex);
        CF.update(accel_data.x, accel_data.y, accel_data.z, gyro_data.x, gyro_data.y, gyro_data.z, GYRO_DT);
// 
        // Cuaternión resultante
        CF.getOrientation(q1, q1, q2, q3);
        quaternionRotCF.w() = q0;
        quaternionRotCF.x() = q1;
        quaternionRotCF.y() = q2;
        quaternionRotCF.z() = q3;
// 
        // std::cout << "Cuaternión v1: (" << q0 << ", " << q1 << ", " << q2 << ", " << q3 << ")" << std::endl;
        // std::cout << "Cuaternión v2: \n(" << quaternionRotCF.coeffs() << ")" << std::endl;
        std::cout << std::fixed << std::setprecision(6) << "Cuaternión de rotación: \n(" << quaternionRotCF << ")"
                  << std::endl;
// 
        // Calculamos conjugado
        quaternionRotCFConj = quaternionRotCF.conjugate();
// 
        std::cout << "Cuaternión conjugado de rotación: \n" << quaternionRotCFConj << std::endl;
// 
        // Cuaternión puro aceleración
        std::cout << "Aceleración sensor: \n" << "(" << accel_data.x << "," << accel_data.y << "," << accel_data.z
        << ")" << std::endl; quaternionPureAccel.w() = 0; quaternionPureAccel.x() = accel_data.x;
        quaternionPureAccel.y() = accel_data.y;
        quaternionPureAccel.z() = accel_data.z;
// 
        std::cout << "Cuaternión puro de aceleración:\n" << quaternionPureAccel << std::endl;
// 
        // Vector aceleración
        // quaternionAccel = quaternionRotCF X quaternionPureAccel X quaternionRotCFConj
        quaternionAccel = quaternionRotCF * quaternionPureAccel * quaternionRotCFConj;
        vectorAccel << quaternionAccel.x(), quaternionAccel.y(), quaternionAccel.z();
// 
        std::cout << "Vector aceleración:\n" << vectorAccel << "\n" << std::endl;
// 
        // Cuaternión aceleración relativa
        // quaternionAccelRel = quaternionAccel - G
        // quaternionAccelRel = quaternionAccel - gravityVector;
        // G = [0 0 g]^T

        // Guardamos valores cuaternion
        // TODO: Revisar por qué está guardando más de
        // 400,000 valores en tan poco tiempo!
        // if (outfile.is_open())
        // {
        // if (writeHeader)
        // {
        // outfile << "q0,q1,q2,q3\n";
        // writeHeader = false;
        // continue;
        // }

        // outfile << q0 << "," << q1 << "," << q2 << "," << q3 << "\n";
        // }

        // Calculamos posición

        iteration++;
    }

    // Cerramos archivo
    // outfile.close();

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