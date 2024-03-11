/*
Título: Filtro complementario para calcular la pose.
Autor: Alfonso Toriz Vázquez^[1]
Descripción: Calcula la pose a partir de las lecturas del IMU de un sensor
Realsense D435i.
*/

#include "complementary_filter.h"
#include <rplidar.h>
// #include <fstream>  // Descomentar lineas texto
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <numeric>
#include <thread>
// TODO: Convertir a vectores de Eigen

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar; // Vaya nombrecito

// Frecuencias de muestreo
const int ACCEL_FPS = 250; // 63 o 250 Hz
const int GYRO_FPS = 400;  // 200 o 400 Hz

// TODO: Calcular DT durante cada iteración
const double GYRO_DT = 1.0 / GYRO_FPS; // Solo para fines de testeo, se necesita calcular con motion.get_timestamp()

// Gravedad
const float GRAVITY = 9.78718;

// Histograma
const int MAX_ANGLE = 360;
const int HIST_SIZE = 1024;
const double BIN_WIDTH = (float)MAX_ANGLE / (float)HIST_SIZE;

int main(int argc, char *argv[])
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
    // IMU
    Eigen::Vector3d vectorAccel(0.0, 0.0, 0.0);
    Eigen::Vector3d vectorGravity(0.0, GRAVITY, 0.0); // Verificar dónde actúa la gravedad en el sensor, en 'y'!
    Eigen::Vector3d vectorAccelRel(0.0, 0.0, 0.0);

    // LIDAR
    Eigen::VectorXd vectorHistDistanceActual(HIST_SIZE);
    Eigen::VectorXd vectorHistDistancePrevious(HIST_SIZE);

    // Eigen::VectorXd vectorDistanceDifference(0);
    std::vector<double> vectorDistanceDifference;

    double distanceDifferenceSum = 0;
    float angleThreshold = 0.1;
    float distanceThreshold = 0.5;
    float standstillThreshold = 0.5;

    // Sensores
    std::cout << "Configurando sensores..." << std::endl;

    // RPLIDAR S1
    std::cout << "1) Estableciendo conexión al LIDAR..." << std::endl;

    RPlidarDriver *lidar = RPlidarDriver::CreateDriver();
    const char *devicePort = "/dev/ttyUSB0"; // or "COM3" for Windows
    u_result result = lidar->connect(devicePort, 256000);

    if (IS_FAIL(result))
    {
        std::cerr << "Fallo en la conexión al RPLIDAR S1" << std::endl;
        return 1;
    }

    if (lidar->isConnected())
        std::cout << "¡Conexión establecida al RPLIDAR S1!\n" << std::endl;

    lidar->startMotor();

    // Realsense IMU config
    std::cout << "2) Estableciendo conexión el IMU..." << std::endl;
    rs2::config cfg;

    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, ACCEL_FPS); // Acelerómetro
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, GYRO_FPS);   // Giroscopio
    cfg.enable_stream(RS2_STREAM_DEPTH);

    // TODO: Añadir manejo de errores en conexión con Realsense
    std::cout << "¡Conexión establecida al IMU!\n" << std::endl;

    rs2::pipeline pipe;

    // std::mutex accel_mutex;
    // std::mutex gyro_mutex;
    std::mutex filter_mutex;
    std::mutex lidar_mutex;

    // Procesamiento
    std::cout << "Configuración lista. Presione una tecla para continuar: ";
    std::cin.get();

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

    // Ciclo de procesamiento
    bool initPeriod = false;
    bool initLidar = false;

    const int bufferSize = 8192;
    rplidar_response_measurement_node_hq_t nodes[bufferSize];

    size_t nodeCount = _countof(nodes);
    bool force = true; // Necesario para activar sensor

    int iterations = 0;
    // while (iterations < 10)
    while (true)
    {
        iterations++;
        // Periodo de inicialización, evita NaNs.
        if (!initPeriod)
        {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            initPeriod = true;
        }

        // Sección IMU
        std::cout << "Procesamiento de datos obtenidos del IMU" << std::endl;
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
        std::cout << "Cuaternión puro de aceleración del sensor: \n"
                  << "(" << accel_data.x << "," << accel_data.y << "," << accel_data.z << ")" << std::endl;
        quaternionPureAccel.w() = 0;
        quaternionPureAccel.x() = accel_data.x;
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
        std::cout << "Vector aceleración:\n" << vectorAccel << std::endl;
        //
        // Cuaternión aceleración relativa
        vectorAccelRel = vectorAccel - vectorGravity;
        std::cout << "Vector aceleración relativa:\n" << vectorAccelRel << std::endl;
        //
        // G = [0 0 g]^T

        // Sección LIDAR
        std::cout << "Procesamiento de datos del LIDAR" << std::endl;

        // Muestrea una ronda de información
        lidar->startScanExpress(force, 1);                // 0 = Modo denso, 1 = modo simple
        result = lidar->grabScanDataHq(nodes, nodeCount); // Guarda datos
        // result = lidar->getScanDataWithIntervalHq(nodes, nodeCount); // Guarda datos, pero sale error!!
        if (IS_FAIL(result))
        {
            std::cerr << "Error en datos del LIDAR con código: " << result << std::endl;
            lidar->disconnect();
            delete lidar;
            return EXIT_FAILURE;
        }

        result = lidar->ascendScanData(nodes, nodeCount); // Ordena de menor a mayor
        if (IS_FAIL(result))
        {
            // TODO: AVERIGUAR POR QUE FALLA, CHECAR types.h EN CARPETA hal
            // DE LO CONTRARIO, IMPLEMENTAR FUNCION PARA EVALUAR FALLOS EN EL SENSOR.
            std::cerr << "Error en datos del LIDAR con código: " << result << std::endl;
            lidar->disconnect();
            delete lidar;
            return EXIT_FAILURE;
        }

        if (!initLidar)
        {
            for (int pos = 0; pos < (int)nodeCount; ++pos)
            {
                // std::lock_guard<std::mutex> lock(lidar_mutex);
                // Conversión de unidades
                float angleDegrees = static_cast<float>(nodes[pos].angle_z_q14) * 90.f / (1 << 14);
                float distanceMeters = static_cast<float>(nodes[pos].dist_mm_q2) / 10.f / (1 << 2);
                // std::cout << "Distancia obtenida " << pos << ": " << distanceMeters << std::endl;
                // std::cout << "Angulo obtenido " << pos << ": " << angleDegrees << std::endl;
                // vectorDistancePrevious[pos] = distanceMeters;
                // vectorAnglePrevious[pos] = angleDegrees;

                int binIndex = (int)ceil(angleDegrees / BIN_WIDTH) - 1;
                if (binIndex < 0)
                    binIndex++;
                // std::cout << "binIndex: " << binIndex << " ";
                vectorHistDistancePrevious[binIndex] = distanceMeters;
            }
            initLidar = true;
        }
        else
        {
            for (int pos = 0; pos < (int)nodeCount; ++pos)
            {
                // std::lock_guard<std::mutex> lock(lidar_mutex);
                // Conversión de unidades
                float angleDegrees = static_cast<float>(nodes[pos].angle_z_q14) * 90.f / (1 << 14);
                float distanceMeters = static_cast<float>(nodes[pos].dist_mm_q2) / 10.f / (1 << 2);
                // std::cout << "Distancia obtenida " << pos << ": " << distanceMeters << std::endl;
                // std::cout << "Angulo obtenido " << pos << ": " << angleDegrees << std::endl;
                // vectorDistanceActual[pos] = distanceMeters;
                // vectorAngleActual[pos] = angleDegrees;

                int binIndex = (int)ceil(angleDegrees / BIN_WIDTH) - 1;
                if (binIndex < 0)
                    binIndex++;
                // std::cout << "binIndex: " << binIndex << " ";
                vectorHistDistanceActual[binIndex] = distanceMeters;
            }

            // Compara para ver si calcularon la distancia en el mismo ángulo de rotación
            // std::cout << "Ángulos previos: \n" << vectorAnglePrevious.transpose() << std::endl;
            // std::cout << "Ángulos actuales: \n" << vectorAngleActual.transpose() << std::endl;

            int j = vectorHistDistanceActual.size();
            int jFaulty = 0;
            int jEpsilon = 0;

            distanceDifferenceSum = 0;

            // Recorremos arreglos comparando elementos entre ambos
            for (int i = 0; i < vectorHistDistanceActual.size(); i++)
            {
                // Valores t-1 y t
                int val_previous = vectorHistDistancePrevious[i];
                int val_actual = vectorHistDistanceActual[i];

                if (val_previous == 0)
                {
                    // Ambos son cero
                    if (val_actual == 0)
                        j--; // Disminuimos longitud de elementos del histograma.
                    else     // Uno es cero
                        jFaulty++;
                }
                else
                {
                    // Uno es cero
                    if (val_actual == 0)
                        jFaulty++;
                    else // Ambos son diferentes de cero -> Son comparables!
                    {
                        double difference = abs(val_previous - val_actual);
                        // Si la diferencia es menor al umbral, la contabilizamos
                        if (difference <= distanceThreshold)
                        {
                            vectorDistanceDifference.push_back(difference);
                            jEpsilon++;
                        }
                    }
                }
            }

            std::cout << "Tamaño del histograma: " << j << std::endl;
            std::cout << "Distancias dentro del umbral: " << jEpsilon << std::endl;
            std::cout << "Distancias con fallas: " << jFaulty << std::endl;

            // Calculamos diferencias
            distanceDifferenceSum =
                std::accumulate(vectorDistanceDifference.begin(), vectorDistanceDifference.end(), 0);

            std::cout << "Suma acumulada de diferencias: " << distanceDifferenceSum << std::endl;

            // Determinamos si hubo desplazamiento
            float m = (float)jEpsilon / ((float)j - (float)jFaulty);

            if (m > standstillThreshold)
            {
                std::cout << "Sistema en reposo     con m = " << m << "\n" << std::endl;
            }
            else
            {
                std::cout << "Sistema en movimiento con m = " << m << "\n" << std::endl;
            }

            vectorDistanceDifference.clear(); // Limpia

            // Actualiza histogramas Lidar t-1 <- t
            vectorHistDistancePrevious = vectorHistDistanceActual;
            vectorHistDistanceActual.setZero();
        }

        // vectorDistancePrevious.resize((int)nodeCount);  // Ajusta tamaño
        // vectorAnglePrevious.resize((int)nodeCount); // Ajusta tamaño

        // vectorDistanceActual.setZero();
        // vectorDistancePrevious.setZero();

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
    }

    // Cerramos archivo
    // outfile.close();

    // Paramos IMU
    pipe.stop();
    // Paramos LIDAR
    lidar->disconnect();
    delete lidar;

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