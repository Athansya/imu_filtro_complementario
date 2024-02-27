/*
 * File: main.cpp
 * Project: imu_datos_calibracion
 * File Created: Tuesday, 6th February 2024 12:56:24 pm
 * Author: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx)
 * -----
 * Last Modified: Tuesday, 6th February 2024 12:56:26 pm
 * Modified By: Alfonso Toriz Vazquez (atoriz98@comunidad.unam.mx>)
 * -----
 * License: MIT License
 * -----
 * Description: Lectura de datos RealSense D435i para calibración.
 * Se leen datos de aceleración y giros del IMU. Se sigue la metodología
 * expuesta por Tedaldi et al. 2014. "A Robust and Easy to Implement
 * Method for IMU Calibration without External Equipments". Los datos
 * se almacenan en un archivo de texto.
 */

// TODO: Calibrar IMU siguiendo pasos de Tedaldi i.e. posiciones y tiempos del IMU

#include <chrono>
#include <fstream>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <map>
#include <thread>
//#include <vector>

// Frecuencias de muestreo
const int ACCEL_FPS = 250;  // 63 or 250 Hz
const int GYRO_FPS = 400;  // 200 or 400 Hz

const double ACCEL_DT = 1.0 / ACCEL_FPS;  // Utilizaré el acelerómetro como base
//const double GYRO_DT = 1.0 / GYRO_FPS;

// Variables de calibración de Tedaldi
const float T_INIT = 50.0;  // Tiempo en reposo inicial (seg)
const float T_WAIT = 8.0;  // Tiempo en reposo de cada N posición (seg)
const int N = 40;  // Número de posiciones diferentes

int main(int argc, char * argv[]) try
{
    // Vectores
    std::vector<double> ax;
    std::vector<double> ay;
    std::vector<double> az;

    std::vector<double> gx;
    std::vector<double> gy;
    std::vector<double> gz;
    //
    // Crea un archivo de texto para los datos del acelerómetro.
    // Guarda el tiempo y la aceleracion en x, y, z
    std::ofstream accel_data_file;
    accel_data_file.open("d435i_datos_accel.txt");

    // Crea un archivo de texto para los datos del giroscopio.
    // Guarda el tiempo y giros en x, y, z
    std::ofstream gyro_data_file;
    gyro_data_file.open("d435i_datos_gyro.txt");


    // Banderas
    bool running = true;
    bool t_init_done = false;
    bool first_iteration_accel = true;
    bool first_iteration_gyro = true;
    // Tiempos de los sensores
    float dt = 0.0;
    double time_accel_init = 0.0;
    double time_gyro_init = 0.0;
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    // Definimos contadores para los frames
    std::map<int, int> counters;
    std::map<int, std::string> stream_names;
    std::mutex mutex;  // Protegerá el contador de ser accesado por múltiples hilos.

    // Callback de frames. Método preferido para trabajar con altas frecuencias
    // Utiliza un hilo en específico para el sensor. Se utilizan bloqueos para gestionar
    // modificaciones a memoria compartida.
    //
    // Aqui irá todo el código para extraer los datos
    auto callback = [&](const rs2::frame& frame)
    {
        //std::cout << frame.get_profile().stream_name() << " at " << frame.get_profile().fps() << " fps." << std::endl;
        std::lock_guard<std::mutex> lock(mutex);
        
        // Los flujos que no esten tan sincronizados, como el IMU, producen frames individuales
        counters[frame.get_profile().unique_id()]++;

        auto motion = frame.as<rs2::motion_frame>();
        // Checar si es acelerometro o giroscopio
        if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            double ts_gyro = 0.0;
            if (first_iteration_gyro)
            {
                time_gyro_init = motion.get_timestamp();
                ts_gyro = time_gyro_init;
                first_iteration_gyro = false;
            }
            else
            {
                ts_gyro = (motion.get_timestamp() - time_gyro_init) / 1000.0;
            }

            rs2_vector gyro_data = motion.get_motion_data();


            //std::cout << std::fixed << "Timestamp gyro: " << ts_gyro << std::endl;
            //std::cout << "Gyro: " << gyro_data.x << "\t" << gyro_data.y << "\t" << gyro_data.z << std::endl;
            gyro_data_file << ts_gyro << " " << gyro_data.x << " " << gyro_data.y << " " << gyro_data.z << "\n";
        }

        if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL &&
            motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
        {
            double ts_accel = 0.0;
            if (first_iteration_accel)
            {
                time_accel_init = motion.get_timestamp();
                ts_accel = time_accel_init;
                first_iteration_accel = false;
            }
            else
            {
                ts_accel = (motion.get_timestamp() - time_accel_init) / 1000.0;
            }

            rs2_vector accel_data = motion.get_motion_data();

            //std::cout << "Timestamp accel: " << ts_accel << std::endl;
            //std::cout << "Accel: " << accel_data.x << "\t" << accel_data.y << "\t" << accel_data.z << std::endl;
            accel_data_file << ts_accel << " " << accel_data.x << " " << accel_data.y << " " << accel_data.z << "\n";
        }


    };


    // Configuración 
    rs2::config cfg;
    // Habilitamos el stream del IMU
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, ACCEL_FPS);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, GYRO_FPS);

    // Desactivamos los streams que no se van a usar 
    cfg.disable_stream(RS2_STREAM_COLOR);
    cfg.disable_stream(RS2_STREAM_INFRARED);

    // Fix para obtener las frecuencias de muestreo correctas del IMU
    // Se debe habilitar el stream de profundidad.
    // Ref: https://github.com/IntelRealSense/librealsense/issues/6737
    cfg.enable_stream(RS2_STREAM_DEPTH);

    // Inicializamos el flujo de trabajo que contiene los sensores
    rs2::pipeline pipe;
    // Inicializamos el dispositivo

    //for (auto p: profiles.get_streams())
    //    stream_names[p.unique_id()] = p.stream_name();


    // GUI
    std::cout << "Iniciando calibración..." << std::endl;
    // Ciclo principal, lectura y guardado de datos

    //auto init_time = std::chrono::high_resolution_clock::now();
    //std::cout << std::chrono::high_resolution_clock::period::den << std::endl;
    //std::cout << "Tiempo inicial: " << std::chrono::high_resolution_clock::to_time_t(init_time) << std::endl;
    //std::getchar();
    // T_INIT
    std::cout << "Mantenga el sensor en reposo por " << T_INIT << " segundos..." << std::endl;
    std::cout << "Presione una tecla para continuar..." << std::endl;
    std::getchar();  // Recibe input cualquiera

    //while ((T_INIT - dt) > 0)
    // Contador Chrono inicial

    auto start_time = std::chrono::high_resolution_clock::now();
    pipe.start(cfg, callback); 
    while (true)
    {
        //std::this_thread::sleep_for(std::chrono::seconds(1));

        // Calculamos tiempo
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
        //std::cout << "Diff milliseg: " << duration_ms.count() << std::endl;
                                                                                                    
        if (duration_ms.count() >= T_INIT)
            //running = false;
            break;
    }

    std::cout << "Iniciando con las N posiciones..." << std::endl;
    // N Posiciones
    for (int i = 0; i < N; i++)
    {
        std::cout << "Posición " << i << std::endl;
        std::cout << "Coloque el sensor en una posición estable durante " << T_WAIT << " segundos." << std::endl;
        //std::cout << "Presione una tecla para continuar..." << std::endl;
        //std::getchar();  // Recibe input cualquiera

        //std::this_thread::sleep_for(std::chrono::seconds(1));
        //running = true;
        start_time = std::chrono::high_resolution_clock::now();
        while (true)
        {

            // Calculamos tiempo
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration_ms = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
            //std::cout << "Diff milliseg: " << duration_ms.count() << std::endl;
                                                                                                        
            if (duration_ms.count() >= T_WAIT)
                //running = false;
                break;
        }
    }

//    while (running)
//    {
//        rs2::frameset frameset = pipe.wait_for_frames();
//
//        // Lee información del IMU
//        rs2::motion_frame accel_frame = frameset.first_or_default(RS2_STREAM_ACCEL);
//
//        rs2::motion_frame gyro_frame = frameset.first_or_default(RS2_STREAM_GYRO);
//        // Imprime los FPS del acelerómetro
//        //std::cout << "Accel FPS: " << accel_frame.get_profile().fps() << std::endl;
//        
//        // Imprime los FPS del acelerómetro
//        //std::cout << "Gyro FPS: " << gyro_frame.get_profile().fps() << std::endl;
//
//
//        // Checamos que sean datos válidos
//        if (accel_frame && gyro_frame)
//        {
//
//            // Extraemos datos del IMU
//            rs2_vector accel_sample = accel_frame.get_motion_data();
//            rs2_vector gyro_sample = gyro_frame.get_motion_data();
//
//            // Mostramos los datos
//            std::cout << "Tiempo: " << dt << std::endl;
//
//            std::cout << "Accel: " << accel_sample.x << ", " << accel_sample.y << ", " << accel_sample.z << std::endl;
//            std::cout << "Gyro: " << gyro_sample.x << ", " << gyro_sample.y << ", " << gyro_sample.z << "\n" << std::endl;
//
//            // Escribimos los datos en sus respectivos archivos
//            accel_data_file << dt << " " << accel_sample.x << " " << accel_sample.y << " " << accel_sample.z << "\n";
//            gyro_data_file << dt << " " << gyro_sample.x << " " << gyro_sample.y << " " << gyro_sample.z << "\n";
//        }
//
//        // Incrementamos los contadores
//        // Nos basamos en la frecuencia de muestreo del acelerómetro.
//        // En un futuro habría que tomar en cuenta la diferencia que
//        // hay con la del giroscopio a la hora de fusionar la información
//        // y llevar un dt para cada uno.
//        dt += ACCEL_DT;
//
//        // TODO: Manejar los errores
//        //if (!(accel_frame && gyro_frame))
//        //{
//        //    std::cerr << "accel_frame o gyro_frame no tienen datos validos.";
//        //}
//    }

    // Calibración terminado
    pipe.stop();
    std::cout << "Calibración terminada" << std::endl;
    // Cerrar archivos
    accel_data_file.close();
    gyro_data_file.close();

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "Realsense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n   " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

