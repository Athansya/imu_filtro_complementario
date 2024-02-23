/*
Título: Filtro complementario para calcular la pose.
Autor: Alfonso Toriz Vázquez^[1]
Descripción: Calcula la pose a partir de las lecturas del IMU de un sensor
Realsense D435i.
*/

#include <iostream>
#include <librealsense2/rs.hpp>
#include <mutex>
#include "complementary_filter.h"


int main()
{
    imu_tools::ComplementaryFilter CF;
    std::cout << "Hello, world!" << std::endl;
}