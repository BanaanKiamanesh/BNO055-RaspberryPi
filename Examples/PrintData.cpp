/*
    This Code Prints the Data Coming from the Sensor
*/

#include "BNO055.h"
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>

int main()
{
    std::cout << "Init..." << std::endl;

    /*
    ````` BNO055 IMU Initialization `````
    */
    BNO055 imu;
    imu.update();

    /*
    ````` Main Loop `````
    */
    while (true)
    {
        system("clear");
        imu.print_state();
        delay(100);
    }

    return 0;
}