



#include "world.h"
#include "robot.h"

#include <iostream>
//#include <Eigen/Dense>

//using namespace std;
//using namespace Eigen;


int main()
{
    try
    {
        // TODO: enter the world size, the number of landmarks, and the number of time steps

        // initialize the world of size 8 x 10 with 12 landmarks
        World world(8, 10, 2);

        
        // show the landmarks
        std::cout << world << std::endl;

        
        // create the robot at position (3; 4) in the world
        //Robot& robot = world.getRobot();
        Robot robot(3, 4, world);

        /*
        // simulate robot motions and measurements over 50 time steps
        auto [measurements, displacements] = robot.moveAndSense(50);
        //robot.moveAndSense(50);
        //auto [motions, measurements] = robot.getHistory();
        //std::cout << robot << std::endl;

        // TODO: print actual robot positions

        // run SLAM to estimate positions of the robot and landmarks
        auto [estPositions, estLandmarks] = robot.localize();
        //auto [mux, muy] = slam(data);

        // TODO: print estimated positions and landmarks
        */
    }
    catch (const std::exception & e)
    {
        std::cout << e.what() << std::endl;
    }
}