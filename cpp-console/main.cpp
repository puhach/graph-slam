



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

        // Initialize the world of size 8 x 10 with 12 landmarks.
        World world(8, 10, 2);

        
        // Show the landmarks.
        std::cout << world << std::endl;

        
        // Create the robot at position (3; 4) in the world.
        Robot robot(3, 4, 2, 1, 0.8, 0.8, world);

        
        // Simulate robot motions and measurements over 50 time steps.
        robot.moveAndSense(50);
        //auto [measurements, displacements] = robot.moveAndSense(50);
        //robot.moveAndSense(50);
        //auto [motions, measurements] = robot.getHistory();
        //std::cout << robot << std::endl;

        // Print actual robot positions.
        // In this implementation we assume that the initial position is known and the first displacement 
        // equals to the distance from the world origin.
        // TODO: try not to use the initial position info in the displacement history.
        std::cout << "Actual robot positions: " << std::endl;
        double x = 0, y = 0;
        //for (const auto& d : displacements)
        for (const auto &d : robot.getDisplacements())
        {
            x += d.first;
            y += d.second;

            std::cout << x << " " << y << std::endl;
        }

        /*
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