



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
        World world(8, 10, 3);

        
        // Show the landmarks.
        std::cout << world << std::endl;

        
        // Create the robot at position (3; 4) in the world.
        //Robot robot(3, 4, 20, 1, 0.008, 0.008, world);
        Robot robot(3, 4, 20, 1, 0.08, 0.08, world);

        
        // Simulate robot motions and measurements over 50 time steps.
        robot.moveAndSense(15);
        //auto [measurements, displacements] = robot.moveAndSense(50);
        //robot.moveAndSense(50);
        //auto [motions, measurements] = robot.getHistory();
        //std::cout << robot << std::endl;

        // Print actual robot positions.
        // In this implementation we assume that the initial position is known and the first displacement 
        // equals to the distance from the world origin.
        // TODO: try not to use the initial position info in the displacement history.
        std::cout << "Actual robot positions:" << std::endl;
        double x = 0, y = 0;
        //for (const auto& d : displacements)
        for (const auto &d : robot.getDisplacements())
        {
            x += d.first;
            y += d.second;

            std::cout << x << " " << y << std::endl;
        }

        
        // Run SLAM to estimate positions of the robot and landmarks.

        auto [estPositions, estLandmarks] = robot.localize();
        

        // Print estimated positions and landmarks.

        std::cout << "\nEstimated robot positions:" << std::endl;
        for (const auto& [x, y] : estPositions)
            std::cout << x << " " << y << std::endl;
        

        std::cout << "\nEstimated landmark locations:" << std::endl;
        for (const auto& [x, y] : estLandmarks)
            std::cout << x << " " << y << std::endl;

    }   // try
    catch (const std::exception & e)
    {
        std::cout << e.what() << std::endl;
    }
}