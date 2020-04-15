



#include "world.h"
#include "robot.h"
#include "graphslam.h"
#include "genetic.h"

#include <iostream>


int main()
{
    try
    {
        // TODO: enter the world size, the number of landmarks, and the number of time steps

        // Initialize the world of size 8 x 10 with 12 landmarks.
        World world(80, 100, 30);

        
        // Show the landmarks.
        std::cout << world << std::endl;

        
        // Create the robot at position (3; 4) in the world.
        double rx0 = 3, ry0 = 4;
        Robot& robot = world.createRobot(rx0, ry0, 200, 1, 0.008, 0.008);

        
        // Simulate robot motions and measurements over 50 time steps.
        
        constexpr int timesteps = 15;

        std::cout << "Actual robot positions:" << std::endl << rx0 << " " << ry0 << std::endl;

        // TEST!
        std::vector<std::pair<double,double>> actualRPosV;
        actualRPosV.emplace_back(rx0, ry0);

        robot.sense(); // record zero displacement and landmark distances

        for (int t = 1; t <= timesteps; ++t)
        {
            robot.roamAndSense();

            std::cout << world.getRobotX() << " " << world.getRobotY() << std::endl;

            actualRPosV.emplace_back(world.getRobotX(), world.getRobotY());
        }

                
        // Run SLAM to estimate positions of the robot and landmarks.

        //auto [estPositions, estLandmarks] = robot.localize(GraphSlam(rx0, ry0, static_cast<int>(world.getLandmarkNum())));
        //auto [estPositions, estLandmarks] = robot.localize(Genetic(world));
        Genetic g(world);
        g.giveRPositions(actualRPosV);
        auto [estPositions, estLandmarks] = robot.localize(g);
        

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