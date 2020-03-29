
//#include <format> // since C++20
#include <iterator>
#include <vector>
#include <random>
#include <exception>

#include "world.h"
#include "robot.h"



class World::RobotWrapper : public Robot
{
public:
    RobotWrapper(double sensorRange, double stepSize, double measurementNoise, double motionNoise, World& world
        , std::function<bool (World&, double dx, double dy)> move
        , std::function<Measurement (const World&) > senseLandmarks)
        : Robot(sensorRange, stepSize, measurementNoise, motionNoise, world, std::move(move), std::move(senseLandmarks)) {}
};	// RobotWrapper


/*
using LandmarkList = std::vector<std::tuple<int, int> >;


LandmarkList create_landmarks(int n, int worldHSize, int worldVSize)
{
    LandmarkList landmarks(n);

    // Seed with a real random value, if available
    std::random_device r;

    // Choose a random mean between 1 and 6
    //std::default_random_engine e1(r());
    std::mt19937 gen(r());
    std::uniform_int_distribution<int> uniform_dist;

    for (auto& [x, y] : landmarks)
    {
        x = uniform_dist(gen) % worldHSize;
        y = uniform_dist(gen) % worldVSize;
    }

    return landmarks;
}
*/



World::World(int width, int height, int nLandmarks)
    : width(width)
    , height(height)
    , landmarks(nLandmarks >= MinLandmarks && nLandmarks <= MaxLandmarks ? nLandmarks : throw std::invalid_argument("Invalid number of landmarks."))
    , robotX(-1)
    , robotY(-1)
{
    if (width < MinWorld || height < MinWorld)
        throw std::invalid_argument(std::string("The world is too small."));

    if (width > MaxWorld || height > MaxWorld)
        throw std::invalid_argument(std::string("The world is too large."));


    // Initialize the landmark positions from a uniform random distribution.

    //thread_local std::uniform_int_distribution<int> uniform_dist;
    std::uniform_int_distribution<int> randomX(0, width - 1), randomY(0, height - 1);

    for (auto& [x, y] : this->landmarks)
    {
        //x = uniform_dist(World::getRandomEngine()) % width;
        //y = uniform_dist(World::getRandomEngine()) % height;
        x = randomX(World::getRandomEngine());
        y = randomY(World::getRandomEngine());
    }

    /*
    enum class World : int
    {
        rows = 10,
        cols = 8
    };

    int nLandmarks = 12;
    std::vector<std::tuple<int, int> > landmarks = create_landmarks(nLandmarks, static_cast<int>(World::cols), static_cast<int>(World::rows));

    //std::copy(landmarks.begin(), landmarks.end(), std::ostream_iterator<std::tuple<int, int> >(std::cout, " "));
    for (const auto& [x, y] : landmarks)
        std::cout << "(" << x << ", " << y << ") ";
    std::cout << std::endl;
    */

}

// Define the destructor in the source file to fix the incomplete type error around unique_ptr<RobotWrapper>.
// https://stackoverflow.com/questions/9954518/stdunique-ptr-with-an-incomplete-type-wont-compile
World::~World() noexcept = default;


void World::getWorldSize(int& width, int& height) const noexcept
{
    width = this->width;
    height = this->height;
}



std::pair<double, double> World::getLandmark(int lkIndex) const
{
    // If lkIndex is not within the range of the landmarks vector, an exception of type std::out_of_range is thrown.
    return this->landmarks.at(lkIndex);
} // getLandmark

Robot& World::createRobot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise)
{
    RobotWrapper* ptr = this->robot.release();

    try
    {
        this->robotX = x >= 0 && x < this->width ? x : throw std::invalid_argument("Robot's X coordinate is out of the world.");
        this->robotY = y >= 0 && y < this->height ? y : throw std::invalid_argument("Robot's Y coordinate is out of the world.");
                
        if (ptr)
        {
            ptr->~RobotWrapper();
            new (ptr) RobotWrapper(sensorRange, stepSize, measurementNoise, motionNoise, *this
                , &World::moveRobot, &World::revealLandmarks);
        }
        else
        {
            ptr = new RobotWrapper(sensorRange, stepSize, measurementNoise, motionNoise, *this
                , &World::moveRobot, &World::revealLandmarks);
            this->robot.reset(ptr);
        }

        return *ptr;
    }
    catch (...)
    {
        ::operator delete(ptr);
        this->robotX = this->robotY = -1;
        throw;
    }
}   // createRobot

std::mt19937& World::getRandomEngine()
{
    // Seed with a real random value, if available. As long as we are not using 
    // threads thread_local could be replaced with static.
    thread_local std::mt19937 randomEngine(std::random_device{}()); 
    return randomEngine;
}

bool World::moveRobot(double dx, double dy)
{
    if (!this->robot)
        throw std::logic_error("Attempted to move a robot which doesn't exist.");
            
    std::uniform_real_distribution<double> motionDist(-this->robot->getMotionNoise(), this->robot->getMotionNoise());

    dx += motionDist(World::getRandomEngine());
    dy += motionDist(World::getRandomEngine());

    double newX = this->robotX + dx, newY = this->robotY + dy;

    if (newX < 0 || newX >= this->width || newY < 0 || newY >= this->height)
    	return false;
    
    this->robotX = newX;
    this->robotY = newY;
    
    return true;
}   // moveRobot

Measurement World::revealLandmarks() const
{
    if (!this->robot)
        throw std::logic_error("Attempted to sense landmarks while the robot doesn't exist.");

    Measurement measurement;
    std::uniform_real_distribution<double> measurementDist(-this->robot->getMeasurementNoise(), this->robot->getMeasurementNoise());
    
    //for (const auto& [lkx, lky] : this->landmarks)
    for (int i = 0; i < this->landmarks.size(); ++i)
    {
        const auto & [lkx, lky] = this->landmarks[i];

        // Get the distance to the landmark.
		double dx = lkx - this->robotX;
		double dy = lky - this->robotY;
				
        if (dx * dx + dy * dy <= this->robot->getSensorRange() * this->robot->getSensorRange())
        {
            // Distort the measurement.		
            dx += measurementDist(World::getRandomEngine());
            dy += measurementDist(World::getRandomEngine());
            //distortMeasurement(dx, dy);

            measurement.emplace_back(i, dx, dy);
        }
    }
    return measurement;
}   // revealLandmarks





std::ostream& operator << (std::ostream& ostream, const World& world)
{
    ostream << "Landmarks: [";

    //for (const auto& lk : world.landmarks)
    //    ostream << "(" << std::get<0>(lk) << "," << std::get<1>(lk) << "), ";

    auto it = world.landmarks.cbegin();

    if (it != world.landmarks.end())
    {
        ostream << "(" << it->first << ", " << it->second << ")";
        ++it;

        for (; it != world.landmarks.end(); ++it)
            ostream << ", (" << it->first << ", " << it->second << ")";

    }
        
    ostream << "]" << std::endl;


    //std::copy(world.landmarks.begin(), world.landmarks.end(), std::ostream_iterator<std::tuple<int, int> >(std::cout, ","));

    // not sure if we need to show the map
    /*for (int i = 0; i < world.height; ++i)
    {
        for (int j = 0; j < world.width; ++j)
        {

        }

        ostream << std::endl;
    }*/

    return ostream;
}
