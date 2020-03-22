
//#include <format> // since C++20
#include <iterator>
#include <vector>
#include <random>
#include <exception>

#include "world.h"
#include "robot.h"


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

std::mt19937& World::getRandomEngine()
{
    // Seed with a real random value, if available. As long as we are not using 
    // threads thread_local could be replaced with static.
    thread_local std::mt19937 randomEngine(std::random_device{}()); 
    return randomEngine;
}





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
