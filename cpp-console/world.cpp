
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

    
    // Seed with a real random value, if available
    std::random_device r;

    // Choose a random mean between 1 and 6
    //std::default_random_engine e1(r());
    std::mt19937 gen(r());
    std::uniform_int_distribution<int> uniform_dist;

    for (auto& [x, y] : this->landmarks)
    {
        x = uniform_dist(gen) % width;
        y = uniform_dist(gen) % height;
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

