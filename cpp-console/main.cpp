#include <iostream>
#include <iterator>
#include <vector>
#include <random>

//#include <Eigen/Dense>

//using namespace std;
//using namespace Eigen;

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

int main()
{
    enum class World: int
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
}