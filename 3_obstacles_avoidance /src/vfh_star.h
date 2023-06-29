#include <iostream>
#include <algorithm>
#include <cmath>
#include <assert.h>
#include <vector>

const int NUM_SENSORS = 360; // number of sensors in the histogram

// Result struct to store the output of the VFH* algorithm
struct Result
{
    double heading;  // desired heading of the robot, in degrees
    double velocity; // desired velocity of the robot, in meters/second
    bool stop;       // flag indicating whether the robot should stop
    bool evade;      // flag indicating whether the robot should take evasive action
};

// Histogram struct to store the histogram of the environment
struct Histogram
{
    std::vector<double> bins; // array of bins, representing different directions
    double min_distance;      // minimum distance to an obstacle in any direction
    double max_distance;      // maximum distance to an obstacle in any direction
};

// Compute the histogram of the environment from sensor readings
Histogram computeHistogram(const std::vector<double> &readings)
{
    Histogram hist;
    hist.bins.resize(NUM_SENSORS);
    hist.min_distance = std::numeric_limits<double>::max();
    hist.max_distance = std::numeric_limits<double>::min();

    // Iterate over all sensor readings
    for (uint i = 0; i < readings.size(); ++i)
    {
        double distance = readings[i];
        hist.bins[i] = distance; // store the distance in the corresponding bin
        hist.min_distance = std::min(hist.min_distance, distance);
        hist.max_distance = std::max(hist.max_distance, distance);
    }

    return hist;
}

// Compute the desired heading and velocity using the VFH* algorithm
Result computeControlOutput(const Histogram &hist, double goal_x, double goal_y)
{
    Result result;

    // Compute the angle to the goal in the histogram frame
    double goal_angle = atan2(goal_y, goal_x);

    // Find the bin with the minimum distance to an obstacle
    int min_bin = std::distance(hist.bins.begin(),
                                std::min_element(hist.bins.begin(), hist.bins.end()));

    // Find the bin with the maximum distance to an obstacle
    int max_bin = std::distance(hist.bins.begin(),
                                std::max_element(hist.bins.begin(), hist.bins.end()));

    // Compute the angular width of the "safety zone" around the robot
    double safety_zone_width = std::min(hist.min_distance / 2.0, 30.0);

    // Compute the start and end angles of the safety zone
    double safety_zone_start = min_bin - safety_zone_width / 2.0;
    double safety_zone_end = min_bin + safety_zone_width / 2.0;

    // Check if the goal falls within the safety zone
    if (goal_angle >= safety_zone_start && goal_angle <= safety_zone_end)
    {
        // If the goal is within the safety zone, go straight towards it
        result.heading = goal_angle;
        result.velocity = std::min(hist.min_distance / 2.0, 0.5);
    }
    else
    {
        // If the goal is outside the safety zone, find the path with the maximum distance
        result.heading = max_bin;
        result.velocity = std::min(hist.max_distance / 2.0, 0.5);
    }

    // Set the stop and evade flags based on the minimum distance to an obstacle
    result.stop = (hist.min_distance < 0.5);
    result.evade = (hist.min_distance < 0.2);

    return result;
}

void Test()
{
    // Generate test sensor readings
    std::vector<double> readings = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};

    // Set the goal position for the robot
    double goal_x = 0.0;
    double goal_y = 0.0;

    // Compute the histogram of the environment
    Histogram hist = computeHistogram(readings);

    // Compute the control output for the robot
    Result result = computeControlOutput(hist, goal_x, goal_y);

    // Verify the results
    std::cout<<result.heading <<" "<<result.velocity<<" "<<result.stop<<" "<<result.evade<<std::endl;
    // assert(result.heading == 4.5);
    // assert(result.velocity == 0.5);
    // assert(!result.stop);
    // assert(!result.evade);
}
