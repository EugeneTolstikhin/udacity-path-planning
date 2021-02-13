# Path planning project

Path planning project is about driving in the highway. The car should properly decide if it should change the lane or keep the lane with keeping the safe speed and distance to be able to proceed driving in the most effective way and without collision with any vehicle, as well as the car should not violate speed limit, sharply change acceleration and jerk values (*according to the requirements: The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3*)

The project contains the following configuration parameters:

```cpp
  constexpr float SIMULATOR_FREQUENCY = 1.0 / 50; //0.02
  constexpr float SPEED_LIMIT = 49.5; //mph
  constexpr float LANE_WIDTH = 4.0; //m
  constexpr size_t AMOUNT_OF_POINTS_FOREHAND = 3;
  constexpr size_t AMOUNT_OF_POINTS_WHOLE_PATH = 50;
  constexpr size_t TOO_CLOSE_POINTS = 30;
  constexpr float SPEED_CHANGE = 0.224; //0.224mph = 10m/s
  constexpr size_t AMOUNT_OF_LANES = 3;
```

The start up values:
```cpp
int lane = 1; // Central lane
```

The following FSM (Finite State Machine) is used:
* Keep lane until the car comes too close (The car ahead is 30 points ahead)
* When the car ahead is too close and no cars are too close both ahead and behind in the left lane -> change lane to the left
* When the car ahead is too close, there is at least 1 car is too close on the left lane either ahead or behind and no cars are too close both ahead and behind in the right lane -> change lane to the right
* When the car ahead is too close and both sides have the cars too close -> keep lane and decrease the speed -> increase the speed when the car ahead is far away

The following code shows the FSM:
```cpp
// FSM
if (too_close)
{
    std::cout << "Too close ahead" << std::endl;
    if (!too_close_left)
    {
        std::cout << "Turn left" << std::endl;
        --lane;
    }
    else if (!too_close_right)
    {
        std::cout << "Turn right" << std::endl;
        ++lane;
    }
    else
    {
        std::cout << "Keep lane and decrease speed" << std::endl;
        ref_vel -= SPEED_CHANGE;
    }
}
else if (ref_vel < SPEED_LIMIT)
{
    std::cout << "Far away ahead. Increase speed" << std::endl;
    ref_vel += SPEED_CHANGE;
}
```

To be able the car to move smoothly Spline is used taken from [here](http://kluge.in-chemnitz.de/opensource/spline/).