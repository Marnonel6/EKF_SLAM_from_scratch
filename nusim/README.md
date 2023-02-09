# Nusim
* Author: Marthinus Nel
* Published: Winter 2023
# Package description
The nusim package is a simulation and visualization tool for the turtlebot3 robots.
It uses rviz2 for visualization and provides a simulated environment. The package
creates stationary walls, obstacles and tracks the position of a robot. This class
publishes the current timestep of the simulation, obstacles and walls that appear
in Rviz as markers. The class has a timer_callback to continually update the
simulation at each timestep. The reset service resets the simulation to the initial
state thus restarting the simulation. A teleport service is available to teleport a
turtlebot to any pose. A broadcaster broadcasts the robots TF frames to a topic for
visualization in Rviz. The simulation operates in a loop, updating the state of the
world, publishing messages that provides state information simulating a real robot,
and processing service/subscriber callbacks for commands for the next time step. The
loop runs at a fixed frequency until termination.

# Launchfile description
- `nusim.launch.xml`:
    * The launch file launches a turtlebot3 by launching the launch file
      `nuturtle_description/launch/load_one.launch.py`.
    * The node `nusim` is also launched which is the main simulation node.
    * Rviz is launched to display the robots and the environment.
    * `ros2 launch nusim nusim.launch.xml`

# Parameters
* ```rate``` (int): Timer callback/simulation frequency [Hz]
* ```x0``` (float): Initial x coordinate of the robot [m]
* ```y0``` (float): Initial y coordinate of the robot [m]
* ```theta0``` (float): Initial theta angle of the robot [radians]
* ```obstacles.x``` (std::vector<double>): Vector of x coordinates for each obstacle [m]
* ```obstacles.y``` (std::vector<double>): Vector of y coordinates for each obstacle [m]
* ```obstacles.r``` (float): Radius of cylindrical obstacles [m]
* ```obstacles.h``` (float): Height of cylindrical obstacles [m]
* ```walls.x_lenght``` (float): Inner lenght of walls in x direction [m]
* ```walls.y_lenght``` (float): Inner lenght of walls in y direction [m]
* ```walls.h``` (float): Walls height [m]
* ```walls.w``` (float): Walls width [m]

# Example

![nusim1](https://user-images.githubusercontent.com/60977336/213889349-1d9f9921-aff3-4881-8608-860486bf1bc1.png)
