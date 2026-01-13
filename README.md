# Fuel and Hub Simulation for the 2026 FRC Rebuilt Game

A 3D physics simulation of Fuel and the Hub for the 2026 FRC game Rebuilt.

# Installation

1. Install Python

2. Create a Python virtual environment (optional, but recommended)

3. Install the necessary python libraries with the following command

    `pip install -r requirements.txt`

# Usage

An example can be seen in `experiment_example.py`. The usage is also outlined below.

## Simulation Object

The simulation object can be created with the command below. The `experiment_name` is used to create a folder to output the data into. If multiple runs of an experiment are performed they will be output into different directories (e.g. `experiment1`, `experiment2`, etc.). The `show_gui` parameter controls if the 3D simulation is actually displayed or not.

`sim = RebuiltHubSimuation(experiment_name="example", show_gui=True)`

## Adding Fuel

There are two functions for adding fuel to the simulation. Fuel should be added to the simulation before it is run. 

### Adding a Single Fuel
An example of adding a single single fuel is shown below. Parameter descriptions are as follows. All units are SI units.

`starting_time (float)`: The time (seconds from start of simulation) at which the fuel should be spawned into the simulation and launched. This is adjustable so all the fuel does not spawn in at once and interfere with eachother. However, if multiple fuel has the same `starting_time` they will be spawned at the same time, allowing for testing of potential interactions.

`starting_distance (float)`: The distance away from the hub that the fuel should be spawned in at in meters. We use polar coordinates to define the starting location of the fuel.

`starting_angle (float)`: The angle in polar coordinates relative to the hub that fuel should be spawned in at in radians.

`target_yaw (float)`: The direction that the fuel should be launched at in radians. This is relative to the world coordinate frame, but in the simulation if this is equal to `starting_angle` then the fuel will be launched directly towards the center of the hub.

`target_pitch (float)`: The launch angle or pitch the fuel should be launched at in radians. 0 is horizontal and pi/2 (90 degrees) is vertical.

`starting_velocity (float)`: The velocity in m/s of the fuel in the direction of travel when it spawned into the simulation (i.e. launched).

`experiment_tag`: A value that can be used to mark the fuel with a tag for later use in the analysis. This can be anything that can be turned into a string. The primary idea behind this is if the simulation sub-experiments you can use it to identify which fuel is associated with which sub-experiment.

```
sim.(starting_time=0,
            starting_distance=5,
            starting_angle=np.radians(45),
            target_yaw=np.radians(45),
            target_pitch=np.radians(60),
            starting_velocity=8.25,
            experiment_tag="RandomTest")
```

### Adding multiple fuel

For some experiments you might want to test fuel with a large combination of parameters. The following function helps set that up. Below are the parameters

`start_time (float)`: The time which the first fuel should be spawned into the simulation and launched in seconds.
`time_interval (float)`: The time between when fuel are spawned into the simulation.
`experiment_tag`: Same as above

All other parameters are the same as those listed previously, but they can also be provided as lists. The function will generate fuel with all possible combinations of values in each list. In the example below, 36 fuel will be created in total (3 `starting_distance` x 3 `target_pitch` x 4 `starling_velocity`), with each fuel being launched 0.25 seconds apart. Any parameter that is provided not as a list will be used for all fuel.

```
current_fuel_launch_time = sim.addAllFuelCombinations(
    start_time=current_fuel_launch_time,
    time_interval=0.25,
    starting_distance=[4, 5, 6],
    starting_angle=0,
    target_yaw=0,
    target_pitch=[np.radians(55),
                  np.radians(60),
                  np.radians(65)],
    starting_velocity=[7.75, 8.0, 8.25, 8.5],
    experiment_tag="GridTest")
```

### Randomization
While we do not provide a direct function for generating a lot of fuel with randomized parameters this can be easily achieved with pythons built in `random` library.
                    
## Running the Simulation

After all the fuel has been added the simulation can be run with the following line. `hertz` defines how many simulation steps should happen per second. `realtime` determines if the simulation should be run in realtime (i.e. pausing between steps) or if it should be run as fast as possible.

`sim.run(hertz=240, realtime=False)`

## Generating output

The results can be saved as follows. `save_paths` determines if the full path and velocities for each fuel should be saved. This produces a somewhat large file, so it is optional.

`sim.save_results(save_paths=False)`

## Output
Two files are output. These files can be loaded into your favorite spreadsheet software for analysis or with other tools such as Python itself.

- A CSV file with one fuel per line. The columns include all the initial parameters for the fuel as defined in the `addFuel` function and a `scored` column which is `True` or `False` depending on if that fuel scored in the hub or not.
- A JSON file with the positions and velocities for every fuel at every timestep.

# Physics Simulation
pyBullet is used for the physics simulation. The restitution (i.e. rebound) of the Fuel was determined via a simple drop test, which was then matched with the simulation. The hub is assumed to have the same characteristics as the floor. This is not likely to be the case and the Fuel will likely rebound less and potentiall more unpredictably off the hub, particularly the funnel. The rebound is also treated as linear with velocity, but that might not be the case in reality. The friction coefficent was set to something that seemed reasonable, but there is not really an easy way to match the simuation to reality for this.

The primary motivation for this utility was to dial in the trajectories of the Fuel, so there is less focus on the properties outside of that.

# Limitiations

- Currently fuel is never removed from the simulation. So if an experiment requires spawning in a lot of fuel (multiple hundreds?) then it might be worth spliting this into multiple runs. I have not really tested to see how far it can be pushed.
- Currently it is not setup to allow you to set the spin of the Fuel. This is possible and actually not that difficult with pyBullet. But without an easy way to determine and match the friction coefficent of the actual Fuel the results would be somewhat suspect.
