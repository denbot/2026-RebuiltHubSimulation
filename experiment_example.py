import numpy as np
import random

from rebuilt_hub_simulation import RebuiltHubSimuation


def main():
    sim = RebuiltHubSimuation(experiment_name="example", show_gui=True)

    #Initialize the launch time for fuel to the start of the simulation
    current_fuel_launch_time = 0

    #Loop 40 times adding fuel with different parameters
    for _ in range(40):

        #Increment the launch time by 0.25 seconds for each fuel added so they don't launch at the same time
        current_fuel_launch_time += 0.25

        #Add a fuel with some parameters randomized
        #The target_yaw is 45 degrees plus a random value between -2 and 2 degrees
        #The starting_velocity is a uniform random value between 7.75 and 8.5 m/s
        #The experiment_tag is set to something to differentiate this test from others
        sim.addFuel(starting_time=current_fuel_launch_time,
                    starting_distance=5,
                    starting_angle=np.radians(45),
                    target_yaw=np.radians(45) +
                    np.radians(random.uniform(-2, 2)),
                    target_pitch=np.radians(60),
                    starting_velocity=random.uniform(7.75, 8.5),
                    experiment_tag="RandomTest")

    #This adds multiple fuels using a grid of parameters. i.e. Fuels with all combinations of the provided parameters will be added.
    #The start time is the time the first fuel is added. Each subsequent fuel is added at intervals of time_interval seconds after that.
    #In total this will add 36 fuels (3 starting distances x 3 target pitches x 4 starting velocities)
    #The experiment_tag is set to something to differentiate this test from others

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

    #Run the simulation at 240 Hz (i.e. 240 simulation steps per second of simulation time)
    #Realtime is disabled so the simulation will run as fast as possible
    sim.run(hertz=240, realtime=False)

    #Save the results in a folder based on the experiment name
    #The save_paths flag indicates whether to save the position, velocity, and time paths for each fuel in addition to the summary CSV
    #The paths can be large files so this is optional
    #The summary CSV contains a line for each fuel with its parameters and whether it was scored or not
    #This can be loaded in excel or similar tools for analysis
    #Python can also be used to load and analyze the results programmatically
    sim.save_results(save_paths=False)

    print("Completed the simulation.")


if __name__ == "__main__":
    main()
