import itertools
from typing import Any, List, Optional, Union
import pybullet as p
import time
import pybullet_data
import numpy as np
from dataclasses import dataclass, fields, asdict
import os
import csv
import json


@dataclass
class SimulationFuel:
    id: int
    starting_time: float
    starting_distance: float
    starting_angle: float
    target_yaw: float
    target_pitch: float
    starting_velocity: float
    starting_height: float
    experiment_tag: Any


class RebuiltHubSimuation:

    def __init__(self,
                 experiment_name: str = "experiment",
                 base_path: str = ".",
                 show_gui: bool = True):
        self.base_path = base_path
        self.experiment_name = experiment_name

        self.physicsClient = p.connect(p.GUI if show_gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
        p.setRealTimeSimulation(0)
        p.setGravity(0, 0, -9.80665)

        base_asset_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "assets")

        planeId = p.loadURDF(os.path.join(base_asset_path, "plane.urdf"))
        p.changeDynamics(planeId, -1, lateralFriction=1.0, restitution=1.0)

        mesh_path = os.path.join(base_asset_path, "hub.obj")
        collision_shape_id = p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=mesh_path,
            flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
        )

        visual_shape_id = p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=mesh_path,
        )

        hub_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=collision_shape_id,
            baseVisualShapeIndex=visual_shape_id,
        )
        p.resetBasePositionAndOrientation(
            hub_id,
            [0, 0, 0.0],
            p.getQuaternionFromEuler([np.radians(90), 0,
                                      np.radians(180)]),
        )

        p.changeDynamics(hub_id, -1, lateralFriction=1.0, restitution=1.0)

        self.fuel_defs = []
        self.fuel_tracking = []

    def addAllFuelCombinations(self,
                               start_time=0.0,
                               time_interval=0.25,
                               starting_distance: Union[List, float] = 2,
                               starting_angle: Union[List, float] = 0,
                               target_yaw: Union[List, float] = 0,
                               target_pitch: Union[List, float] = 0,
                               starting_velocity: Union[List, float] = 5.0,
                               starting_height: Union[List, float] = 0.5,
                               experiment_tag="") -> float:
        """
        Add fuel definitions for all combinations of the provided parameters.

        Parameters:
            start_time (float): The starting time for the first fuel (seconds)
            time_interval (float): Time interval between each fuel addition (seconds)
            starting_distance (Union[List, float]): List of starting distances or single value (meters)
            starting_angle (Union[List, float]): List of starting angles or single value (radians)
            target_yaw (Union[List, float]): List of target yaws or single value (radians)
            target_pitch (Union[List, float]): List of target pitches or single value (radians)
            starting_velocity (Union[List, float]): List of starting velocities or single value (meters/second)
            experiment_tag (str): Tag to associate with each fuel definition

        Returns:
            float: The simulation time of the last fuel added
        """

        if not isinstance(starting_distance, list):
            starting_distance = [starting_distance]
        if not isinstance(starting_angle, list):
            starting_angle = [starting_angle]
        if not isinstance(target_yaw, list):
            target_yaw = [target_yaw]
        if not isinstance(target_pitch, list):
            target_pitch = [target_pitch]
        if not isinstance(starting_velocity, list):
            starting_velocity = [starting_velocity]
        if not isinstance(starting_height, list):
            starting_height = [starting_height]

        for i, (sd, sa, ty, tp, sv, sh) in enumerate(
                itertools.product(starting_distance, starting_angle,
                                  target_yaw, target_pitch, starting_velocity,
                                  starting_height)):
            self.addFuel(
                starting_time=start_time + i * time_interval,
                starting_distance=sd,
                starting_angle=sa,
                target_yaw=ty,
                target_pitch=tp,
                starting_velocity=sv,
                starting_height=sh,
                experiment_tag=experiment_tag,
            )

    def addFuel(
        self,
        starting_time: float = 0.0,
        starting_distance: float = 2,
        starting_angle: float = 0,
        target_yaw: float = 0,
        target_pitch: float = 0,
        starting_velocity: float = 5.0,
        starting_height: float = 0.5,
        experiment_tag: Any = "",
    ):
        """
        Add a fuel definition to the simulation

        Parameters:
            starting_time (float): Time at which to add the fuel to the simulation (seconds)
            starting_distance (float): Distance from the hub center to start (meters)
            starting_angle (float): Angle around the hub to start (radians)
            target_yaw (float): Yaw angle to aim the fuel (radians)
            target_pitch (float): Pitch angle to aim the fuel (radians)
            starting_velocity (float): Initial velocity of the fuel (meters/second)
            starting_height (float): Initial height of the fuel (meters)
            experiment_tag (Any): User defined tag for tracking purposes
        """
        self.fuel_defs.append(
            SimulationFuel(
                id=None,
                starting_time=starting_time,
                starting_distance=starting_distance,
                starting_angle=starting_angle,
                target_yaw=target_yaw,
                target_pitch=target_pitch,
                starting_velocity=starting_velocity,
                starting_height=starting_height,
                experiment_tag=experiment_tag,
            ))

    def _addFuelToSim(self, fuel):
        startX = fuel.starting_distance * np.cos(fuel.starting_angle + np.pi)
        startY = fuel.starting_distance * np.sin(fuel.starting_angle + np.pi)
        startPos = [startX, startY, fuel.starting_height]

        startVelocity = fuel.starting_velocity

        startVectorVelocity = [
            startVelocity * np.cos(fuel.target_pitch) *
            np.cos(fuel.target_yaw),
            startVelocity * np.cos(fuel.target_pitch) *
            np.sin(fuel.target_yaw),
            startVelocity * np.sin(fuel.target_pitch),
        ]

        id = p.loadURDF("fuel.urdf", startPos,
                        p.getQuaternionFromEuler([0, 0, 0]))
        p.changeDynamics(id,
                         -1,
                         lateralFriction=1.0,
                         restitution=0.55,
                         linearDamping=0)
        p.resetBaseVelocity(id, linearVelocity=startVectorVelocity)

        fuel.id = id
        fuel_data = {
            "definition": fuel,
            "positions": [],
            "times": [],
            "velocities": [],
            "scored": False,
        }
        self.fuel_tracking.append(fuel_data)

    def did_score(self, fuel) -> bool:
        """Check if a fuel scored by checking its position

        Parameters:
            fuel (SimulationFuel): The fuel to check
        Returns:
            bool: True if the fuel scored, False otherwise
        """

        scoring_epsilon = 0.1

        hub_width = 1.168
        height_to_outlet = 0.751
        outlet_size = 0.198
        scoring_x_plane = hub_width / 2
        scoring_box_y = [-hub_width / 2, hub_width / 2]
        scoring_box_z = [height_to_outlet, height_to_outlet + outlet_size]

        # Some parameters to check if the z velocity is low enough to stop checking
        low_velocity_counter = 0
        velocity_threshold = 0.5
        velocity_check_count = 10

        for pos, vel in zip(fuel["positions"], fuel["velocities"]):
            if (abs(pos[0] - scoring_x_plane) < scoring_epsilon
                    and scoring_box_y[0] <= pos[1]
                    and pos[1] <= scoring_box_y[1]
                    and scoring_box_z[0] <= pos[2]
                    and pos[2] <= scoring_box_z[1]):
                return True

            # Check if the vertical velocity is low for a while while being below the scoring box
            # This indicates the fuel has fallen past the scoring area and is not moving upward so we can stop checking
            if vel[2] < velocity_threshold and pos[2] < scoring_box_z[0]:
                low_velocity_counter += 1
            else:
                low_velocity_counter = 0
            if low_velocity_counter >= velocity_check_count:
                break

        return False

    def save_results(self, save_paths: bool = False):
        """Save the simulation results to files"""

        # Find the next experiment number to add to the dictionary
        experiment_num = 1
        for path in os.listdir(self.base_path):
            if (path.startswith(self.experiment_name)
                    and path[len(self.experiment_name):].isdigit() and int(
                        path[len(self.experiment_name):]) >= experiment_num):
                experiment_num = int(path[len(self.experiment_name):]) + 1

        experiment_base_dir = os.path.join(
            self.base_path, f"{self.experiment_name}{experiment_num}")

        os.makedirs(experiment_base_dir, exist_ok=True)

        csv_filename = os.path.join(experiment_base_dir, "results.csv")

        # Define the fieldnames (CSV header) based on dataclass fields
        fieldnames = [field.name
                      for field in fields(SimulationFuel)] + ["scored"]

        # Write to a CSV file
        with open(csv_filename, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)

            writer.writeheader()
            for fuel in self.fuel_tracking:
                row_dict = asdict(fuel["definition"])
                formatted_row = {}
                for k, v in row_dict.items():
                    if isinstance(v, float):
                        formatted_row[k] = f"{v:.5f}"
                    else:
                        formatted_row[k] = v
                formatted_row["scored"] = fuel["scored"]
                writer.writerow(formatted_row)

        #Save Positions, Velocities, and Simulation Times in JSON file
        if save_paths:
            json_output_dict = {}
            for fuel in self.fuel_tracking:
                json_output_dict[fuel["definition"].id] = {
                    "positions": fuel["positions"],
                    "velocities": fuel["velocities"],
                    "times": fuel["times"],
                }

            json_filename = os.path.join(experiment_base_dir,
                                         "fuel_path_data.json")
            with open(json_filename, "w") as f:
                json.dump(json_output_dict, f, indent=4)

        print("Results saved to:", experiment_base_dir)

    def run(
        self,
        simLength: Optional[float] = None,
        hertz: int = 240,
        realtime: bool = False,
    ):
        """Run the simulation

        Parameters:
            simLength (Optional[float]): Length of the simulation in seconds
            hertz (int): Simulation frequency in Hz
            realtime (bool): Whether to run the simulation in real-time
        """

        extra_time = 10.0  # Extra time after last fuel is added to let it settle

        self.fuel_defs.sort(key=lambda x: x.starting_time, reverse=False)
        if simLength is None:
            if len(self.fuel_defs) == 0:
                simLength = 10.0
            else:
                last_fuel = self.fuel_defs[-1]
                simLength = last_fuel.starting_time + extra_time

        next_fuel_index = 0
        p.setTimeStep(1.0 / hertz)
        numSteps = int(simLength * hertz)
        for i in range(numSteps):
            curr_time = i * (1.0 / hertz)

            while (next_fuel_index < len(self.fuel_defs) and curr_time
                   >= self.fuel_defs[next_fuel_index].starting_time):
                fuel_def = self.fuel_defs[next_fuel_index]
                self._addFuelToSim(fuel_def)
                next_fuel_index += 1

            p.stepSimulation()

            if realtime:
                time.sleep(1.0 / hertz)

            for fuel in self.fuel_tracking:
                fuel_id = fuel["definition"].id

                pos, orn = p.getBasePositionAndOrientation(fuel_id)
                vel, ang_vel = p.getBaseVelocity(fuel_id)

                fuel["positions"].append(pos)
                fuel["times"].append(curr_time)
                fuel["velocities"].append(vel)

        p.disconnect()

        for fuel in self.fuel_tracking:
            fuel["scored"] = self.did_score(fuel)
