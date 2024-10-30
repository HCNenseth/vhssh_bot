# SPDX-License-Identifier: BSD-3-Clause

# flake8: noqa F401
from collections.abc import Callable

import numpy as np

from vendeeglobe import (
    Checkpoint,
    Heading,
    Instructions,
    Location,
    Vector,
    config,
)
from vendeeglobe.utils import distance_on_surface


class Bot:
    """
    This is the ship-controlling bot that will be instantiated for the competition.
    """

    def __init__(self):
        self.team = "TeamName"  # This is your team name
        # This is the course that the ship has to follow
        self.course = [
            Checkpoint(latitude=46.797109, longitude=-6.264905, radius=50),
            Checkpoint(latitude=58.797109, longitude=-48.264905, radius=50),
            Checkpoint(latitude=73.908577, longitude=-70.999811, radius=50),
            Checkpoint(latitude=73.797109, longitude=-76.264905, radius=50),
            Checkpoint(latitude=74.297109, longitude=-91.264905, radius=50),
            Checkpoint(latitude=73.797109, longitude=-113.264905, radius=10),
            Checkpoint(latitude=74.02, longitude=-111.10, radius=10),
            Checkpoint(latitude=75.0, longitude=-126.0, radius=10),
            Checkpoint(latitude=71, longitude=-128.660252, radius=50),
            Checkpoint(latitude=69.240264, longitude=-138.025125, radius=50),
            Checkpoint(latitude=70.806318, longitude=-150.943864, radius=50.0),
            Checkpoint(latitude=72.052286, longitude=-160.214572, radius=50.0),
            Checkpoint(latitude=67.668984, longitude=-169.674694, radius=50.0),
            Checkpoint(latitude=61.438937, longitude=-167.836265, radius=50.0),
            Checkpoint(latitude=-29.438937, longitude=171.836265, radius=50.0),
            Checkpoint(latitude=-48.5, longitude=147, radius=50.0),
            Checkpoint(latitude=-15, longitude=77, radius=1100.0),
            Checkpoint(latitude=-42, longitude=13, radius=50.0),
            Checkpoint(latitude=8, longitude=-24, radius=50.0),
            Checkpoint(latitude=46, longitude=-11, radius=50.0),
            Checkpoint(
                latitude=config.start.latitude,
                longitude=config.start.longitude,
                radius=5,
            ),
        ]

    def run(
        self,
        t: float,
        dt: float,
        longitude: float,
        latitude: float,
        heading: float,
        speed: float,
        vector: np.ndarray,
        forecast: Callable,
        world_map: Callable,
    ) -> Instructions:
        """
        This is the method that will be called at every time step to get the
        instructions for the ship.

        Parameters
        ----------
        t:
            The current time in hours.
        dt:
            The time step in hours.
        longitude:
            The current longitude of the ship.
        latitude:
            The current latitude of the ship.
        heading:
            The current heading of the ship.
        speed:
            The current speed of the ship.
        vector:
            The current heading of the ship, expressed as a vector.
        forecast:
            Method to query the weather forecast for the next 5 days.
            Example:
            current_position_forecast = forecast(
                latitudes=latitude, longitudes=longitude, times=0
            )
        world_map:
            Method to query map of the world: 1 for sea, 0 for land.
            Example:
            current_position_terrain = world_map(
                latitudes=latitude, longitudes=longitude
            )

        Returns
        -------
        instructions:
            A set of instructions for the ship. This can be:
            - a Location to go to
            - a Heading to point to
            - a Vector to follow
            - a number of degrees to turn Left
            - a number of degrees to turn Right

            Optionally, a sail value between 0 and 1 can be set.
        """
        # Initialize the instructions
        instructions = Instructions()

        # TODO: Remove this, it's only for testing =================
        current_position_forecast = forecast(
            latitudes=latitude, longitudes=longitude, times=0
        )
        current_position_terrain = world_map(latitudes=latitude, longitudes=longitude)
        # ===========================================================

        # Go through all checkpoints and find the next one to reach
        for ch in self.course:
            # Compute the distance to the checkpoint
            dist = distance_on_surface(
                longitude1=longitude,
                latitude1=latitude,
                longitude2=ch.longitude,
                latitude2=ch.latitude,
            )
            # Consider slowing down if the checkpoint is close
            jump = dt * np.linalg.norm(speed)
            if dist < 2.0 * ch.radius + jump:
                instructions.sail = min(ch.radius / jump, 1)
            else:
                instructions.sail = 1.0
            # Check if the checkpoint has been reached
            if dist < ch.radius:
                ch.reached = True
            if not ch.reached:
                instructions.location = Location(
                    longitude=ch.longitude, latitude=ch.latitude
                )
                break

        return instructions
