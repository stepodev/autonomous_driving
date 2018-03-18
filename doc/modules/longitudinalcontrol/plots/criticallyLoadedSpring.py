import matplotlib.pyplot as plt
import numpy as np
import math


class SpringSeries:
    def __init__(self):
        self.current_time_ = 0
        self.velocities_ = []
        self.distances_ = []
        self.time_points_ = []

        self.spring_ = SpringState()

    def calc_series(self, start_velocity, start_distance, time_step, to):

        current_velocity = start_velocity
        current_distance = start_distance

        while self.current_time_ < to:
            self.velocities_.append(current_velocity)
            self.distances_.append(current_distance)
            self.time_points_.append(self.current_time_)

            current_velocity = self.spring_.do_timestep(time_step=time_step,
                                                        current_velocity=current_velocity,
                                                        current_distance=current_distance)

            current_distance -= current_velocity * time_step

            self.current_time_ += time_step


class SpringState:

    def __init__(self):
        self.spring_constant_ = 2
        self.target_distance_ = 0

    def do_timestep(self, current_velocity, current_distance: float, time_step: float) -> float:
        current_to_target = current_distance - self.target_distance_
        spring_force = current_to_target * self.spring_constant_
        damping_force = -current_velocity * 2 * math.sqrt(self.spring_constant_)
        force = spring_force + damping_force
        current_velocity += force * time_step
        return current_velocity


def plot_distance_5to1():
    # y axis velocities
    #
    time_step_len = 0.2

    current_velocity = 0
    current_distance = 5
    s = SpringSeries()
    s.calc_series(current_velocity, current_distance, time_step_len, 10 )

    plot_distance(s, "dist 5 to 1, vel 0")


def plot_distance(spring_series: SpringSeries, header: str):
    max_velo = max(spring_series.velocities_) * 1.2
    min_velo = min(0, min(spring_series.velocities_) * 1.2)
    max_distance = max(spring_series.distances_) * 1.2
    min_distance = min(0, min(spring_series.distances_) * 1.2)
    max_time = 15

    plt.figure(1)
    p = plt.subplot(211)
    plt.title(header)
    p.axes.axis([0, max_time, min_velo, max_velo])
    h = plt.ylabel("velocity")
    h.set_rotation(90)
    p.plot(spring_series.time_points_, spring_series.velocities_)

    p = plt.subplot(212)
    p.axes.axis([0, max_time, min_distance, max_distance])
    h = plt.ylabel("distance")
    h.set_rotation(90)
    p.axes.plot(spring_series.time_points_, spring_series.distances_)

    # fig.savefig("test.png")

    plt.show()


class Car:
    def __init__(self, initial_pos: float, timestep: float, current_velocity: float,
                 target_distance: float):
        self.velocity = current_velocity
        self.pos = initial_pos
        self.time_step = timestep
        self.target_distance = target_distance

        self.catch_up_spring_ = SpringState()
        self.keep_away_spring_ = SpringState()
        self.keep_away_spring_.target_distance_ = -self.target_distance
        self.keep_away_spring_.spring_constant_ = 20


    def drive_constant(self):
        self.pos += self.velocity * self.time_step

    def drive_spring(self, current_distance):

        print( current_distance, self.target_distance )

        if current_distance > self.target_distance:
            self.velocity = self.catch_up_spring_.do_timestep(current_velocity=self.velocity,
                                                     current_distance=current_distance,
                                                     time_step=self.time_step)
        else:
            self.velocity = self.keep_away_spring_.do_timestep(current_velocity=self.velocity,
                                                              current_distance=-current_distance,
                                                              time_step=self.time_step)
        self.pos += self.velocity * self.time_step


class Platoon:
    def __init__(self, lv: Car, fv: Car):
        self.LV = lv
        self.FV = fv

        self.FV.drive = self.FV.drive_spring

        self.current_time = 0
        self.time_points = []

        self.lv_pos = []
        self.fv_pos = []
        self.lv_velocity = []
        self.fv_velocity = []
        self.dist_diff = []

    def do_series(self, timestep: float, to: float):
        while self.current_time < to:

            self.lv_pos.append(self.LV.pos)
            self.fv_pos.append(self.FV.pos)

            self.lv_velocity.append(self.LV.velocity)
            self.fv_velocity.append(self.FV.velocity)

            self.dist_diff.append(self.LV.pos - self.FV.pos)

            self.time_points.append(self.current_time)

            self.LV.drive_constant()
            self.FV.drive_spring(self.LV.pos - self.FV.pos)

            self.current_time += timestep


def plot_platoon_catchup():
    time_step_len = 0.2

    lv = Car(initial_pos=5, timestep=0.3, current_velocity=5, target_distance=1)
    fv = Car(initial_pos=0, timestep=0.3, current_velocity=0, target_distance=1)

    platoon = Platoon(lv, fv)

    platoon.do_series(time_step_len, 15)

    plot_platoon(platoon=platoon, header="catch up from dist5 to 1, from vel 0 to x")

def plot_platoon_brake():
    time_step_len = 0.2

    lv = Car(initial_pos=0.3, timestep=0.3, current_velocity=5, target_distance=1)
    fv = Car(initial_pos=0, timestep=0.3, current_velocity=5, target_distance=1)

    platoon = Platoon(lv, fv)

    platoon.do_series(time_step_len, 15)

    plot_platoon(platoon=platoon, header="brake from 0.3 to 1, vel 5")


def plot_platoon(platoon: Platoon, header: str):
    max_velo = max(platoon.fv_velocity) * 1.2
    min_velo = min(0, min(platoon.fv_velocity) * 1.2, min(platoon.fv_velocity) * 1.2 )
    max_distance = max(platoon.dist_diff) * 1.2
    min_distance = min(0, min(platoon.dist_diff) * 1.2)
    max_time = 15

    plt.figure(1)
    p = plt.subplot(211)
    plt.title(header)
    p.axes.axis([0, max_time, min_velo, max_velo])
    h = plt.ylabel("LV/FV velocities")
    h.set_rotation(90)
    p.plot(platoon.time_points, platoon.lv_velocity)
    p.plot(platoon.time_points, platoon.fv_velocity)

    plt.figure(1)
    p = plt.subplot(212)
    p.axes.axis([0, max_time, min_distance, max_distance])
    h = plt.ylabel("FV/LV distance difference")
    h.set_rotation(90)
    p.plot(platoon.time_points, platoon.dist_diff)

    plt.show()


plot_distance_5to1()
plot_platoon_catchup()
plot_platoon_brake()
