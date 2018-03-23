import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import numpy as np
from abc import ABCMeta, abstractmethod
import typing
import math


class SpringSeries:
    def __init__(self, spring_type):
        self.current_time_ = 0
        self.velocities_ = []
        self.distances_ = []
        self.time_points_ = []

        self.spring_ = spring_type

    def calc_series(self, start_velocity, start_position, time_step, to):
        current_velocity = start_velocity
        current_position = start_position

        while self.current_time_ < to:
            self.velocities_.append(current_velocity)
            self.distances_.append(current_position)
            self.time_points_.append(self.current_time_)

            current_velocity = self.spring_.do_timestep(time_step=time_step,
                                                        relative_velocity=current_velocity,
                                                        current_position=current_position)

            current_position += current_velocity * time_step

            self.current_time_ += time_step


class AbstractSpring(metaclass=ABCMeta):

    @abstractmethod
    def do_timestep(self, relative_velocity, current_position: float, time_step: float) -> float:
        pass


class CriticallyLoadedSpring(AbstractSpring):

    def __init__(self, target_position):
        self.spring_constant_ = 3
        self.target_position_ = target_position
        self.predicted_position_ = 0

    def do_timestep(self, relative_velocity, current_position: float, time_step: float) -> float:
        current_to_target = self.target_position_ - current_position
        spring_force = current_to_target * self.spring_constant_
        damping_force = -relative_velocity * 2 * math.sqrt(self.spring_constant_)
        force = spring_force + damping_force
        relative_velocity += force * time_step
        self.predicted_position_ = current_position + (relative_velocity * time_step)
        return relative_velocity


class SelfAdjustingRigiditySpring(AbstractSpring):
    def __init__(self, target_position):
        self.boring_spring_ = CriticallyLoadedSpring(target_position=target_position)

        # geradengleichung ermitteln
        # je näher wir an den punkt kommen zu dem wir wollen, desto härter muss die Feder ziehen
        # da wir sonst immer hinter dem führenden auto zu weit hinterher fahren

        self.a = (0 - 15) / (target_position - 5 - target_position)
        self.b = 15 - (self.a * target_position)

        """
        target_position = -1
        a = (2 - 30) / (target_position-5 - target_position)
        b = 30 - (a * target_position)
        
        print(a * max(-6, -6) + b, a * max(-10, -6) + b, a * max(0, -6) + b)
        """

    def do_timestep(self, relative_velocity, current_position: float, time_step: float) -> float:
        # dont go below -6 as position, so we have at least 2 as a spring constant
        # dont go above -1 as position so we have at most 15 as spring constant
        self.boring_spring_.spring_constant_ = \
            self.a * max(-6., min(current_position, -1.)) + self.b

        return self.boring_spring_.do_timestep(relative_velocity=relative_velocity,
                                               current_position=current_position,
                                               time_step=time_step)


# http://robotic-controls.com/learn/programming/pd-feedback-control-introduction
class PDController(AbstractSpring):
    def __init__(self, target_position):
        self.kp = 0.5 # factor to multiply with error. puts the error proportionally to the desired value
        self.kd = 0.1 # rate of change factor. says "slow down before you get there"
        self.target_position = -target_position

    def do_timestep(self, relative_velocity, current_position: float, time_step: float) ->float:
        y = self.kp * (self.target_position - current_position) - self.kd * relative_velocity
        print( self.target_position,  current_position, y)
        return y

class Car:
    def __init__(self, initial_pos: float, timestep: float, current_velocity: float,
                 target_distance: float):
        self.velocity = current_velocity
        self.pos = initial_pos
        self.time_step = timestep
        self.target_distance = target_distance

        self.catch_up_spring_ = CriticallyLoadedSpring(target_position=-self.target_distance)

    def drive_constant(self):
        self.pos += self.velocity * self.time_step

    def drive_by_range(self, distance: float, lv_speed_offset: float):
        relative_velocity = self.velocity - lv_speed_offset
        self.velocity = lv_speed_offset + self.calc_spring_(-distance, relative_velocity)

        self.pos += self.velocity * self.time_step

    def calc_spring_(self, current_position: float, relative_velocity: float) -> float:
        return self.catch_up_spring_.do_timestep(relative_velocity=relative_velocity,
                                                 current_position=current_position,
                                                 time_step=self.time_step)

    def clamp_vel(self, n, time_step):
        smallest = -5 * time_step
        largest = 8 * time_step
        return max(smallest, min(n, largest))


class Platoon:
    def __init__(self, lv: Car, fv: Car):
        self.LV = lv
        self.FV = fv

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

            self.FV.drive_by_range(self.LV.pos - self.FV.pos, self.LV.velocity)
            self.LV.drive_constant()

            self.current_time += timestep


def plot_distance(spring_series: SpringSeries, header: str):
    max_velo = max(spring_series.velocities_) + 2
    min_velo = min(0, min(spring_series.velocities_) - 2)
    max_position = max(spring_series.distances_) + 2
    min_position = min(0, min(spring_series.distances_) - 2)
    max_time = max(spring_series.time_points_)

    plt.figure(1)
    p = plt.subplot(211)
    plt.title(header)
    p.axes.axis([0, max_time, min_velo, max_velo])
    plt.locator_params(axis='y', nbins=10)
    plt.grid()
    h = plt.ylabel("velocity")
    h.set_rotation(90)
    p.plot(spring_series.time_points_, spring_series.velocities_)

    p = plt.subplot(212)
    p.axes.axis([0, max_time, min_position, max_position])
    plt.locator_params(axis='y', nbins=10)
    plt.grid()
    h = plt.ylabel("position")
    h.set_rotation(90)
    p.axes.plot(spring_series.time_points_, spring_series.distances_)

    # fig.savefig("test.png")

    plt.show()


def plot_platoon(platoon: Platoon, header: str):
    max_velo = max(platoon.fv_velocity) + 2
    min_velo = min(0, min(platoon.fv_velocity) - 2, min(platoon.fv_velocity) - 2)
    max_distance = max(platoon.dist_diff) + 2
    min_distance = min(0, min(platoon.dist_diff) - 2)
    max_time = 15

    plt.figure(1)
    p = plt.subplot(211)
    plt.title(header)
    p.axes.axis([0, max_time, min_velo, max_velo])
    plt.locator_params(axis='y', nbins=10)
    plt.grid()
    h = plt.ylabel("LV/FV velocities")
    h.set_rotation(90)
    p.plot(platoon.time_points, platoon.lv_velocity)
    p.plot(platoon.time_points, platoon.fv_velocity)

    plt.figure(1)
    p = plt.subplot(212)
    p.axes.axis([0, max_time, min_distance, max_distance])
    plt.locator_params(axis='y', nbins=10)
    plt.grid()
    h = plt.ylabel("FV/LV distance difference")
    h.set_rotation(90)
    p.plot(platoon.time_points, platoon.dist_diff)

    plt.show()


def plot_distance_5to1():
    # y axis velocities
    #
    time_step_len = 0.2

    current_velocity = 0
    current_distance = 5
    target_distance = 1
    s = SpringSeries(spring_type=CriticallyLoadedSpring(-target_distance))
    s.calc_series(current_velocity, -current_distance, time_step_len, 10)

    plot_distance(s, "CLS vel 0, -5m to -1m")

    s = SpringSeries(spring_type=PDController(-target_distance))
    s.calc_series(current_velocity, -current_distance, time_step_len, 10)

    plot_distance(s, "PDcontroller vel 0, -5m to -1m")


def plot_distance_1to5():
    # y axis velocities
    #
    time_step_len = 0.2

    current_velocity = 0
    current_distance = 1
    target_distance = 5
    s = SpringSeries(spring_type=CriticallyLoadedSpring(-target_distance))
    s.calc_series(current_velocity, -current_distance, time_step_len, 10)

    plot_distance(s, "CLS vel 0, -1m to -5m")

    s = SpringSeries(spring_type=PDController(-target_distance))
    s.calc_series(current_velocity, -current_distance, time_step_len, 10)

    plot_distance(s, "PD vel 0, -1m to -5m")


def plot_platoon_catchup():
    time_step_len = 0.2

    lv = Car(initial_pos=5, timestep=0.3, current_velocity=5, target_distance=1)
    fv = Car(initial_pos=1, timestep=0.3, current_velocity=5, target_distance=1)

    platoon = Platoon(lv, fv)

    platoon.do_series(time_step_len, 15)

    plot_platoon(platoon=platoon, header="CLS catch up from dist5 to 1, from vel 0 to x")

    lv = Car(initial_pos=5, timestep=0.3, current_velocity=5, target_distance=1)
    fv = Car(initial_pos=1, timestep=0.3, current_velocity=5, target_distance=1)
    fv.catch_up_spring_ = PDController(target_position=-1)

    platoon = Platoon(lv, fv)

    platoon.do_series(time_step_len, 15)

    plot_platoon(platoon=platoon, header="PD catch up from dist5 to 1, from vel 0 to x")


def plot_platoon_brake():
    time_step_len = 0.2

    lv = Car(initial_pos=0.3, timestep=0.3, current_velocity=5, target_distance=1)
    fv = Car(initial_pos=0, timestep=0.3, current_velocity=5, target_distance=1)

    platoon = Platoon(lv, fv)

    platoon.do_series(time_step_len, 15)

    plot_platoon(platoon=platoon, header="CLS brake from 0.3 to 1, vel 5")

    lv = Car(initial_pos=0.3, timestep=0.3, current_velocity=5, target_distance=1)
    fv = Car(initial_pos=0, timestep=0.3, current_velocity=5, target_distance=1)
    fv.catch_up_spring_ = PDController(target_position=-1)

    platoon = Platoon(lv, fv)

    platoon.do_series(time_step_len, 15)

    plot_platoon(platoon=platoon, header="CLS brake from 0.3 to 1, vel 5")


plot_distance_5to1()
plot_distance_1to5()
plot_platoon_catchup()
plot_platoon_brake()