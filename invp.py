"""Models for the nonlinear inverted pendulum and its controller."""

import control
import numpy as np
from scipy import constants


class InvertedPendulumPlant:
    """A model for an inverted pendulum.

    The motion of the pendulum at any given time is described by its state vector, which contains the pendulum's
    angular displacement and velocity.
    """

    def __init__(self, m, l, gamma, x):
        """Creates a new inverted pendulum with a given state.

        Args:
            m: The mass of the pendulum.
            l: The length of the pendulum.
            gamma: The drag coefficient.
            x: The initial state.
        """
        self.m = m
        self.l = l
        self.gamma = gamma
        self.x = x

    def step(self, u, dt=1e-6):
        """Computes the next state of the system for a given input.

        Args:
            u: The input applied to the pendulum.
            dt: The time step to use.
        """
        # dx = f(x, u)
        dd_theta = (self.m * constants.g * self.l * np.sin(self.x[0])
                    - self.gamma * self.x[1]
                    + self.l * u * np.cos(self.x[0])) / (self.m * self.l ** 2)
        self.x = np.add(self.x, [self.x[1] * dt, dd_theta * dt])

    def output(self, u):
        """Computes the sensed output of the system for a given input.

        Args:
            u: The input applied to the pendulum.

        Returns:
            The sensed output of the system.
        """
        # y = h(x, u) = l sin(theta)
        return self.l * np.sin(self.x[0])


class ControlledInvertedPendulumPlant:
    def __init__(self, plant, x_eq, u_eq, y_eq, controller_poles, estimator_poles):
        """Creates a new inverted pendulum controller.

        Args:
            plant: The plant being controlled.
            x_eq: The equilibrium state (or operating point).
            u_eq: The input required to maintain equilibrium.
            y_eq: The sensed output in the equilibrium state.
            controller_poles: The poles to use for the controller.
            estimator_poles: The poles to use for the estimator.
        """
        self.plant = plant
        self.u_eq = u_eq
        self.y_eq = y_eq
        self.z_hat = [0, 0]

        # Create system matrices.
        self.A = [[0, 1],
                  [(plant.m * constants.g * np.cos(x_eq[0]) - u_eq * np.sin(x_eq[0])) / (plant.m * plant.l),
                   -plant.gamma / (plant.l ** 2 * plant.m)]]
        self.B = [[0], [np.cos(x_eq[0]) / (plant.m * plant.l)]]
        self.C = [[plant.l * np.cos(x_eq[0]), 0]]
        self.D = [[0]]
        self.K = control.place(self.A, self.B, controller_poles)
        self.L = np.transpose(control.place(np.transpose(self.A), np.transpose(self.C), estimator_poles))

        # Precompute frequently used matrices.
        self.A_BK = np.subtract(self.A, np.matmul(self.B, self.K))
        self.C_DK = np.subtract(self.C, np.matmul(self.D, self.K))

    def step(self, dt=1e-6):
        """Computes the next state of the controlled system.

        Returns:
            The next state of the controlled system.
        """
        # Compute the input.
        u = self.input()

        # Compute the output error.
        #   eta = h(x, u_e - K z_hat) - y_e
        #   eta_hat = (C - DK) z_hat
        eta = self.plant.output(u) - self.y_eq
        eta_hat = np.dot(self.C_DK, self.z_hat)

        # Update the estimator's state.
        #   dz_hat = (A - BK) z_hat + L(eta - eta_hat)
        #   z_hat = z_hat + dz_hat * dt
        dz_hat = np.add(np.dot(self.A_BK, self.z_hat), np.dot(self.L, np.subtract(eta, eta_hat)))
        self.z_hat = np.add(self.z_hat, np.dot(dz_hat, dt))

        # Apply input to the plant.
        return self.plant.step(u, dt)

    def input(self):
        """Computes the input to apply to the plant."""
        #   u = u_e - K z_hat
        return self.u_eq - np.matmul(self.K, self.z_hat)[0]

    def output(self):
        """Computes the sensed output for the estimated system."""
        # eta_hat = (C - DK) z_hat
        return np.dot(self.C_DK, self.z_hat)
