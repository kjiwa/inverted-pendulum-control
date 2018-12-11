import invp
import matplotlib.pyplot as plt
import numpy as np
from scipy import constants

mass = 1
length = 0.25
friction = 0.1

# Operating point
x_eq = [np.deg2rad(33), 0]
u_eq = -mass * constants.g * np.tan(x_eq[0])
y_eq = length * np.sin(x_eq[0])

# Plant and controlled system

controller_poles = [-1 + 1j * np.sqrt(3), -1 - 1j * np.sqrt(3)]
estimator_poles = [-5, -5 + 1e-12]

# Phase portrait of the uncontrolled system
dt = 1e-3
theta0, dtheta0 = np.vstack(
    map(np.ravel, np.meshgrid(np.linspace(np.deg2rad(31), np.deg2rad(35), 20), np.linspace(-0.125, 0.125, 20))))
theta1, dtheta1 = [], []
for i, j in zip(theta0, dtheta0):
    plant = invp.InvertedPendulumPlant(mass, length, friction, [i, j])
    plant.step(u_eq, dt)
    x1 = plant.x

    theta1.append(x1[0] - i)
    dtheta1.append(x1[1] - j)

plt.figure()
plt.quiver(theta0, dtheta0, theta1, dtheta1, np.hypot(theta1, dtheta1))
plt.xlabel('$\\theta$ (rad)')
plt.ylabel('$\\dot{\\theta}$ (rad/s)')
plt.tight_layout()

# Phase portrait of the controlled system
for i, j in zip(theta0, dtheta0):
    plant = invp.InvertedPendulumPlant(mass, length, friction, [i, j])
    controlled_plant = invp.ControlledInvertedPendulumPlant(plant, x_eq, u_eq, y_eq, controller_poles, estimator_poles)
    controlled_plant.step(dt)
    x1 = plant.x

    theta1.append(x1[0] - i)
    dtheta1.append(x1[1] - j)

fig_phase = plt.figure()
plt.quiver(theta0, dtheta0, theta1, dtheta1, np.hypot(theta1, dtheta1))
plt.xlabel('$\\theta$ (rad)')
plt.ylabel('$\\dot{\\theta}$ (rad/s)')
plt.tight_layout()

# Time response
t_0 = 0
t_f = 6
t = np.linspace(t_0, t_f, int(t_f / dt))

initial_conditions = [
    [np.deg2rad(33), 0],
    [np.deg2rad(33.3), -0.01],
    [np.deg2rad(32.9), 0.02],
    [np.deg2rad(33), -0.03]
]

for ic in initial_conditions:
    plant = invp.InvertedPendulumPlant(mass, length, friction, ic)
    controlled_plant = invp.ControlledInvertedPendulumPlant(plant, x_eq, u_eq, y_eq, controller_poles, estimator_poles)

    X, Z_hat, U, Y, ETA_hat = [], [], [], [], []
    for _ in t:
        u = controlled_plant.input()
        controlled_plant.step(dt)

        U.append(u)
        X.append(plant.x)
        Z_hat.append(controlled_plant.z_hat)
        Y.append(plant.output(u))
        ETA_hat.append(controlled_plant.output())

    # Plot time response
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(t, np.dot(100, Y))
    plt.plot(t, np.dot(100, ETA_hat + y_eq))
    plt.title('Pendulum Horizontal Displacement')
    plt.xlabel('Time (s)')
    plt.ylabel('Displacement (cm)')
    plt.legend(['Actual Displacement', 'Estimated Displacement'])

    plt.subplot(2, 1, 2)
    plt.plot(t, U)
    plt.title('Torque')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (N)')
    plt.tight_layout()

    # Plot trajectory on phase portrait
    plt.figure(fig_phase.number)
    plt.plot([x[0] for x in X], [x[1] for x in X])

    # Actual vs estimated trajectories
    plt.figure()
    plt.plot([x[0] for x in X], [x[1] for x in X])
    plt.plot([z[0] + x_eq[0] for z in Z_hat], [z[1] for z in Z_hat])
    plt.title('Actual vs Estimated Trajectories')
    plt.xlabel('$\\theta$ (rad)')
    plt.ylabel('$\\dot{\\theta}$ (rad/s)')
    plt.xlim([np.deg2rad(31), np.deg2rad(35)])
    plt.ylim([-0.125, 0.125])
    plt.legend(['Actual Trajectory', 'Estimated Trajectory'])
    plt.tight_layout()

plt.show()
