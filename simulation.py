import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from itertools import combinations
from MCBF import CBF_SDP

def main():
    DRONE_COUNT = 5
    COMM_RANGE = 1.3
    DRONE_SIZE = 0.2
    FLIGHT_HEIGHT = 0.5
    SCALE_FACTOR = 1/1.3
    save_sim_animation = False
    avoid_collision = True
    kp = 2

    simulation_time = 10.0
    dt = 0.01 # sec

    time_data = np.arange(0, simulation_time, dt).reshape(-1,1)
    pos_data = [np.tile([0.0, 0.0, FLIGHT_HEIGHT], (len(time_data), 1)) for _ in range(DRONE_COUNT)]
    vel_data = [np.zeros((len(time_data), 3)) for _ in range(DRONE_COUNT)]

    
    pos_data[0][0, :2] += SCALE_FACTOR * COMM_RANGE * np.array([0.0, 0.0])
    pos_data[1][0, :2] += SCALE_FACTOR * COMM_RANGE * np.array([-0.707, 0.707])
    pos_data[2][0, :2] += SCALE_FACTOR * COMM_RANGE * np.array([0.707, 0.707])
    pos_data[3][0, :2] += SCALE_FACTOR * COMM_RANGE * np.array([-0.707, -0.707])
    pos_data[4][0, :2] += SCALE_FACTOR * COMM_RANGE * np.array([0.607, -0.707])

    destination = np.array([pos_data[_][0, :2] for _ in range(DRONE_COUNT)])

    cbf_sdp = CBF_SDP(COMM_RANGE, DRONE_COUNT, DRONE_SIZE, avoid_collision)

    for t in range(1, len(time_data)):
        current_positions = np.array([pos_data[i][t - 1, :2] for i in range(DRONE_COUNT)])
        destination[0,:] = 0.5 * (1 - np.cos(2 * np.pi * time_data[t,0] / simulation_time)) * SCALE_FACTOR * COMM_RANGE * np.array([1, -1])

        u_des = cbf_sdp.compute_desired_velocities(current_positions, destination, kp=kp)
        u_des[0,:] +=  0.5 * (2 * np.pi / simulation_time) * (np.sin(2 * np.pi * time_data[t,0] / simulation_time)) * SCALE_FACTOR * COMM_RANGE * np.array([1, -1])
        u = cbf_sdp.solve_sdp(current_positions, u_des, 0)

        for i in range(DRONE_COUNT):
            vel_data[i][t, :2] = u[i,:]
            pos_data[i][t, :2] = pos_data[i][t - 1, :2] + dt * u[i,:]

    animate(time_data, pos_data, COMM_RANGE, DRONE_COUNT, save_sim_animation)

    plot_position(time_data, pos_data)
    plot_input(time_data,vel_data)
    
    eig_data = []
    for t in range(len(pos_data[0])):
        L = create_Laplacian_matrix(np.array([pos[t] for pos in pos_data]),COMM_RANGE)
        eigenvalues, _ = np.linalg.eigh(L)
        eig_data.append(eigenvalues)
    plot_eigenvalues(time_data, eig_data, DRONE_COUNT)
    plt.show()
    

def animate(time_data, pos_data, comm_range, num_drone, save_animation = True):
    # Create figure for animation
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_title("Drone 2D Position (X-Y)")
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.grid(True)

    # Set axis limits based on data range
    all_data = np.vstack(pos_data)
    x_min, x_max = all_data[:, 0].min(), all_data[:, 0].max()
    y_min, y_max = all_data[:, 1].min(), all_data[:, 1].max()
    ax.set_xlim(x_min - 1, x_max + 1)
    ax.set_ylim(y_min - 1, y_max + 1)

    # Initialize scatter plots for each drone
    scatters = [ax.plot([], [], 'o', label=f"Drone {i+1}")[0] for i in range(num_drone)]
    connection_lines = [ax.plot([], [], 'k-', linewidth=1, alpha=0.5)[0] for _ in combinations(range(num_drone), 2)]
    ax.legend()

    # Animation update function
    def update(frame):
        positions = []
        for i, scatter in enumerate(scatters):
            x = [pos_data[i][frame, 0]]
            y = [pos_data[i][frame, 1]]
            scatter.set_data(x, y)
            positions.append((x[0], y[0]))

        # Update lines between close drones
        line_idx = 0
        for i, j in combinations(range(num_drone), 2):
            x1, y1 = positions[i]
            x2, y2 = positions[j]
            dist = np.hypot(x2 - x1, y2 - y1)
            if dist < comm_range:
                connection_lines[line_idx].set_data([x1, x2], [y1, y2])
            else:
                connection_lines[line_idx].set_data([], [])
            line_idx += 1

        return scatters + connection_lines

    
    # Create animation
    ani = animation.FuncAnimation(fig, update, frames=len(time_data), interval=10, blit=True)
    plt.tight_layout()
    plt.show()
    if save_animation:
        print('Saving animation as video. Please wait.')
        writer = animation.FFMpegWriter(fps=60, bitrate=1800)
        ani.save("sim_data/drone_sim.mp4", writer=writer)
        print('Done.')




def plot_position(time_data, pos_data):
    dim_labels = ['x', 'y', 'z']
    y_limits = [(-1.2, 1.2), (-1.2, 1.2), (0, 0.6)]
    fig, axes = plt.subplots(2, 1, sharex=True)

    for dim in range(2):
        ax = axes[dim]
        for i, drone_data in enumerate(pos_data):
            ax.plot(time_data[:, 0], drone_data[:, dim], label=f'Drone {i+1}')
        ax.set_ylabel(rf'Position ${dim_labels[dim]}$ (m)')
        ax.set_xlim(0,10)
        ax.set_ylim(-1.5,1.5)
        ax.grid(True)
        if dim == 0:
            ax.set_title('Drone Position Over Time')
        if dim == 2:
            ax.set_xlabel('Time (s)')
    
    axes[0].legend(loc='upper right')

    plt.tight_layout()

def plot_input(time_data, vel_data):
    dim_labels = ['x', 'y', 'z']
    y_limits = [(-0.8, 0.8), (-0.8, 0.8), (-.6, .6)]
    fig, axes = plt.subplots(2, 1, sharex=True)

    for dim in range(2):
        ax = axes[dim]
        for i, drone_data in enumerate(vel_data):
            ax.plot(time_data[:, 0], drone_data[:, dim], label=f'Drone {i+1}')
        ax.set_ylabel(rf'Control Input ${dim_labels[dim]}$ (m/s)')
        ax.set_xlim(0,10)
        ax.set_ylim(-1.5,1.5)
        ax.grid(True)
        if dim == 0:
            ax.set_title('Control Inputs Over Time')
        if dim == 2:
            ax.set_xlabel('Time (s)')
    
    axes[0].legend(loc='upper right')

    plt.tight_layout()

def create_Laplacian_matrix(positions,comm_range):
    positions = np.array(positions)  # Ensure positions is a NumPy array
    position_difference = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]
    distances_sq = np.sum(position_difference ** 2, axis=2)

    A = np.exp(((1 - distances_sq/comm_range**2)**2) ) - 1 # Compute adjacency values
    A[distances_sq > comm_range ** 2] = 0
    np.fill_diagonal(A, 0)
    D = np.diag(np.sum(A, axis=1))
    return D - A

def plot_eigenvalues(time_data,eig_data,num_drone):
    fig, ax = plt.subplots(1, 1, figsize=(11.5, 2.5))
    eig_data = np.array(eig_data)
    for i in range(num_drone):
        plt.plot(time_data, eig_data[:, i], label=rf'$\phi_{i+1}$')
    
    ax.set_xlim(0,10)
    ax.set_ylim(-0.5, 3.0)
    plt.xlabel(r"Time [ticks]")
    plt.ylabel(r"$\phi_i(L(x))$")
    plt.title(r"Eigenvalues over time")
    plt.grid(True)
    plt.legend()  
    plt.tight_layout()
main()