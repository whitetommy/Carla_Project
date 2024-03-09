import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

speed_data = []
brake_data = []
steer_data = []
rpm_data = []

plt.ion()  # interactive mode
fig, axs = plt.subplots(4, 1, figsize=(6, 6))

axs[0].set_title('Speed')
axs[0].set_xlabel('Time')
axs[0].set_ylabel('Speed (m/s)')

axs[1].set_title('Brake')
axs[1].set_xlabel('Time')
axs[1].set_ylabel('Brake Value')

axs[2].set_title('Steer')
axs[2].set_xlabel('Time')
axs[2].set_ylabel('Steer Value')

axs[3].set_title('RPM')
axs[3].set_xlabel('Time')
axs[3].set_ylabel('RPM Value')

def update_graphs(speed, brake, steer, rpm):
    plt.show(block=False)

    speed_data.append(speed)
    brake_data.append(brake)
    steer_data.append(steer)
    rpm_data.append(rpm)

    # Plot speed data
    axs[0].cla()
    axs[0].plot(speed_data)
    axs[0].set_xlabel('Time')
    axs[0].set_ylabel('Speed (m/s)')

    # Plot brake data
    axs[1].cla()
    axs[1].plot(brake_data)
    axs[1].set_xlabel('Time')
    axs[1].set_ylabel('Brake Value')

    # Plot steer data
    axs[2].cla()
    axs[2].plot(steer_data)
    axs[2].set_xlabel('Time')
    axs[2].set_ylabel('Steer Value')

    # Plot throttle data
    axs[3].cla()
    axs[3].plot(rpm_data)
    axs[3].set_xlabel('Time')
    axs[3].set_ylabel('RPM Value')

    plt.tight_layout()
    plt.draw()
    plt.pause(0.001)