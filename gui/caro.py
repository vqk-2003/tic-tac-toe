import numpy as np
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

mcu = serial.Serial('COM29', 115200, timeout=50)

fig = plt.figure()
ax = fig.add_subplot(111)

def animate(i):
    data = mcu.readline().decode('utf-8')

    ax.clear()
    ax.set_xlim(-0.5, 2.5)
    ax.set_ylim(-0.5, 2.5)
    ax.set_xticks(np.arange(3))
    ax.set_yticks(np.arange(3))
    for i in range(len(data) - 2):
        if data[i] == 'x':
            ax.text(int(i/3), int(i%3), 'X', fontsize=20, ha='center', va='center', color='red')
        elif data[i] == 'o':
            ax.text(int(i/3), int(i%3), 'O', fontsize=20, ha='center', va='center', color='blue')
    ax.text(int(int(data[9])/3), int(data[9])%3, '[ ]', fontsize=20, ha='center', va='center', color='green')
    ax.grid(True)

ani = FuncAnimation(fig, animate, frames=100, interval=20)
plt.show()