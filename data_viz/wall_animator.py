#! /usr/bin/python3

import numpy as np
import pandas as pd
import json
import pickle as pkl
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os


ped_positions = []

num_peds = int(os.popen("grep 'int num_peds' main.cc | tr -d ';' | awk -F ' ' '{print $4}'").read())

for i in range(num_peds):
    with open(f"../src/trial_sim/p{i}_traj.json") as f:
        dat = json.load(f)

    pos = dat["positions"]
    pos = [np.array(p.split()) for p in pos]
    pos = [p.astype(float) for p in pos]

    ped_positions.append(pos)


fig, ax = plt.subplots(figsize=(12,8))

steps = len(ped_positions[0])


def animate(i):
    ax.clear()
    count = 0
    for pos in ped_positions:
        if (count % 2 == 0):
            col = "red"
        else:
            col = "blue"
        count += 1
        ax.set_xlim(-5,55)
        ax.set_ylim(-5,30)
        ax.scatter(pos[i][0],pos[i][1],c=col)
        ax.plot([0,50],[-1,-1],"k-")
        ax.plot([0,50],[25,25],"k-")
        ax.plot([25,25],[25,15], "k-")
        ax.plot([25,25],[10,-1],"k-")

ani = FuncAnimation(fig, animate, frames=steps,
                    interval=5, repeat=False)

plt.show()
