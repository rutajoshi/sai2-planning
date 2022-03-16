import numpy as np
import matplotlib.pyplot as plt

def get_grid_from_file(filename):
    statespace_lo = [0,0]
    statespace_hi = []
    resolution = 0.0
    with open(filename) as file:
        for line in file:
            x, y, resolution = line.strip().split(",")
            x, y, resolution = float(x), float(y), float(resolution)
            statespace_hi = [x, y]
    return [statespace_lo, statespace_hi, resolution]

def get_obstacles_from_file(filename):
    obstacles = []
    centerX, centerY, radius = 0, 0, 0
    with open(filename) as file:
        for line in file:
            centerX, centerY, radius = line.strip().split(",")
            centerX, centerY, radius = float(centerX), float(centerY), float(radius)
            obstacles.append([centerX, centerY, radius])
    return obstacles

def get_path_from_file(filename):
    path = []
    with open(filename) as file:
        for line in file:
            line = line.strip().split(",")
            line = [float(x) for x in line]
            path.append(line)
    path = np.array(path)
    return path

def plot_everything(grid, obstacles, path):
    statespace_lo, statespace_hi, resolution = grid
    fig, ax = plt.subplots(figsize=(7, 7))
    print("Made a figure")

    # Set grid size
    plt.xlim([statespace_lo[0], statespace_hi[0]])
    plt.ylim([statespace_lo[1], statespace_hi[1]])
    print("Set limits")

    # Add obstacles as circular patches
    for obs in obstacles:
        centerX, centerY, radius = obs
        circle = plt.Circle((centerX, centerY), radius, color='blue')
        ax.add_patch(circle)
        print("Added a circle")

    # Plot each point on the path as a single point
    plt.scatter(path[:,0], path[:,1], c='red')
    print("Added path")

    # Show and save (invert axes to be at top left)
    # plt.gca().invert_yaxis()
    plt.savefig("vis.png")
    plt.show()
    print("Saved figure")


def main():
    # 1. get the grid, obstacles, and path from the files
    grid = get_grid_from_file("grid.txt")
    obstacles = get_obstacles_from_file("obstacles.txt")
    path = get_path_from_file("rrt_path.txt")

    # 2. plot everything
    plot_everything(grid, obstacles, path)

main()
