import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Environment setup
GRID_SIZE = (50, 50, 30)
START = np.array([0, 0, 0])
GOAL = np.array([49, 49, 29])
NUM_WAYPOINTS = 20
NUM_PARTICLES = 30
NUM_ITERATIONS = 100
OBSTACLE_RADIUS = 2

# Create a few moving spherical obstacles (defined path)
class DynamicObstacle:
    def __init__(self, center, velocity):
        self.center = np.array(center)
        self.velocity = np.array(velocity)

    def move(self):
        self.center += self.velocity
        for i in range(3):
            if not (0 <= self.center[i] <= GRID_SIZE[i]):
                self.velocity[i] *= -1
                self.center[i] += self.velocity[i]

obstacles = [
    DynamicObstacle([20.0, 25, 15], [0.0, 0, 0]),
    DynamicObstacle([35.0, 10, 10], [-0.0, 0, 0]),
    DynamicObstacle([10.0, 25, 5], [0, 0.3, 0]),
    DynamicObstacle([0.0, 50, 10], [-0, 0, 0.0])
]

def is_collision(point):
    for obs in obstacles:
        if np.linalg.norm(point - obs.center) < OBSTACLE_RADIUS:
            return True
    return False

def path_length(path):
    return np.sum([np.linalg.norm(path[i+1] - path[i]) for i in range(len(path)-1)])

def smoothness_penalty(path):
    penalty = 0
    for i in range(1, len(path)-1):
        v1 = path[i] - path[i-1]
        v2 = path[i+1] - path[i]
        angle = np.arccos(np.clip(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2) + 1e-8), -1.0, 1.0))
        penalty += angle
    return penalty

def obstacle_penalty(path):
    return sum(is_collision(p) for p in path) * 1000

def fitness(path):
    return path_length(path) + smoothness_penalty(path) + obstacle_penalty(path)

# Initialize swarm
particles = [np.linspace(START, GOAL, NUM_WAYPOINTS) + np.random.randn(NUM_WAYPOINTS, 3) * 2 for _ in range(NUM_PARTICLES)]
velocities = [np.zeros_like(p) for p in particles]
pBests = list(particles)
pBestScores = [fitness(p) for p in particles]
gBest = pBests[np.argmin(pBestScores)]
gBestScore = min(pBestScores)

# PSO params
w, c1, c2 = 0.5, 1.5, 1.5

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
paths_plot = [ax.plot([], [], [], 'gray', alpha=0.4)[0] for _ in range(NUM_PARTICLES)]
best_plot, = ax.plot([], [], [], 'r', linewidth=3)
obstacle_scatters = [ax.plot([], [], [], 'bo')[0] for _ in obstacles]

def update(frame):
    global gBest, gBestScore, particles, velocities, pBests, pBestScores
    ax.clear()
    ax.set_xlim(0, GRID_SIZE[0])
    ax.set_ylim(0, GRID_SIZE[1])
    ax.set_zlim(0, GRID_SIZE[2])
    ax.set_title(f"Iteration {frame}")

    # Move obstacles
    for obs in obstacles:
        obs.move()

    for i in range(NUM_PARTICLES):
        r1, r2 = np.random.rand(NUM_WAYPOINTS, 3), np.random.rand(NUM_WAYPOINTS, 3)
        velocities[i] = (w * velocities[i] +
                         c1 * r1 * (pBests[i] - particles[i]) +
                         c2 * r2 * (gBest - particles[i]))
        particles[i] += velocities[i]
        particles[i][0] = START  # Keep start and goal fixed
        particles[i][-1] = GOAL

        f = fitness(particles[i])
        if f < pBestScores[i]:
            pBests[i] = particles[i].copy()
            pBestScores[i] = f
        if f < gBestScore:
            gBest = particles[i].copy()
            gBestScore = f

    for i, p in enumerate(particles):
        paths_plot[i] = ax.plot(p[:,0], p[:,1], p[:,2], 'gray', alpha=0.4)[0]
    best_plot = ax.plot(gBest[:,0], gBest[:,1], gBest[:,2], 'r', linewidth=3)[0]

    for i, obs in enumerate(obstacles):
        u, v = np.mgrid[0:2*np.pi:8j, 0:np.pi:5j]
        x = OBSTACLE_RADIUS * np.cos(u)*np.sin(v) + obs.center[0]
        y = OBSTACLE_RADIUS * np.sin(u)*np.sin(v) + obs.center[1]
        z = OBSTACLE_RADIUS * np.cos(v) + obs.center[2]
        ax.plot_surface(x, y, z, color='blue', alpha=0.3)

ani = FuncAnimation(fig, update, frames=NUM_ITERATIONS, interval=500, repeat=False)
plt.show()
