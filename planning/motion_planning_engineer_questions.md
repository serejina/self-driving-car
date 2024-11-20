# Q: Explain some of the different techniques used for global planning (such as at a city map level) vs. micro planning (such as planning on how to go from the entrance of a parking lot to a desired space). Show how you could implement techniques from these levels into one end-to-end algorithm to guide a mobile robot from one distant location to another.

## Global planning (such as at a city map level) vs. micro planning (such as planning on how to go from the entrance of a parking lot to a desired space):

Global planning focuses on generating a high-level, long-distance path from a start location to a distant goal, usually across a city/large-scale environment. Common techniques include:
- Graph-based algorithms: Algorithms like Dijkstra's or A* use a pre-defined graph of the map where nodes represent waypoints, and edges represent possible routes. A heuristic (e.g., Euclidean distance) improves efficiency in A*.
- Probabilistic Roadmaps (PRM): Useful in environments where graphs are not predefined. PRM samples random points in the environment, connects them into a graph, and searches for the shortest path.
- Cost Maps: Cost-based planning takes into account traffic, terrain, or other dynamic factors to find an optimized route.\
Additional techniques may be employed for global planning based on the environment's complexity and specific requirements.

## Micro planning (such as planning on how to go from the entrance of a parking lot to a desired space):
Micro planning deals with short-range, fine-tuned path adjustments to navigate specific environments, like a parking lot or crowded street. Techniques include:

- Dynamic Window Approach (DWA): A local trajectory planner that considers the robot's kinematics and real-time obstacle avoidance within its velocity limits.
- Model Predictive Control (MPC): Generates a trajectory by solving an optimization problem over a short time horizon, balancing control inputs and constraints like avoiding obstacles or staying on track.
- Spline-Based Planning: Smooth path generation for precision tasks, such as aligning to a parking space.

## Show how you could implement techniques from these levels into one end-to-end algorithm to guide a mobile robot from one distant location to another.
The core idea is to integrate global and micro planning into a cohesive end-to-end algorithm. This approach ensures efficient, adaptive navigation for a mobile robot, even in complex environments. Below is an explanation of the components, along with diagrams illustrating the process:
1. Global Planning:
A global planner, such as A*, is used to compute an initial path on a city-scale road network. This step involves:
    - Representing the environment as a graph or high-level grid.
    - Calculating a path from the start to the destination while factoring in terrain, traffic, and other constraints.
    - Segmenting the global path into waypoints, which act as intermediate goals for the local planner.
2. Local Path Planning:
    - Use the Dynamic Window Approach (DWA) to navigate towards the next waypoint while avoiding dynamic obstacles like pedestrians or other vehicles.
    - For precise navigation in confined spaces (e.g., parking lots or narrow lanes), transition to Model Predictive Control (MPC), which ensures smooth and accurate maneuvering.
3. End-to-End Integration: The global and local planners are integrated into a hierarchical system:
    - The global planner provides high-level guidance by defining waypoints.
    - The local planner ensures safe, real-time adjustments to follow the global path while adapting to dynamic changes in the environment.
4. Feedback Mechanism: Sensors like LiDAR, cameras, and GPS continuously monitor the environment. If obstacles or deviations render the current path invalid, the global planner is re-triggered to replan the route dynamically.\
Diagrams below illustrate the end-to-end algorithm and its hierarchical structure:
![End-to-End Algo 1](./imgs/q_1.png)
![End-to-End Algo 2](./imgs/q1_1.png)

# Q: How do you would generate a desired trajectory over a fixed time horizon for a robot traveling at high speed? Consider what information you would need in order to determine a reasonable trajectory, as well as any additional constraints you may need to consider.
To generate a desired trajectory over a fixed time horizon for a robot traveling at high speed, the following steps and considerations are essential:
1. Information Required
To determine a reasonable trajectory, gather the following inputs:
    - Start and Goal States:
        - Initial position, velocity, and orientation of the robot.
        - Desired endpoint and final velocity within the fixed time horizon.
    - Environment Data:
        - Static and dynamic obstacles in the robot's path.
        - Map of the environment (e.g., grid map, occupancy map).
    - Robot Dynamics and Kinematics:
        - Maximum velocity, acceleration, and deceleration limits.- - Turning radius and motion model (e.g., bicycle model, differential drive).
    - External Conditions:
        - Road friction, wind, or other external forces that may impact motion.
    - Time Horizon:
        - The fixed time period within which the trajectory must be completed.
2. Generating the Trajectory
    - Planning Techniques
        - Use MPC:
            - MPC optimizes the trajectory over a rolling time horizon by solving a cost function while considering constraints.
            - Cost function may include terms for minimizing deviation from the goal, energy consumption, and maintaining safety margins.
        - Polynomial Trajectory Generation:
            - Generate smooth trajectories using splines or quintic polynomials that satisfy boundary conditions (position, velocity, and acceleration at start and goal).
    - Discretization
        - Divide the time horizon into small intervals. For each interval, compute intermediate states using the selected planning algorithm.
        - Ensure continuity of position, velocity, and acceleration across intervals.
3. Constraints to Consider
    - Dynamic Constraints:
        - Respect robot kinematics (e.g., maximum turning angle or angular velocity).
    - Maintain feasible accelerations to prevent slipping or instability.
    - Collision Avoidance:
        - Ensure the trajectory keeps the robot at a safe distance from static and moving obstacles.
        - Use real-time sensor data for updates if the environment is dynamic.
    - Time Constraints:
        - Ensure the trajectory can be executed within the fixed time horizon.
    - Control Limitations:
        - Consider actuator delays and limits when designing feasible control inputs.
4. Real-Time Adjustments
    - Continuously monitor the environment using sensors (e.g., LiDAR, cameras).
    - Implement a feedback mechanism to adapt the trajectory dynamically if obstacles or unforeseen changes occur.
    - Recompute or refine the trajectory over a shorter time horizon if deviations are detected.
    
Workflow is presented below:
![desired-trajectory](./imgs/q2_1.png)

# Q: [Code] Explain the concepts behind a rapidly-exploring random tree (RRT), and implement an example in code.
The RRT algorithm is a popular technique for finding a path from a start state to a goal state in high-dimensional or complex spaces. Here's a breakdown of the key concepts:

Core Idea:
- RRT incrementally builds a tree rooted at the start position by randomly sampling the search space.
- The algorithm biases exploration towards unexplored regions, making it efficient in high-dimensional spaces.

Steps:
- Sampling: Randomly sample a point in the search space.
- Nearest Neighbor: Find the closest node in the current tree to the sampled point.
- Extension: Extend the tree by moving a small step from the nearest neighbor towards the sampled point.
- Goal Check: If the extended tree reaches the goal region, terminate the search.

Advantages:
- Efficiently explores large or complex spaces.
- Handles high-dimensional planning problems effectively.

Constraints:
- The algorithm can be adapted to consider obstacles by checking for collisions during the extension step.

## Code
```python
import numpy as np
import matplotlib.pyplot as plt

class RRT:
    def __init__(self, start, goal, map_limits, step_size, max_iterations):
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.map_limits = map_limits
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.tree = [self.start]  # Tree starts with the root

    def distance(self, p1, p2):
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def nearest_node(self, sample):
        distances = [self.distance(node, sample) for node in self.tree]
        nearest_index = np.argmin(distances)
        return self.tree[nearest_index]

    def steer(self, from_node, to_node):
        direction = np.array(to_node) - np.array(from_node)
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = direction / distance * self.step_size
        return from_node + direction

    def plan(self):
        for _ in range(self.max_iterations):
            # Sample a random point
            random_sample = np.random.uniform(
                self.map_limits[0], self.map_limits[1], size=2
            )
            # Find the nearest node in the tree
            nearest_node = self.nearest_node(random_sample)
            # Steer towards the sample point
            new_node = self.steer(nearest_node, random_sample)
            self.tree.append(new_node)
            # Check if the goal is reached
            if self.distance(new_node, self.goal) <= self.step_size:
                self.tree.append(self.goal)
                return self.tree
        print("Max iterations reached without finding a path.")
        return self.tree

# Define parameters
start = (0, 0)
goal = (8, 8)
map_limits = [(-1, -1), (10, 10)]  # (min, max) for x and y
step_size = 0.5
max_iterations = 1000

# Create and run RRT
rrt = RRT(start, goal, map_limits, step_size, max_iterations)
tree = rrt.plan()
```
To otmaze the path RRT* can be used.
![desired-trajectory](./imgs/q3.png)