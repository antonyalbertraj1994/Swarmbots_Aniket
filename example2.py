import sys
import cv2
import numpy as np
from RVO import RVO_update, reach, compute_V_des, reach

###just for fun








def draw_bots(image, positions, radius, color=(255, 0, 255), font=cv2.FONT_HERSHEY_SIMPLEX):
    """ Function to draw the positions of bots or goals on the image. """
    for c, i in enumerate(positions):
        x1 = int(i[0]) - radius
        y1 = int(i[1]) - radius
        x2 = int(i[0]) + radius
        y2 = int(i[1]) + radius
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
        cv2.putText(image, str(c), (x1, y1), font, 1, color, 1, cv2.LINE_AA)
    return image


def update_positions(X, V, step):
    """ Update positions of bots based on current velocities and time step. """
    return [[X[i][0] + V[i][0] * step, X[i][1] + V[i][1] * step] for i in range(len(X))]


def visualize_simulation(X, goal, radius, step, total_time, ws_model, V_max):
    """ Main function to simulate and visualize bot movements with collision avoidance. """
    # Visualization setup
    initial_image = np.zeros((400, 400, 3), dtype=np.uint8)
    goal_image = np.zeros((400, 400, 3), dtype=np.uint8)
    track_image = np.zeros((400, 400, 3), dtype=np.uint8)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Draw initial and goal positions
    initial_image = draw_bots(initial_image, X, 10, (255, 0, 255), font)
    goal_image = draw_bots(goal_image, goal, radius, (255, 0, 0), font)

    # Show initial and goal positions
    cv2.imshow("Initial positions", initial_image)
    cv2.imshow("Goal positions", goal_image)
    cv2.waitKey(0)

    # Initialize velocities
    V = [[0, 0] for _ in range(len(X))]

    # Simulation loop
    t = 0
    while t * step < total_time:
        # Compute desired velocity to goal
        V_des = compute_V_des(X, goal, V_max)

        # Compute the optimal velocity to avoid collision
        V = RVO_update(X, V_des, V, ws_model)

        # Update positions
        X = update_positions(X, V, step)

        # Visualize each step
        track_image[:] = 0  # Clear the track image
        track_image = draw_bots(track_image, X, radius, (255, 0, 255), font)

        # Show the track image
        cv2.imshow("Track Image", track_image)
        cv2.waitKey(1)

        t += 1

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Initial location of 10 bots
    X = [[125, 125], [50, 50], [75, 200], [150, 50], [200, 125],
         [225, 75], [175, 175], [100, 300], [300, 150], [50, 300]]

    # Goal location for 10 bots
    goal = [[200, 200], [350, 350], [300, 50], [100, 100], [275, 275],
            [375, 100], [50, 50], [150, 300], [300, 300], [100, 200]]

    # Maximal velocity norm
    V_max = [40 for _ in range(len(X))]

    # Define workspace model
    ws_model = {
        'robot_radius': 10,
        'robot_dimensions': [(2, 2) for _ in range(10)],
        'circular_obstacles': [],  # No obstacles
        'boundary': []  # No boundary defined
    }

    # Simulation parameters
    total_time = 1000  # Total simulation time (s)
    step = 0.1  # Simulation step

    # Run the simulation
    visualize_simulation(X, goal, ws_model['robot_radius'], step, total_time, ws_model, V_max)
    # Main control loop (pseudo-code)
    num_bots = len(X)  # Number of bots
    start_point = [0, 0]  # Starting point of the line formation
    spacing = 1.5  # Distance between each bot in the line formation

    # Generate the formation points
    formation_points = arrange_in_line(num_bots, start_point, spacing)

    phase = FORMATION_PHASE  # Start with the formation phase

    while True:
        if phase == FORMATION_PHASE:
            # Update velocities to arrange in line
            V_current = RVO_two_phase_update(X, V_des, V_current, ws_model, phase, formation_points)

            # Check if bots have arranged themselves in the line
            if check_formation_complete(X, formation_points):
                phase = DESTINATION_PHASE  # Move to destination phase

        elif phase == DESTINATION_PHASE:
            # Update velocities to move to final destination
            V_current = RVO_two_phase_update(X, V_des, V_current, ws_model, phase)

        # Update bot positions
        X = update_positions(X, V_current)
        visualize_simulation(X, goal, ws_model['robot_radius'], step, total_time, ws_model, V_max)

        # Check for final goal condition (bots reaching their target destinations)
        if all_bots_reached_goal(X, V_des):
            break
