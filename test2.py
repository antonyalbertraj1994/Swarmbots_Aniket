import sys
import tkinter as tk
from tkinter import simpledialog, messagebox
import numpy as np
from RVO import RVO_update, compute_V_des


class BotSimulationApp:
    def __init__(self, root):
        self.root = root
        self.canvas_width = 400
        self.canvas_height = 400
        self.bot_size = 20  # Define the size of the square bots
        self.goal_size = 20  # Define the size of the square goals
        self.goal_positions = []

        # Initialize simulation parameters
        self.num_bots = 0
        self.bots_positions = []
        self.V_max = []
        self.ws_model = {
            'robot_radius': self.bot_size // 2,
            'robot_dimensions': [(2, 2) for _ in range(10)],
            'circular_obstacles': [],
            'boundary': []
        }

        # Setup UI
        self.setup_ui()

        # Track mouse position for transparent square
        self.mouse_x, self.mouse_y = None, None
        self.canvas_goals.bind("<Motion>", self.on_mouse_move)

    def setup_ui(self):
        """Set up the Tkinter interface."""
        self.canvas_goals = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg='white')
        self.canvas_goals.grid(row=0, column=0, padx=10, pady=10)
        self.canvas_bots = tk.Canvas(self.root, width=self.canvas_width, height=self.canvas_height, bg='white')
        self.canvas_bots.grid(row=0, column=1, padx=10, pady=10)

        # Buttons
        btn_set_bots = tk.Button(self.root, text="Set Bots", command=self.set_bots)
        btn_set_bots.grid(row=1, column=0, pady=10)

        btn_start_simulation = tk.Button(self.root, text="Start Simulation", command=self.start_simulation)
        btn_start_simulation.grid(row=1, column=1, pady=10)

        btn_reset = tk.Button(self.root, text="Reset", command=self.reset_simulation)
        btn_reset.grid(row=2, column=0, columnspan=2, pady=10)

        # Bind click events
        self.canvas_goals.bind("<Button-1>", self.on_click_set_goal)

    def set_bots(self):
        """Prompt user to input number of bots."""
        self.num_bots = simpledialog.askinteger("Input", "Enter number of bots:")
        self.V_max = [40 for _ in range(self.num_bots)]

        # Arrange bots in a line on the right canvas
        self.bots_positions = self.arrange_in_line()
        self.draw_bots_in_line()

    def arrange_in_line(self):
        """Arrange bots in a straight horizontal line on the second canvas."""
        bot_spacing = self.canvas_width // (self.num_bots + 1)  # Spacing between bots
        y_position = self.canvas_height // 2  # Middle of the canvas

        return [[(i + 1) * bot_spacing, y_position] for i in range(self.num_bots)]

    def draw_bots_in_line(self):
        """Draw the bots arranged in a line on the second (right) canvas."""
        self.canvas_bots.delete("all")  # Clear canvas before drawing
        for bot_id, pos in enumerate(self.bots_positions):
            x, y = pos
            self.canvas_bots.create_rectangle(x - self.bot_size // 2, y - self.bot_size // 2,
                                              x + self.bot_size // 2, y + self.bot_size // 2, fill='magenta')

            # Add the bot ID next to the square
            self.canvas_bots.create_text(x - self.bot_size // 2 - 10, y - self.bot_size // 2 - 10,
                                         text=str(bot_id), fill='black', font=('Helvetica', 10))

    def on_mouse_move(self, event):
        """Update the mouse position and display a transparent square."""
        self.mouse_x, self.mouse_y = event.x, event.y
        self.canvas_goals.delete("preview")  # Remove previous preview
        self.canvas_goals.create_rectangle(self.mouse_x - self.goal_size // 2, self.mouse_y - self.goal_size // 2,
                                           self.mouse_x + self.goal_size // 2, self.mouse_y + self.goal_size // 2,
                                           outline='gray', dash=(2, 2), tags="preview")

    def on_click_set_goal(self, event):
        """Set goal positions on the left (first) canvas with intersection check."""
        x, y = event.x, event.y

        # Check for intersection with existing goals
        if self.is_intersecting(x, y):
            self.flash_red_warning(x, y)
            messagebox.showwarning("Warning", "Goal intersects with another square!")
            return

        # Add goal position and draw the square
        self.goal_positions.append([x, y])
        self.canvas_goals.create_rectangle(x - self.goal_size // 2, y - self.goal_size // 2,
                                           x + self.goal_size // 2, y + self.goal_size // 2, fill='blue')

    def is_intersecting(self, x, y):
        """Check if a new square at (x, y) would intersect with any existing squares."""
        for gx, gy in self.goal_positions:
            if abs(gx - x) < self.goal_size and abs(gy - y) < self.goal_size:
                return True
        return False

    def flash_red_warning(self, x, y):
        """Flash a red square when an intersection is detected."""
        self.canvas_goals.create_rectangle(x - self.goal_size // 2, y - self.goal_size // 2,
                                           x + self.goal_size // 2, y + self.goal_size // 2,
                                           fill='red', tags="warning")
        self.root.after(500, lambda: self.canvas_goals.delete("warning"))  # Remove warning after 500ms

    def start_simulation(self):
        """Run the bot simulation after setting up initial and goal positions."""
        if len(self.bots_positions) == self.num_bots and len(self.goal_positions) == self.num_bots:
            self.run_simulation()

    def reset_simulation(self):
        """Reset the canvas and clear the positions of bots and goals."""
        # Clear the canvas
        self.canvas_bots.delete("all")
        self.canvas_goals.delete("all")

        # Reset positions and velocity
        self.bots_positions = []
        self.goal_positions = []
        self.num_bots = 0
        self.V_max = []

        # Notify the user that the reset is complete
        messagebox.showinfo("Reset Complete", "The simulation has been reset.")

    def run_simulation(self):
        """Run the simulation and visualize bot movements."""
        X = self.bots_positions
        goal = self.goal_positions
        total_time = 1000
        step = 0.1

        # Initialize velocities
        V = [[0, 0] for _ in range(len(X))]

        # Simulation loop
        t = 0
        while t * step < total_time:
            # Compute desired velocity to goal
            V_des = compute_V_des(X, goal, self.V_max)

            # Compute the optimal velocity to avoid collision
            V = RVO_update(X, V_des, V, self.ws_model)

            # Update positions
            X = update_positions(X, V, step)

            # Visualize each step
            self.canvas_bots.delete("all")
            for bot_id, pos in enumerate(X):
                # Draw the bot square in the new position
                self.canvas_bots.create_rectangle(pos[0] - self.bot_size // 2, pos[1] - self.bot_size // 2,
                                                  pos[0] + self.bot_size // 2, pos[1] + self.bot_size // 2, fill='magenta')

                # Add the bot ID next to the square
                self.canvas_bots.create_text(pos[0] - self.bot_size // 2 - 10, pos[1] - self.bot_size // 2 - 10,
                                             text=str(bot_id), fill='black', font=('Helvetica', 10))

            self.root.update()
            t += 1

        # Simulation complete
        messagebox.showinfo("Simulation Complete", "All bots reached their goals.")


def update_positions(X, V, step):
    """Update positions of bots based on current velocities and time step."""
    return [[X[i][0] + V[i][0] * step, X[i][1] + V[i][1] * step] for i in range(len(X))]


def main():
    """Main entry point for the application."""
    root = tk.Tk()
    root.title("Bot Simulation")
    app = BotSimulationApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
