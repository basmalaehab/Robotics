import customtkinter as ctk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import messagebox
import time

class RobotSimulator(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Robotics Kinematics")
        self.geometry("1300x950")
        ctk.set_appearance_mode("dark")
        
        # --- DARKER CYAN PALETTE ---
        self.primary_cyan = "#00ced1"
        self.mid_cyan = "#008080"
        self.dark_cyan = "#004d4d"
        self.bg_color = "#0a0f0f"
        self.plot_bg = "#0d1111"

        # State & Physics
        self.L = [60.0, 40.0, 30.0]
        self.limits = [[-90.0, 150.0], [-90.0, 150.0], [-90.0, 150.0]]
        self.elbow_config = 1 
        self.workspace_points = []
        self.full_path_history = [] 

        self.setup_ui()
        # Initialize the start position on the X-axis
        self.reset_to_x_axis()

    def setup_ui(self):
        self.sidebar = ctk.CTkFrame(self, width=350, border_color=self.dark_cyan, border_width=2, fg_color="#121818")
        self.sidebar.pack(side="left", fill="y", padx=10, pady=10)

        ctk.CTkLabel(self.sidebar, text="RRR ROBOT CONTROL", font=("Orbitron", 18, "bold"), text_color=self.primary_cyan).pack(pady=15)

        # Link Lengths
        self.l1_in = self.create_input("Link 1 Length:", str(self.L[0]))
        self.l2_in = self.create_input("Link 2 Length:", str(self.L[1]))
        self.l3_in = self.create_input("Link 3 Length:", str(self.L[2]))

        ctk.CTkLabel(self.sidebar, text="WAYPOINT NAVIGATION", font=("Arial", 12, "bold"), text_color=self.mid_cyan).pack(pady=(15, 5))
        # These will be updated dynamically by reset_to_x_axis
        self.p1_in = self.create_input("Start (X, Y):", "0, 0")
        self.p2_in = self.create_input("End (X, Y):", "50, 180")

        self.config_toggle = ctk.CTkSegmentedButton(self.sidebar, values=["Elbow Up", "Elbow Down"], 
                                                     selected_color=self.mid_cyan, 
                                                     selected_hover_color=self.primary_cyan,
                                                     unselected_color="#1a2424",
                                                     command=self.set_config)
        self.config_toggle.set("Elbow Up")
        self.config_toggle.pack(pady=15)

        self.lim1_min, self.lim1_max = self.create_limit_row("J1 Lim:", "-180", "180")
        self.lim2_min, self.lim2_max = self.create_limit_row("J2 Lim:", "-150", "150")
        self.lim3_min, self.lim3_max = self.create_limit_row("J3 Lim:", "-120", "120")

        self.status_frame = ctk.CTkFrame(self.sidebar, fg_color="#0a0f0f", border_color=self.mid_cyan, border_width=1)
        self.status_frame.pack(fill="x", padx=15, pady=20)
        self.joint_bars, self.joint_labels = [], []
        for i in range(3):
            lbl = ctk.CTkLabel(self.status_frame, text=f"J{i+1}: 0°", font=("Consolas", 11), text_color=self.primary_cyan)
            lbl.pack()
            bar = ctk.CTkProgressBar(self.status_frame, width=220, progress_color=self.mid_cyan, fg_color="#1a2424")
            bar.set(0.5)
            bar.pack(pady=(0, 10))
            self.joint_bars.append(bar)
            self.joint_labels.append(lbl)

        ctk.CTkButton(self.sidebar, text="EXECUTE PATH", fg_color=self.mid_cyan, hover_color=self.primary_cyan, height=40, font=("Arial", 13, "bold"),
                      command=self.animate_trajectory).pack(pady=5, fill="x", padx=20)
        
        ctk.CTkButton(self.sidebar, text="CLEAR TRACE", fg_color="#662222", hover_color="#882222", command=self.clear_history).pack(pady=5, fill="x", padx=20)
        ctk.CTkButton(self.sidebar, text="UPDATE & RESET", fg_color="#222e2e", border_color=self.mid_cyan, border_width=1, command=self.update_params).pack(pady=5, fill="x", padx=20)

        self.fig, self.ax = plt.subplots(figsize=(8, 8), facecolor=self.bg_color)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self)
        self.canvas.get_tk_widget().pack(side="right", fill="both", expand=True)

    def create_input(self, label, default):
        f = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        f.pack(fill="x", padx=20)
        ctk.CTkLabel(f, text=label, font=("Arial", 11), text_color="#aaaaaa").pack(side="left")
        entry = ctk.CTkEntry(f, width=110, border_color=self.dark_cyan, fg_color="#0d1111")
        entry.insert(0, default)
        entry.pack(side="right", pady=2)
        return entry

    def create_limit_row(self, label, d_min, d_max):
        frame = ctk.CTkFrame(self.sidebar, fg_color="transparent")
        frame.pack(pady=2, padx=20, fill="x")
        ctk.CTkLabel(frame, text=label, width=50, font=("Arial", 11), text_color="#aaaaaa").pack(side="left")
        min_in = ctk.CTkEntry(frame, width=70, border_color=self.dark_cyan, fg_color="#0d1111")
        max_in = ctk.CTkEntry(frame, width=70, border_color=self.dark_cyan, fg_color="#0d1111")
        for e, v in zip([min_in, max_in], [d_min, d_max]):
            e.insert(0, v)
            e.pack(side="left", padx=5)
        return min_in, max_in

    def reset_to_x_axis(self):
        """Forces the robot to start horizontally along the X-axis."""
        total_length = sum(self.L)
        self.p1_in.delete(0, 'end')
        self.p1_in.insert(0, f"{total_length}, 0")
        self.draw_scene(angles=[0.0, 0.0, 0.0])

    def update_params(self):
        try:
            self.L = [float(self.l1_in.get()), float(self.l2_in.get()), float(self.l3_in.get())]
            self.limits = [[float(self.lim1_min.get()), float(self.lim1_max.get())],
                           [float(self.lim2_min.get()), float(self.lim2_max.get())],
                           [float(self.lim3_min.get()), float(self.lim3_max.get())]]
            self.calculate_workspace()
            self.reset_to_x_axis()
        except: messagebox.showerror("Error", "Invalid numeric inputs")

    def inverse_kinematics(self, tx, ty):
        phi = np.arctan2(ty, tx) 
        wx, wy = tx - self.L[2]*np.cos(phi), ty - self.L[2]*np.sin(phi)
        d_sq = wx**2 + wy**2
        cos2 = (d_sq - self.L[0]**2 - self.L[1]**2) / (2 * self.L[0] * self.L[1])
        if abs(cos2) > 1.0: return None
        sin2 = self.elbow_config * np.sqrt(1 - cos2**2)
        q2 = np.arctan2(sin2, cos2)
        q1 = np.arctan2(wy, wx) - np.arctan2(self.L[1]*sin2, self.L[0] + self.L[1]*cos2)
        q3 = phi - (q1 + q2)
        angles = np.degrees([q1, q2, q3])
        for i in range(3):
            if not (self.limits[i][0] <= angles[i] <= self.limits[i][1]): return None 
        return angles

    def update_ui_bars(self, angles):
        for i in range(3):
            span = max(1, self.limits[i][1] - self.limits[i][0])
            val = np.clip((angles[i] - self.limits[i][0]) / span, 0, 1)
            self.joint_bars[i].set(val)
            self.joint_labels[i].configure(text=f"J{i+1}: {angles[i]:.1f}°")

    def calculate_workspace(self):
        self.workspace_points = []
        r_max, step = int(sum(self.L)), 7
        for x in range(-r_max, r_max + step, step):
            for y in range(-r_max, r_max + step, step):
                if self.inverse_kinematics(x, y) is not None: 
                    self.workspace_points.append((x, y))

    def draw_scene(self, angles=None):
        self.ax.clear()
        self.ax.set_facecolor(self.plot_bg)
        self.ax.grid(color='#142121', linestyle='-', alpha=0.6)

        # White axes and ticks
        self.ax.tick_params(colors='white', which='both')
        for spine in self.ax.spines.values():
            spine.set_color('white')

        # Workspace points in white
        if self.workspace_points:
            xs, ys = zip(*self.workspace_points)
            self.ax.scatter(xs, ys, s=1.5, color='white', alpha=0.12)

        if self.full_path_history:
            hx, hy = zip(*self.full_path_history)
            self.ax.plot(hx, hy, color=self.primary_cyan, lw=1.5, alpha=0.6, linestyle=":")

        if angles is not None:
            self.update_ui_bars(angles)
            q = np.radians(angles)
            x, y, curr_q = [0], [0], 0
            for i in range(3):
                curr_q += q[i]
                x.append(x[-1] + self.L[i] * np.cos(curr_q))
                y.append(y[-1] + self.L[i] * np.sin(curr_q))

            self.ax.plot(x, y, '-o', color=self.mid_cyan, lw=5, markersize=8, 
                         markerfacecolor='white', markeredgecolor=self.primary_cyan)

            if hasattr(self, '_animating') and self._animating:
                self.full_path_history.append((x[-1], y[-1]))

        limit = sum(self.L) + 30
        self.ax.set_xlim(-limit, limit)
        self.ax.set_ylim(-limit, limit)
        self.ax.set_aspect('equal')
        self.canvas.draw()

    def set_config(self, value):
        self.elbow_config = 1 if value == "Elbow Up" else -1
        p1 = [float(x) for x in self.p1_in.get().split(',')]
        q = self.inverse_kinematics(p1[0], p1[1])
        self.draw_scene(angles=q)

    def clear_history(self):
        self.full_path_history = []
        p1 = [float(x) for x in self.p1_in.get().split(',')]
        q = self.inverse_kinematics(p1[0], p1[1])
        self.draw_scene(angles=q)

    def animate_trajectory(self):
        try:
            p1 = [float(x) for x in self.p1_in.get().split(',')]
            p2 = [float(x) for x in self.p2_in.get().split(',')]
        except: 
            messagebox.showerror("Error", "Format coordinates as: x, y"); return

        self._animating = True
        steps = 50
        path_x, path_y = np.linspace(p1[0], p2[0], steps), np.linspace(p1[1], p2[1], steps)
        
        for i in range(steps):
            q = self.inverse_kinematics(path_x[i], path_y[i])
            if q is not None:
                self.draw_scene(q)
                self.update()
                time.sleep(0.005)
            else:
                self._animating = False
                messagebox.showwarning("Limit Hit", "Out of workspace or joint limits.")
                return

        self._animating = False
        self.p1_in.delete(0, 'end')
        self.p1_in.insert(0, f"{p2[0]}, {p2[1]}")
        self.p2_in.delete(0, 'end')
        self.p2_in.focus() 

if __name__ == "__main__":
    app = RobotSimulator()
    app.mainloop()
