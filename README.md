# RRR Robot Arm Simulator (3-DOF)

https://github.com/user-attachments/assets/c4ad55dc-f530-4173-b4c6-b39ab480dd07


## 1. Project Overview
This project is a **3-DOF planar RRR robotic arm simulator** implemented in **Python**, using **CustomTkinter** for the graphical user interface and **Matplotlib** for visualization.

The simulator allows the user to:
- Define link lengths and joint limits
- Select elbow-up or elbow-down configurations
- Compute and visualize the robot workspace
- Perform inverse kinematics
- Animate a straight-line trajectory between two points
- Visualize joint angles and end-effector path history

The application is intended for **educational purposes** in robotics and kinematics courses.

---

## 2. Required Python Libraries

| Library          | Purpose                          |
|------------------|----------------------------------|
| customtkinter    | Modern GUI framework             |
| numpy            | Numerical computations           |
| matplotlib       | Plotting and animation           |
| tkinter          | Built-in GUI backend (Python)    |

---

## 3. Virtual Environment Setup
Using a virtual environment is recommended to isolate dependencies.

### Create a Virtual Environment
```bash
python -m venv robot_env
```
### Activate the Environment
- Windows
 ```bash
robot_env\Scripts\activate
```

- Linux
 ```bash
source robot_env/bin/activate
```
  
## 4. Installing Dependencies
Install the required packages using pip:
```bash
pip install customtkinter numpy matplotlib
```

## 5. Running the Application
Execute the program using:
```bash
python RRR_robot.py
```

## NOTES
- The robot starts fully extended along the X-axis, with all links aligned in a straight horizontal configuration.
- If a target position lies outside the robot workspace or exceeds joint limits, an error message will appear:
"Out of workspace or Joint Limits"
