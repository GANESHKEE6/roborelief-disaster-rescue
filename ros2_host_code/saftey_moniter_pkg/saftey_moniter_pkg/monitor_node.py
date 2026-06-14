import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import customtkinter as ctk
import tkinter as tk
import threading
import math
import time
import json

# Set Light Theme
ctk.set_appearance_mode("Light")
ctk.set_default_color_theme("blue")

class ValleyWatchROSNode(Node):
    """ROS2 Node running in the background to handle data publishing."""
    def __init__(self):
        super().__init__('valleywatch_gui_node')
        self.telemetry_pub = self.create_publisher(String, '/valleywatch/telemetry', 10)
        self.event_pub = self.create_publisher(String, '/valleywatch/events', 10)

    def publish_telemetry(self, data):
        msg = String()
        msg.data = json.dumps(data)
        self.telemetry_pub.publish(msg)

    def publish_event(self, event_msg):
        msg = String()
        msg.data = event_msg
        self.event_pub.publish(msg)


class ValleyWatchApp(ctk.CTk):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.title("ValleyWatch // Tactical Command (Light Edition)")
        self.geometry("1100x700")
        
        # System State
        self.base_station = {"x": 300, "y": 200}
        self.nodes = {
            "N-01": {"name": "NW Ridge", "x": 50, "y": 50, "mq3": 15, "temp": 22, "hum": 55, "imu": 0.1, "status": "Safe"},
            "N-02": {"name": "NE Slope", "x": 550, "y": 50, "mq3": 12, "temp": 18, "hum": 60, "imu": 0.2, "status": "Safe"},
            "N-03": {"name": "SW Riverbank", "x": 50, "y": 350, "mq3": 10, "temp": 15, "hum": 45, "imu": 0.1, "status": "Safe"},
            "N-04": {"name": "SE Landslide Zone", "x": 550, "y": 350, "mq3": 14, "temp": 20, "hum": 50, "imu": 0.1, "status": "Safe"}
        }
        self.drone = {
            "status": "Idle", "x": self.base_station["x"], "y": self.base_station["y"],
            "alt": 0.0, "speed": 0.0, "battery": 100.0, "target": None
        }
        self.selected_node = "N-01"

        self.setup_ui()
        self.draw_map()
        
        # Start GUI Loop
        self.update_loop()

    def setup_ui(self):
        # --- Left Panel: Simulation & Sensors ---
        self.left_frame = ctk.CTkFrame(self, width=250, corner_radius=10, fg_color="white")
        self.left_frame.pack(side="left", fill="y", padx=10, pady=10)

        ctk.CTkLabel(self.left_frame, text="SENSOR NETWORK", font=("Arial", 16, "bold")).pack(pady=10)
        
        self.node_buttons = {}
        for nid, data in self.nodes.items():
            btn = ctk.CTkButton(self.left_frame, text=f"{nid} - {data['name']}", 
                                fg_color="#e0e0e0", text_color="black", hover_color="#d0d0d0",
                                command=lambda n=nid: self.select_node(n))
            btn.pack(pady=5, padx=10, fill="x")
            self.node_buttons[nid] = btn

        ctk.CTkLabel(self.left_frame, text="SIMULATION OVERRIDE", font=("Arial", 16, "bold")).pack(pady=(30,10))
        self.target_lbl = ctk.CTkLabel(self.left_frame, text="Target: N-01", text_color="#ff7700", font=("Arial", 14, "bold"))
        self.target_lbl.pack()

        # Sliders
        self.slider_mq3 = self.create_slider(self.left_frame, "MQ3 Gas (ppm)", 0, 100, 15)
        self.slider_imu = self.create_slider(self.left_frame, "IMU Vib (g)", 0.0, 1.0, 0.1)

        # --- Center Panel: Map ---
        self.center_frame = ctk.CTkFrame(self, corner_radius=10, fg_color="#f0f4f8") # Light blue-gray map background
        self.center_frame.pack(side="left", fill="both", expand=True, padx=10, pady=10)
        
        self.canvas = tk.Canvas(self.center_frame, bg="#eef2f3", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=5, pady=5)

        # --- Right Panel: Telemetry & Logs ---
        self.right_frame = ctk.CTkFrame(self, width=250, corner_radius=10, fg_color="white")
        self.right_frame.pack(side="right", fill="y", padx=10, pady=10)

        ctk.CTkLabel(self.right_frame, text="UAV TELEMETRY", font=("Arial", 16, "bold")).pack(pady=10)
        self.lbl_status = ctk.CTkLabel(self.right_frame, text="Status: Idle", font=("Arial", 14))
        self.lbl_status.pack(anchor="w", padx=20)
        self.lbl_alt = ctk.CTkLabel(self.right_frame, text="Altitude: 0.0m", font=("Arial", 14))
        self.lbl_alt.pack(anchor="w", padx=20)
        self.lbl_speed = ctk.CTkLabel(self.right_frame, text="Speed: 0.0km/h", font=("Arial", 14))
        self.lbl_speed.pack(anchor="w", padx=20)
        self.lbl_batt = ctk.CTkLabel(self.right_frame, text="Battery: 100%", font=("Arial", 14))
        self.lbl_batt.pack(anchor="w", padx=20)

        ctk.CTkLabel(self.right_frame, text="EVENT LOG", font=("Arial", 16, "bold")).pack(pady=(30,10))
        self.log_box = ctk.CTkTextbox(self.right_frame, height=300, fg_color="#f9f9f9", text_color="black")
        self.log_box.pack(padx=10, pady=10, fill="both", expand=True)

    def create_slider(self, parent, label_text, min_val, max_val, default_val):
        frame = ctk.CTkFrame(parent, fg_color="transparent")
        frame.pack(fill="x", padx=10, pady=10)
        ctk.CTkLabel(frame, text=label_text).pack(anchor="w")
        slider = ctk.CTkSlider(frame, from_=min_val, to=max_val)
        slider.set(default_val)
        slider.pack(fill="x")
        return slider

    def log_event(self, msg):
        time_str = time.strftime("%H:%M:%S")
        self.log_box.insert("1.0", f"[{time_str}] {msg}\n")
        self.ros_node.publish_event(msg)

    def select_node(self, nid):
        self.selected_node = nid
        self.target_lbl.configure(text=f"Target: {nid}")
        self.slider_mq3.set(self.nodes[nid]["mq3"])
        self.slider_imu.set(self.nodes[nid]["imu"])

    def draw_map(self):
        self.canvas.delete("all")
        
        # Draw River
        self.canvas.create_line(150, 0, 200, 200, 150, 400, fill="#a0c8e0", width=30, smooth=True)
        self.canvas.create_line(450, 0, 400, 200, 450, 400, fill="#a0c8e0", width=30, smooth=True)

        # Draw Base Station
        cx, cy = self.base_station["x"], self.base_station["y"]
        self.canvas.create_rectangle(cx-15, cy-15, cx+15, cy+15, fill="#555555", outline="black")
        self.canvas.create_text(cx, cy+25, text="Command Base", fill="black", font=("Arial", 10, "bold"))

        # Draw Nodes
        self.node_shapes = {}
        for nid, data in self.nodes.items():
            color = "#00cc44" if data["status"] == "Safe" else ("#ffaa00" if data["status"] == "Warning" else "#ff3333")
            shape = self.canvas.create_oval(data["x"]-10, data["y"]-10, data["x"]+10, data["y"]+10, fill=color, outline="black", width=2)
            self.canvas.create_text(data["x"], data["y"]-20, text=nid, fill="black", font=("Arial", 10, "bold"))
            self.node_shapes[nid] = shape

        # Draw Drone
        self.drone_shape = self.canvas.create_polygon(
            self.drone["x"], self.drone["y"]-10, 
            self.drone["x"]-10, self.drone["y"]+10, 
            self.drone["x"]+10, self.drone["y"]+10, 
            fill="#ff7700", outline="black"
        )

    def update_loop(self):
        # Update selected node from sliders
        if self.selected_node:
            self.nodes[self.selected_node]["mq3"] = self.slider_mq3.get()
            self.nodes[self.selected_node]["imu"] = self.slider_imu.get()

        # Risk Logic
        for nid, n in self.nodes.items():
            old_status = n["status"]
            if n["mq3"] > 40 or n["imu"] > 0.4:
                n["status"] = "Critical"
            elif n["mq3"] > 25 or n["imu"] > 0.2:
                n["status"] = "Warning"
            else:
                n["status"] = "Safe"

            # Check if we need to dispatch
            if n["status"] == "Critical" and old_status != "Critical" and self.drone["status"] == "Idle":
                self.drone["status"] = "En Route"
                self.drone["target"] = nid
                self.log_event(f"ALERT! Critical limits at {nid}. Drone Dispatched.")

        # Drone Movement Logic
        if self.drone["status"] == "En Route" and self.drone["target"]:
            tx = self.nodes[self.drone["target"]]["x"]
            ty = self.nodes[self.drone["target"]]["y"]
            dx = tx - self.drone["x"]
            dy = ty - self.drone["y"]
            dist = math.hypot(dx, dy)

            if dist < 5:
                self.drone["status"] = "Monitoring"
                self.drone["speed"] = 0
                self.log_event(f"Drone reached {self.drone['target']}. Commencing surveillance.")
            else:
                speed = 4  # pixels per frame
                self.drone["x"] += (dx / dist) * speed
                self.drone["y"] += (dy / dist) * speed
                self.drone["alt"] = 120.0
                self.drone["speed"] = 45.0
                self.drone["battery"] -= 0.05

        # Return Home Logic
        if self.drone["status"] == "Monitoring":
            # If target node goes back to safe, return home
            if self.nodes[self.drone["target"]]["status"] == "Safe":
                self.log_event(f"Conditions at {self.drone['target']} stabilized. Returning to base.")
                self.drone["status"] = "Returning"
                self.drone["target"] = "Base"

        if self.drone["status"] == "Returning":
            tx, ty = self.base_station["x"], self.base_station["y"]
            dx = tx - self.drone["x"]
            dy = ty - self.drone["y"]
            dist = math.hypot(dx, dy)

            if dist < 5:
                self.drone["status"] = "Idle"
                self.drone["alt"] = 0.0
                self.drone["speed"] = 0.0
                self.drone["target"] = None
                self.log_event("Drone landed safely at base.")
            else:
                speed = 4
                self.drone["x"] += (dx / dist) * speed
                self.drone["y"] += (dy / dist) * speed
                self.drone["speed"] = 45.0
                self.drone["battery"] -= 0.05

        # Update Visuals
        self.draw_map()
        
        # Update Telemetry UI
        self.lbl_status.configure(text=f"Status: {self.drone['status']}")
        self.lbl_alt.configure(text=f"Altitude: {self.drone['alt']:.1f}m")
        self.lbl_speed.configure(text=f"Speed: {self.drone['speed']:.1f}km/h")
        self.lbl_batt.configure(text=f"Battery: {self.drone['battery']:.1f}%")

        # Publish state to ROS
        self.ros_node.publish_telemetry({"nodes": self.nodes, "drone": self.drone})

        # Schedule next frame (50ms ~ 20 FPS)
        self.after(50, self.update_loop)

def main(args=None):
    # 1. Initialize ROS 2
    rclpy.init(args=args)
    ros_node = ValleyWatchROSNode()

    # 2. Start ROS 2 spinning in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    # 3. Start the GUI in the main thread
    app = ValleyWatchApp(ros_node)
    
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Clean up gracefully when window is closed
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()