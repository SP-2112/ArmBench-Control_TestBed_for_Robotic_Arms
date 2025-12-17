"""GUI module for arm-bench
Provides mode selection and configuration interface
"""
import os
import sys

# CRITICAL: Clean Qt environment BEFORE importing any GUI libraries
# This prevents segmentation faults caused by Qt plugin conflicts
if 'QT_PLUGIN_PATH' in os.environ:
    del os.environ['QT_PLUGIN_PATH']
if 'QT_QPA_PLATFORM_PLUGIN_PATH' in os.environ:
    del os.environ['QT_QPA_PLATFORM_PLUGIN_PATH']

# Set Qt platform to xcb for Linux
os.environ['QT_QPA_PLATFORM'] = 'xcb'

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
from typing import Callable, Optional, Dict, Any
import threading
import time
import json
from datetime import datetime

# DO NOT import OpenCV here - delay until needed to avoid Qt conflicts
# OpenCV will be imported lazily when camera features are used
OPENCV_AVAILABLE = False
cv2 = None
Image = None
ImageTk = None

def _lazy_import_opencv():
    """Lazy import of OpenCV to avoid conflicts at startup"""
    global OPENCV_AVAILABLE, cv2, Image, ImageTk
    if cv2 is None:
        try:
            import cv2 as cv2_module
            from PIL import Image as PIL_Image, ImageTk as PIL_ImageTk
            cv2 = cv2_module
            Image = PIL_Image
            ImageTk = PIL_ImageTk
            OPENCV_AVAILABLE = True
        except ImportError:
            OPENCV_AVAILABLE = False
    return OPENCV_AVAILABLE


# Interbotix arm configurations
INTERBOTIX_ARMS = {
    "vx300s": {
        "name": "ViperX 300 6DOF",
        "dof": 6,
        "motors": 9,  # 6 arm + 1 wrist rotate + 2 gripper
        "description": "High-performance 6DOF arm with 300mm reach",
        "motor_ids": list(range(1, 10))
    },
    "vx300": {
        "name": "ViperX 300s 5DOF",
        "dof": 5,
        "motors": 8,
        "description": "5DOF variant of ViperX 300",
        "motor_ids": list(range(1, 9))
    },
    "wx250": {
        "name": "WidowX 250",
        "dof": 5,
        "motors": 7,
        "description": "Compact 5DOF arm with 250mm reach",
        "motor_ids": list(range(1, 8))
    },
    "wx250s": {
        "name": "WidowX 250 6DOF",
        "dof": 6,
        "motors": 8,
        "description": "6DOF variant of WidowX 250",
        "motor_ids": list(range(1, 9))
    },
    "rx150": {
        "name": "ReactorX 150",
        "dof": 5,
        "motors": 6,
        "description": "Entry-level 5DOF arm",
        "motor_ids": list(range(1, 7))
    },
    "rx200": {
        "name": "ReactorX 200",
        "dof": 5,
        "motors": 7,
        "description": "5DOF ReactorX with 200mm reach",
        "motor_ids": list(range(1, 8))
    }
}


class ArmSelectionGUI:
    """GUI for selecting Interbotix arm type"""
    
    def __init__(self, callback: Optional[Callable] = None):
        self.root = tk.Tk()
        self.root.title("arm-bench - Arm Selection")
        self.root.geometry("700x500")
        self.root.configure(bg="#1e1e1e")
        self.selected_arm = None
        self.callback = callback
        
        self._setup_ui()
        
    def _setup_ui(self):
        """Setup the UI components"""
        # Title
        title = tk.Label(
            self.root, 
            text="Select Interbotix Arm Type",
            font=("Helvetica", 20, "bold"),
            bg="#1e1e1e",
            fg="white"
        )
        title.pack(pady=20)
        
        # Description
        desc = tk.Label(
            self.root,
            text="Choose the robot arm you are using:",
            font=("Helvetica", 12),
            bg="#1e1e1e",
            fg="#aaaaaa"
        )
        desc.pack(pady=5)
        
        # Arm buttons frame
        button_frame = tk.Frame(self.root, bg="#1e1e1e")
        button_frame.pack(pady=20, padx=20, fill='both', expand=True)
        
        # Create buttons for each arm type
        row = 0
        col = 0
        for arm_key, arm_info in INTERBOTIX_ARMS.items():
            arm_btn = tk.Button(
                button_frame,
                text=f"{arm_info['name']}\n({arm_info['dof']} DOF, {arm_info['motors']} motors)",
                command=lambda k=arm_key: self._select_arm(k),
                width=25,
                height=4,
                bg="#2d2d2d",
                fg="white",
                font=("Helvetica", 10),
                activebackground="#007acc",
                activeforeground="white",
                relief=tk.FLAT,
                borderwidth=2
            )
            arm_btn.grid(row=row, column=col, padx=10, pady=10, sticky='nsew')
            
            # Hover effects
            arm_btn.bind('<Enter>', lambda e, b=arm_btn: b.config(bg="#007acc"))
            arm_btn.bind('<Leave>', lambda e, b=arm_btn: b.config(bg="#2d2d2d"))
            
            col += 1
            if col >= 3:
                col = 0
                row += 1
                
        # Configure grid weights
        for i in range(3):
            button_frame.grid_columnconfigure(i, weight=1)
        for i in range(row + 1):
            button_frame.grid_rowconfigure(i, weight=1)
        
    def _select_arm(self, arm_key: str):
        """Handle arm selection"""
        self.selected_arm = arm_key
        arm_info = INTERBOTIX_ARMS[arm_key]
        print(f"Selected arm: {arm_info['name']}")
        
        if self.callback:
            self.callback(arm_key)
        
        self.root.destroy()
        
    def show(self) -> str:
        """Show the GUI and return selected arm"""
        self.root.mainloop()
        return self.selected_arm


class OperationModeGUI:
    """GUI for selecting operation mode: Bimanual or Bilateral"""
    
    def __init__(self, arm_type: str, callback: Optional[Callable] = None):
        self.arm_type = arm_type
        self.arm_info = INTERBOTIX_ARMS.get(arm_type, {})
        self.root = tk.Tk()
        self.root.title("arm-bench - Operation Mode")
        self.root.geometry("800x600")
        self.root.configure(bg="#1e1e1e")
        self.selected_config = None
        self.callback = callback
        
        self._setup_ui()
        
    def _setup_ui(self):
        """Setup the UI components"""
        # Title
        title = tk.Label(
            self.root, 
            text="Select Operation Mode",
            font=("Helvetica", 20, "bold"),
            bg="#1e1e1e",
            fg="white"
        )
        title.pack(pady=15)
        
        # Arm info
        arm_name = self.arm_info.get('name', self.arm_type)
        arm_label = tk.Label(
            self.root,
            text=f"Arm: {arm_name}",
            font=("Helvetica", 12),
            bg="#1e1e1e",
            fg="#4ec9b0"
        )
        arm_label.pack(pady=5)
        
        # Mode selection frame
        mode_frame = tk.Frame(self.root, bg="#1e1e1e")
        mode_frame.pack(pady=20, padx=20, fill='both', expand=True)
        
        # Bilateral mode (1 leader, 1 follower)
        bilateral_frame = tk.LabelFrame(
            mode_frame,
            text="Bilateral Teleoperation",
            font=("Helvetica", 14, "bold"),
            bg="#2d2d2d",
            fg="white",
            padx=20,
            pady=20
        )
        bilateral_frame.pack(fill='x', pady=10)
        
        tk.Label(
            bilateral_frame,
            text="1 Leader arm controls 1 Follower arm",
            font=("Helvetica", 11),
            bg="#2d2d2d",
            fg="#aaaaaa"
        ).pack(anchor='w')
        
        bilateral_btn_frame = tk.Frame(bilateral_frame, bg="#2d2d2d")
        bilateral_btn_frame.pack(fill='x', pady=10)
        
        # Bilateral options
        self._create_mode_button(bilateral_btn_frame, "Both Hardware", 
                                  {'mode': 'bilateral', 'leader': 'hardware', 'follower': 'hardware'},
                                  "#4CAF50", 0)
        self._create_mode_button(bilateral_btn_frame, "Both Simulation", 
                                  {'mode': 'bilateral', 'leader': 'simulation', 'follower': 'simulation'},
                                  "#2196F3", 1)
        self._create_mode_button(bilateral_btn_frame, "Leader HW / Follower Sim", 
                                  {'mode': 'bilateral', 'leader': 'hardware', 'follower': 'simulation'},
                                  "#FF9800", 2)
        self._create_mode_button(bilateral_btn_frame, "Leader Sim / Follower HW", 
                                  {'mode': 'bilateral', 'leader': 'simulation', 'follower': 'hardware'},
                                  "#9C27B0", 3)
        
        # Bimanual mode (2 leaders, 2 followers)
        bimanual_frame = tk.LabelFrame(
            mode_frame,
            text="Bimanual Teleoperation",
            font=("Helvetica", 14, "bold"),
            bg="#2d2d2d",
            fg="white",
            padx=20,
            pady=20
        )
        bimanual_frame.pack(fill='x', pady=10)
        
        tk.Label(
            bimanual_frame,
            text="2 Leader arms control 2 Follower arms (ALOHA-style)",
            font=("Helvetica", 11),
            bg="#2d2d2d",
            fg="#aaaaaa"
        ).pack(anchor='w')
        
        bimanual_btn_frame = tk.Frame(bimanual_frame, bg="#2d2d2d")
        bimanual_btn_frame.pack(fill='x', pady=10)
        
        # Bimanual options
        self._create_mode_button(bimanual_btn_frame, "All Hardware", 
                                  {'mode': 'bimanual', 'leaders': 'hardware', 'followers': 'hardware'},
                                  "#4CAF50", 0)
        self._create_mode_button(bimanual_btn_frame, "All Simulation", 
                                  {'mode': 'bimanual', 'leaders': 'simulation', 'followers': 'simulation'},
                                  "#2196F3", 1)
        self._create_mode_button(bimanual_btn_frame, "Leaders HW / Followers Sim", 
                                  {'mode': 'bimanual', 'leaders': 'hardware', 'followers': 'simulation'},
                                  "#FF9800", 2)
        self._create_mode_button(bimanual_btn_frame, "Leaders Sim / Followers HW", 
                                  {'mode': 'bimanual', 'leaders': 'simulation', 'followers': 'hardware'},
                                  "#9C27B0", 3)
        
        # Control algorithm mode
        control_frame = tk.LabelFrame(
            mode_frame,
            text="Control Algorithm Development",
            font=("Helvetica", 14, "bold"),
            bg="#2d2d2d",
            fg="white",
            padx=20,
            pady=20
        )
        control_frame.pack(fill='x', pady=10)
        
        tk.Label(
            control_frame,
            text="No leader arm - direct control via algorithms/MATLAB",
            font=("Helvetica", 11),
            bg="#2d2d2d",
            fg="#aaaaaa"
        ).pack(anchor='w')
        
        control_btn_frame = tk.Frame(control_frame, bg="#2d2d2d")
        control_btn_frame.pack(fill='x', pady=10)
        
        self._create_mode_button(control_btn_frame, "Single Arm - Hardware", 
                                  {'mode': 'control', 'target': 'hardware', 'count': 1},
                                  "#4CAF50", 0)
        self._create_mode_button(control_btn_frame, "Single Arm - Simulation", 
                                  {'mode': 'control', 'target': 'simulation', 'count': 1},
                                  "#2196F3", 1)
        self._create_mode_button(control_btn_frame, "Dual Arms - Hardware", 
                                  {'mode': 'control', 'target': 'hardware', 'count': 2},
                                  "#FF9800", 2)
        self._create_mode_button(control_btn_frame, "Dual Arms - Simulation", 
                                  {'mode': 'control', 'target': 'simulation', 'count': 2},
                                  "#9C27B0", 3)
        
    def _create_mode_button(self, parent, text: str, config: dict, color: str, column: int):
        """Create a mode selection button"""
        btn = tk.Button(
            parent,
            text=text,
            command=lambda: self._select_config(config),
            width=20,
            height=2,
            bg=color,
            fg="white",
            font=("Helvetica", 10),
            relief=tk.FLAT
        )
        btn.grid(row=0, column=column, padx=5, pady=5)
        parent.grid_columnconfigure(column, weight=1)
        
    def _select_config(self, config: dict):
        """Handle configuration selection"""
        config['arm_type'] = self.arm_type
        self.selected_config = config
        print(f"Selected configuration: {config}")
        
        if self.callback:
            self.callback(config)
        
        self.root.destroy()
        
    def show(self) -> dict:
        """Show the GUI and return selected configuration"""
        self.root.mainloop()
        return self.selected_config


class ModeSelectionGUI:
    """GUI for selecting operation mode: Simulation/Hardware/Hybrid"""
    
    def __init__(self, callback: Optional[Callable] = None):
        self.root = tk.Tk()
        self.root.title("arm-bench - Mode Selection")
        self.root.geometry("500x300")
        self.selected_mode = None
        self.callback = callback
        
        self._setup_ui()
        
    def _setup_ui(self):
        """Setup the UI components"""
        # Title
        title = tk.Label(
            self.root, 
            text="Select Operation Mode",
            font=("Arial", 18, "bold")
        )
        title.pack(pady=20)
        
        # Mode buttons frame
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=20)
        
        # Simulation button
        sim_btn = tk.Button(
            button_frame,
            text="Simulation",
            command=lambda: self._select_mode("simulation"),
            width=15,
            height=2,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 12)
        )
        sim_btn.grid(row=0, column=0, padx=10)
        
        # Hardware button
        hw_btn = tk.Button(
            button_frame,
            text="Hardware",
            command=lambda: self._select_mode("hardware"),
            width=15,
            height=2,
            bg="#2196F3",
            fg="white",
            font=("Arial", 12)
        )
        hw_btn.grid(row=0, column=1, padx=10)
        
        # Hybrid button
        hybrid_btn = tk.Button(
            button_frame,
            text="Hybrid",
            command=lambda: self._select_mode("hybrid"),
            width=15,
            height=2,
            bg="#FF9800",
            fg="white",
            font=("Arial", 12)
        )
        hybrid_btn.grid(row=0, column=2, padx=10)
        
        # Info label
        info_text = (
            "Simulation: No hardware required\n"
            "Hardware: Physical robot arms\n"
            "Hybrid: Mixed simulation and hardware"
        )
        info = tk.Label(self.root, text=info_text, justify=tk.LEFT)
        info.pack(pady=20)
        
    def _select_mode(self, mode: str):
        """Handle mode selection"""
        self.selected_mode = mode
        print(f"Selected mode: {mode}")
        
        if self.callback:
            self.callback(mode)
        
        self.root.destroy()
        
    def show(self) -> str:
        """Show the GUI and return selected mode"""
        self.root.mainloop()
        return self.selected_mode


class MonitoringInterface:
    """Interface to display robot arm telemetry"""
    
    def __init__(self, mode: str, motors: list = None, cameras: list = None):
        self.mode = mode
        self.motors = motors or []
        self.cameras = cameras or []
        
        # Import OpenCV now if we have cameras (lazy import after tkinter is ready)
        if self.cameras:
            _lazy_import_opencv()
        
        self.root = tk.Tk()
        self.root.title(f"arm-bench - Monitoring ({mode.capitalize()})")
        try:
            self.root.geometry("1400x900")
        except:
            self.root.geometry("1200x800")  # Fallback size
        self.root.configure(bg="#1e1e1e")
        
        # Data storage
        self.motor_data = {}
        self.camera_frames = {}
        self.camera_labels = {}
        self.camera_failed_count = {}  # Track consecutive failures
        
        # Recording state
        self.is_recording = False
        self.recording_data = []
        self.recording_start_time = None
        self.recording_output_dir = None
        self.update_interval = 100  # ms
        self.update_running = False
        
        # Colors
        self.colors = {
            'bg': '#1e1e1e',
            'fg': '#ffffff',
            'panel_bg': '#2d2d2d',
            'accent': '#007acc',
            'success': '#4ec9b0',
            'warning': '#dcdcaa',
            'error': '#f48771'
        }
        
        self._setup_ui()
        
    def _setup_ui(self):
        """Setup monitoring interface"""
        # Configure style
        style = ttk.Style()
        try:
            style.theme_use('clam')
        except Exception as e:
            # Fallback to default theme if clam fails
            print(f"Warning: Could not set theme 'clam': {e}")
        style.configure('TNotebook', background=self.colors['bg'])
        style.configure('TNotebook.Tab', background=self.colors['panel_bg'], foreground=self.colors['fg'], padding=[20, 10])
        style.map('TNotebook.Tab', background=[('selected', self.colors['accent'])])
        
        # Title Bar
        title_frame = tk.Frame(self.root, bg=self.colors['accent'], height=60)
        title_frame.pack(fill='x', side='top')
        title_frame.pack_propagate(False)
        
        title = tk.Label(
            title_frame,
            text=f" arm-bench Monitoring Dashboard - {self.mode.upper()} MODE",
            font=("Helvetica", 18, "bold"),
            bg=self.colors['accent'],
            fg=self.colors['fg']
        )
        title.pack(pady=15)
        
        # System Info Bar
        info_frame = tk.Frame(self.root, bg=self.colors['panel_bg'], height=50)
        info_frame.pack(fill='x', side='top')
        info_frame.pack_propagate(False)
        
        self.info_label = tk.Label(
            info_frame,
            text=f"Motors: {len(self.motors)} | Cameras: {len(self.cameras)} | Status: Ready",
            font=("Helvetica", 11),
            bg=self.colors['panel_bg'],
            fg=self.colors['success']
        )
        self.info_label.pack(pady=10)
        
        # Create notebook for tabs
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Overview Tab
        overview_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(overview_frame, text="Overview")
        self._setup_overview_tab(overview_frame)
        
        # Motor Status Tab
        motor_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(motor_frame, text="Motor Status")
        self._setup_motor_tab(motor_frame)
        
        # Camera Tab
        camera_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(camera_frame, text="Cameras")
        self._setup_camera_tab(camera_frame)
        
        # Control Tab
        control_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(control_frame, text="Control")
        self._setup_control_tab(control_frame)
        
        # Data Recording Tab
        recording_frame = tk.Frame(notebook, bg=self.colors['bg'])
        notebook.add(recording_frame, text=" Recording")
        self._setup_recording_tab(recording_frame)
        
    def _setup_overview_tab(self, parent):
        """Setup overview dashboard"""
        parent.configure(bg=self.colors['bg'])
        
        # Arm Information Panel
        arm_panel = tk.LabelFrame(
            parent,
            text=" Arm Information",
            font=("Helvetica", 12, "bold"),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg'],
            padx=20,
            pady=20
        )
        arm_panel.pack(fill='x', padx=20, pady=10)
        
        arm_info_text = f"""
Mode: {self.mode.upper()}
Total Motors: {len(self.motors)}
Total Cameras: {len(self.cameras)}
DOF: {len(self.motors) if self.motors else 'N/A'}
Arm Type: {'Interbotix' if self.motors else 'Simulated'}
Control Frequency: 100 Hz
        """
        
        arm_label = tk.Label(
            arm_panel,
            text=arm_info_text.strip(),
            font=("Consolas", 11),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg'],
            justify=tk.LEFT
        )
        arm_label.pack(anchor='w')
        
        # Motor Summary Grid
        motor_summary_panel = tk.LabelFrame(
            parent,
            text=" Motor Summary",
            font=("Helvetica", 12, "bold"),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg'],
            padx=20,
            pady=20
        )
        motor_summary_panel.pack(fill='both', expand=True, padx=20, pady=10)
        
        if self.motors:
            # Create grid of motor cards
            motors_per_row = 3
            for idx, motor in enumerate(self.motors):
                row = idx // motors_per_row
                col = idx % motors_per_row
                
                motor_card = tk.Frame(
                    motor_summary_panel,
                    bg=self.colors['accent'],
                    relief=tk.RAISED,
                    borderwidth=2
                )
                motor_card.grid(row=row, column=col, padx=10, pady=10, sticky='nsew')
                
                motor_summary_panel.grid_columnconfigure(col, weight=1)
                
                # Motor info
                temp_value = motor.get('temperature', 'N/A')
                motor_info = (f"Motor {motor['id']}\n"
                             f"{motor['model']}\n"
                             f"Temp: {temp_value}C\n"
                             f"Port: {motor['port']}")
                
                motor_label = tk.Label(
                    motor_card,
                    text=motor_info,
                    font=("Helvetica", 10),
                    bg=self.colors['accent'],
                    fg=self.colors['fg'],
                    justify=tk.LEFT,
                    padx=15,
                    pady=10
                )
                motor_label.pack()
        else:
            no_motor_label = tk.Label(
                motor_summary_panel,
                text="No motors detected. Running in simulation mode.",
                font=("Helvetica", 11),
                bg=self.colors['panel_bg'],
                fg=self.colors['warning']
            )
            no_motor_label.pack(pady=20)
    
    def _setup_motor_tab(self, parent):
        """Setup motor status display"""
        parent.configure(bg=self.colors['bg'])
        
        # Control buttons at top
        button_frame = tk.Frame(parent, bg=self.colors['bg'])
        button_frame.pack(fill='x', padx=10, pady=10)
        
        tk.Button(
            button_frame,
            text="Refresh Data",
            command=self._refresh_motor_data,
            bg=self.colors['accent'],
            fg=self.colors['fg'],
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=8
        ).pack(side='left', padx=5)
        
        tk.Button(
            button_frame,
            text="Start Auto-Update",
            command=self._start_auto_update,
            bg=self.colors['success'],
            fg=self.colors['fg'],
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=8
        ).pack(side='left', padx=5)
        
        tk.Button(
            button_frame,
            text="Stop Auto-Update",
            command=self._stop_auto_update,
            bg=self.colors['error'],
            fg=self.colors['fg'],
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=8
        ).pack(side='left', padx=5)
        
        # Create styled treeview for motor data
        tree_frame = tk.Frame(parent, bg=self.colors['bg'])
        tree_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        columns = ("ID", "Model", "Position", "Velocity", "Torque", "Temperature", "Status")
        self.motor_tree = ttk.Treeview(tree_frame, columns=columns, show='headings', height=15)
        
        # Configure columns
        col_widths = {"ID": 60, "Model": 120, "Position": 100, "Velocity": 100, "Torque": 100, "Temperature": 120, "Status": 100}
        for col in columns:
            self.motor_tree.heading(col, text=col, anchor='center')
            self.motor_tree.column(col, width=col_widths.get(col, 100), anchor='center')
        
        # Add scrollbars
        vsb = ttk.Scrollbar(tree_frame, orient="vertical", command=self.motor_tree.yview)
        hsb = ttk.Scrollbar(tree_frame, orient="horizontal", command=self.motor_tree.xview)
        self.motor_tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        
        self.motor_tree.grid(row=0, column=0, sticky='nsew')
        vsb.grid(row=0, column=1, sticky='ns')
        hsb.grid(row=1, column=0, sticky='ew')
        
        tree_frame.grid_rowconfigure(0, weight=1)
        tree_frame.grid_columnconfigure(0, weight=1)
        
        # Configure treeview colors
        style = ttk.Style()
        style.configure("Treeview", background=self.colors['panel_bg'], 
                       foreground=self.colors['fg'], fieldbackground=self.colors['panel_bg'])
        style.configure("Treeview.Heading", background=self.colors['accent'], 
                       foreground=self.colors['fg'], font=("Helvetica", 10, "bold"))
        
        # Initial data load
        self._refresh_motor_data()
        
    def _setup_control_tab(self, parent):
        """Setup control interface"""
        parent.configure(bg=self.colors['bg'])
        
        # Title
        tk.Label(
            parent,
            text="Control Interface",
            font=("Helvetica", 16, "bold"),
            bg=self.colors['bg'],
            fg=self.colors['fg']
        ).pack(pady=20)
        
        # Visualization Section
        viz_panel = tk.LabelFrame(
            parent,
            text="Visualization",
            font=("Helvetica", 12, "bold"),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg'],
            padx=30,
            pady=20
        )
        viz_panel.pack(fill='x', padx=40, pady=10)
        
        viz_btn_frame = tk.Frame(viz_panel, bg=self.colors['panel_bg'])
        viz_btn_frame.pack()
        
        tk.Button(
            viz_btn_frame,
            text="Launch RViz",
            command=self._launch_rviz,
            bg=self.colors['success'],
            fg=self.colors['fg'],
            font=("Helvetica", 11),
            relief=tk.FLAT,
            padx=20,
            pady=12
        ).grid(row=0, column=0, padx=5, pady=5)
        
        tk.Button(
            viz_btn_frame,
            text="2 Robot Simulation",
            command=lambda: self._launch_pybullet_sim(2),
            bg=self.colors['accent'],
            fg=self.colors['fg'],
            font=("Helvetica", 11),
            relief=tk.FLAT,
            padx=20,
            pady=12
        ).grid(row=0, column=1, padx=5, pady=5)
        
        tk.Button(
            viz_btn_frame,
            text="4 Robot Simulation",
            command=lambda: self._launch_pybullet_sim(4),
            bg=self.colors['accent'],
            fg=self.colors['fg'],
            font=("Helvetica", 11),
            relief=tk.FLAT,
            padx=20,
            pady=12
        ).grid(row=0, column=2, padx=5, pady=5)
        
        # Show simulation checkbox
        self.show_sim_var = tk.BooleanVar(value=True)
        tk.Checkbutton(
            viz_panel,
            text="Show simulation alongside hardware",
            variable=self.show_sim_var,
            bg=self.colors['panel_bg'],
            fg=self.colors['fg'],
            selectcolor=self.colors['panel_bg'],
            font=("Helvetica", 10)
        ).pack(pady=5)
        
        # Teleoperation Section
        teleop_panel = tk.LabelFrame(
            parent,
            text="Teleoperation",
            font=("Helvetica", 12, "bold"),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg'],
            padx=30,
            pady=20
        )
        teleop_panel.pack(fill='x', padx=40, pady=10)
        
        teleop_btn_frame = tk.Frame(teleop_panel, bg=self.colors['panel_bg'])
        teleop_btn_frame.pack()
        
        tk.Button(
            teleop_btn_frame,
            text="Joystick Control",
            command=self._start_joystick_teleop,
            bg=self.colors['accent'],
            fg=self.colors['fg'],
            font=("Helvetica", 11),
            relief=tk.FLAT,
            padx=25,
            pady=12
        ).grid(row=0, column=0, padx=10, pady=5)
        
        tk.Button(
            teleop_btn_frame,
            text="Camera Control (WebXR)",
            command=self._start_camera_teleop,
            bg=self.colors['accent'],
            fg=self.colors['fg'],
            font=("Helvetica", 11),
            relief=tk.FLAT,
            padx=25,
            pady=12
        ).grid(row=0, column=1, padx=10, pady=5)
        
        # Custom Control Algorithm Section
        control_algo_panel = tk.LabelFrame(
            parent,
            text="Custom Control Algorithms",
            font=("Helvetica", 12, "bold"),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg'],
            padx=30,
            pady=20
        )
        control_algo_panel.pack(fill='x', padx=40, pady=10)
        
        tk.Label(
            control_algo_panel,
            text="Select Control Mode:",
            font=("Helvetica", 11),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg']
        ).pack(anchor='w', pady=5)
        
        self.control_mode_var = tk.StringVar(value="position")
        
        mode_frame = tk.Frame(control_algo_panel, bg=self.colors['panel_bg'])
        mode_frame.pack(anchor='w', pady=5)
        
        for mode, text in [("position", "Position Control"), ("velocity", "Velocity Control"), ("torque", "Torque Control")]:
            tk.Radiobutton(
                mode_frame,
                text=text,
                variable=self.control_mode_var,
                value=mode,
                bg=self.colors['panel_bg'],
                fg=self.colors['fg'],
                selectcolor=self.colors['accent'],
                font=("Helvetica", 10)
            ).pack(side='left', padx=15)
        
        # Frequency setting
        freq_frame = tk.Frame(control_algo_panel, bg=self.colors['panel_bg'])
        freq_frame.pack(anchor='w', pady=10)
        
        tk.Label(
            freq_frame,
            text="Control Frequency (Hz):",
            font=("Helvetica", 10),
            bg=self.colors['panel_bg'],
            fg=self.colors['fg']
        ).pack(side='left', padx=5)
        
        self.freq_var = tk.StringVar(value="100")
        freq_entry = tk.Entry(
            freq_frame,
            textvariable=self.freq_var,
            width=10,
            font=("Helvetica", 10)
        )
        freq_entry.pack(side='left', padx=5)
        
        # Control buttons
        control_btn_frame = tk.Frame(control_algo_panel, bg=self.colors['panel_bg'])
        control_btn_frame.pack(pady=10)
        
        tk.Button(
            control_btn_frame,
            text="Load Control File",
            command=self._load_control_file,
            bg=self.colors['accent'],
            fg=self.colors['fg'],
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=10
        ).pack(side='left', padx=5)
        
        tk.Button(
            control_btn_frame,
            text="Start Control",
            command=self._start_custom_control,
            bg=self.colors['success'],
            fg=self.colors['fg'],
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=10
        ).pack(side='left', padx=5)
        
        tk.Button(
            control_btn_frame,
            text="MATLAB Parameter Tuning",
            command=self._open_matlab_tuning,
            bg=self.colors['warning'],
            fg='black',
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=10
        ).pack(side='left', padx=5)
        
    def _setup_recording_tab(self, parent):
        """Setup data recording interface"""
        tk.Label(
            parent,
            text="RLDS Dataset Recording",
            font=("Arial", 14, "bold")
        ).pack(pady=10)
        
        # Recording status
        self.recording_status = tk.Label(
            parent,
            text="Status: Not Recording",
            font=("Arial", 11)
        )
        self.recording_status.pack(pady=10)
        
        # Camera configuration
        cam_frame = tk.LabelFrame(parent, text="Camera Configuration", padx=10, pady=10)
        cam_frame.pack(padx=20, pady=10, fill='x')
        
        tk.Button(
            cam_frame,
            text="Scan Cameras",
            command=self._scan_cameras
        ).pack(pady=5)
        
        self.camera_listbox = tk.Listbox(cam_frame, height=5)
        self.camera_listbox.pack(fill='x', pady=5)
        
        tk.Button(
            cam_frame,
            text="Rename Selected Camera",
            command=self._rename_camera
        ).pack(pady=5)
        
        # Recording controls
        control_frame = tk.Frame(parent)
        control_frame.pack(pady=20)
        
        tk.Button(
            control_frame,
            text="Start Recording",
            command=self._start_recording,
            bg="#4CAF50",
            fg="white",
            width=15
        ).grid(row=0, column=0, padx=5)
        
        tk.Button(
            control_frame,
            text="Stop Recording",
            command=self._stop_recording,
            bg="#f44336",
            fg="white",
            width=15
        ).grid(row=0, column=1, padx=5)
        
    def _setup_camera_tab(self, parent):
        """Setup camera feed display"""
        parent.configure(bg=self.colors['bg'])
        
        # Camera controls
        control_frame = tk.Frame(parent, bg=self.colors['bg'])
        control_frame.pack(fill='x', padx=10, pady=10)
        
        tk.Label(
            control_frame,
            text="Live Camera Feeds",
            font=("Helvetica", 14, "bold"),
            bg=self.colors['bg'],
            fg=self.colors['fg']
        ).pack(side='left', padx=10)
        
        tk.Button(
            control_frame,
            text="Start All Cameras",
            command=self._start_all_cameras,
            bg=self.colors['success'],
            fg=self.colors['fg'],
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=8
        ).pack(side='left', padx=5)
        
        tk.Button(
            control_frame,
            text="Stop All Cameras",
            command=self._stop_all_cameras,
            bg=self.colors['error'],
            fg=self.colors['fg'],
            font=("Helvetica", 10),
            relief=tk.FLAT,
            padx=20,
            pady=8
        ).pack(side='left', padx=5)
        
        # Camera grid
        self.camera_grid_frame = tk.Frame(parent, bg=self.colors['bg'])
        self.camera_grid_frame.pack(fill='both', expand=True, padx=10, pady=10)
        
        if self.cameras:
            # Create camera display widgets
            cameras_per_row = 2
            for idx, camera in enumerate(self.cameras):
                row = idx // cameras_per_row
                col = idx % cameras_per_row
                
                camera_name = camera.get('name', f"Camera {camera['id']}")
                camera_frame = tk.LabelFrame(
                    self.camera_grid_frame,
                    text=camera_name,
                    font=("Helvetica", 11, "bold"),
                    bg=self.colors['panel_bg'],
                    fg=self.colors['fg'],
                    padx=10,
                    pady=10
                )
                camera_frame.grid(row=row, column=col, padx=10, pady=10, sticky='nsew')
                
                self.camera_grid_frame.grid_rowconfigure(row, weight=1)
                self.camera_grid_frame.grid_columnconfigure(col, weight=1)
                
                # Video display label - using fixed pixel size instead of character width
                video_label = tk.Label(
                    camera_frame,
                    text="Camera feed will appear here",
                    bg='black',
                    fg='white'
                )
                video_label.pack()
                
                self.camera_labels[camera['id']] = video_label
                
                # Camera info
                info_text = f"Resolution: {camera.get('resolution', 'Unknown')} | FPS: {camera.get('fps', 'N/A')}"
                tk.Label(
                    camera_frame,
                    text=info_text,
                    font=("Helvetica", 9),
                    bg=self.colors['panel_bg'],
                    fg=self.colors['warning']
                ).pack(pady=5)
        else:
            no_cam_label = tk.Label(
                self.camera_grid_frame,
                text="No cameras detected. Use 'arm-bench cameras' to scan.",
                font=("Helvetica", 12),
                bg=self.colors['bg'],
                fg=self.colors['warning']
            )
            no_cam_label.pack(pady=50)
    
    def _start_auto_update(self):
        """Start automatic data updates"""
        if not self.update_running:
            self.update_running = True
            self._auto_update_loop()
            self.info_label.config(text=f"Motors: {len(self.motors)} | Cameras: {len(self.cameras)} | Status: Auto-Updating", 
                                  fg=self.colors['success'])
    
    def _stop_auto_update(self):
        """Stop automatic data updates"""
        self.update_running = False
        self.info_label.config(text=f"Motors: {len(self.motors)} | Cameras: {len(self.cameras)} | Status: Stopped", 
                              fg=self.colors['warning'])
    
    def _auto_update_loop(self):
        """Auto-update loop for motor data"""
        if self.update_running:
            self._refresh_motor_data()
            self.root.after(self.update_interval, self._auto_update_loop)
    
    def _start_all_cameras(self):
        """Start all camera feeds"""
        if not OPENCV_AVAILABLE:
            messagebox.showwarning("OpenCV Required", "Please install opencv-python and pillow:\npip install opencv-python pillow")
            return
        
        try:
            # Check if PIL is available
            if Image is None or ImageTk is None:
                messagebox.showerror("PIL Error", "Pillow is not installed or failed to import.\nPlease install: pip install pillow")
                return
            
            successful_cameras = []
            failed_cameras = []
            
            for camera in self.cameras:
                camera_id = camera['id']
                if camera_id not in self.camera_frames or self.camera_frames[camera_id] is None:
                    print(f"Attempting to open camera {camera_id}...")
                    
                    # Use MJPEG format for bandwidth efficiency with multiple cameras
                    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)
                    
                    if cap.isOpened():
                        # Configure camera for better multi-camera support
                        # MJPEG uses much less USB bandwidth than YUYV format
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 15)  # Lower FPS for multi-camera
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        
                        # Small delay between camera opens for USB bandwidth
                        import time
                        time.sleep(0.1)
                        
                        # Test if we can actually read a frame (try up to 3 times)
                        ret = False
                        for attempt in range(3):
                            ret, test_frame = cap.read()
                            if ret:
                                break
                            time.sleep(0.1)
                        
                        if ret:
                            self.camera_frames[camera_id] = cap
                            self.camera_failed_count[camera_id] = 0
                            successful_cameras.append(camera_id)
                            print(f"✓ Camera {camera_id} opened successfully")
                        else:
                            print(f"✗ Camera {camera_id} opened but cannot read frames (USB bandwidth?)")
                            cap.release()
                            failed_cameras.append(camera_id)
                            # Update label to show error
                            if camera_id in self.camera_labels:
                                self.camera_labels[camera_id].config(
                                    text=f"Camera {camera_id} - Cannot Read Frames\nCheck camera connection",
                                    fg='orange'
                                )
                    else:
                        print(f"✗ Failed to open camera {camera_id}")
                        failed_cameras.append(camera_id)
                        # Update label to show error
                        if camera_id in self.camera_labels:
                            self.camera_labels[camera_id].config(
                                text=f"Camera {camera_id} - Failed to Open\nCamera may be in use",
                                fg='red'
                            )
            
            if successful_cameras:
                self._update_camera_feeds()
                status_msg = f"Motors: {len(self.motors)} | Cameras: {len(successful_cameras)}/{len(self.cameras)} active | Status: Running"
                self.info_label.config(text=status_msg, fg=self.colors['success'])
                
                if failed_cameras:
                    messagebox.showwarning(
                        "Camera Warning", 
                        f"Started {len(successful_cameras)} camera(s) successfully.\n\n"
                        f"Failed cameras: {', '.join(map(str, failed_cameras))}\n\n"
                        f"These cameras may be in use by another application."
                    )
            else:
                messagebox.showerror("Camera Error", "Failed to start any cameras.\nAll cameras are unavailable or in use.")
                
        except Exception as e:
            messagebox.showerror("Camera Error", f"Failed to start cameras: {e}")
    
    def _stop_all_cameras(self):
        """Stop all camera feeds"""
        for camera_id, cap in self.camera_frames.items():
            if cap is not None:
                cap.release()
        self.camera_frames.clear()
        self.info_label.config(text=f"Motors: {len(self.motors)} | Cameras: {len(self.cameras)} | Status: Cameras Stopped", 
                              fg=self.colors['warning'])
    
    def _update_camera_feeds(self):
        """Update camera feed displays"""
        if not OPENCV_AVAILABLE:
            return
        
        # Check if PIL is available
        if Image is None or ImageTk is None:
            print("PIL/Pillow not available for camera display")
            return
        
        try:
            # Update ALL cameras in a single iteration
            any_camera_active = False
            cameras_to_remove = []
            
            for camera_id, cap in list(self.camera_frames.items()):
                if cap is not None and cap.isOpened():
                    any_camera_active = True
                    ret, frame = cap.read()
                    
                    if ret and camera_id in self.camera_labels:
                        # Success! Reset failure counter
                        self.camera_failed_count[camera_id] = 0
                        
                        # Resize frame to larger display size
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                        frame = cv2.resize(frame, (800, 600))  # Much larger display
                        
                        # Convert to PhotoImage
                        img = Image.fromarray(frame)
                        photo = ImageTk.PhotoImage(image=img)
                        
                        # Update label
                        label = self.camera_labels[camera_id]
                        label.config(image=photo)
                        label.image = photo  # Keep a reference
                    elif not ret:
                        # Failed to read frame
                        if camera_id not in self.camera_failed_count:
                            self.camera_failed_count[camera_id] = 0
                        self.camera_failed_count[camera_id] += 1
                        
                        # Only print first few failures, then stop the camera
                        if self.camera_failed_count[camera_id] <= 3:
                            print(f"Failed to read frame from camera {camera_id} (attempt {self.camera_failed_count[camera_id]})")
                        
                        # After 10 consecutive failures, stop this camera
                        if self.camera_failed_count[camera_id] >= 10:
                            print(f"Camera {camera_id} failed too many times. Stopping it.")
                            cameras_to_remove.append(camera_id)
                            # Update label to show error
                            if camera_id in self.camera_labels:
                                self.camera_labels[camera_id].config(
                                    text=f"Camera {camera_id} - Connection Lost\nToo many read failures",
                                    fg='red'
                                )
            
            # Remove failed cameras
            for camera_id in cameras_to_remove:
                if camera_id in self.camera_frames:
                    cap = self.camera_frames[camera_id]
                    if cap is not None:
                        cap.release()
                    del self.camera_frames[camera_id]
            
            # Schedule next update if any camera is active
            if any_camera_active and len(self.camera_frames) > 0:
                self.root.after(33, self._update_camera_feeds)  # ~30 FPS
            elif len(self.camera_frames) == 0:
                self.info_label.config(
                    text=f"Motors: {len(self.motors)} | Cameras: All stopped/failed | Status: Ready", 
                    fg=self.colors['warning']
                )
                
        except Exception as e:
            print(f"Error updating camera feeds: {e}")
            import traceback
            traceback.print_exc()
    
    def _refresh_motor_data(self):
        """Refresh motor data display"""
        # Clear existing data
        for item in self.motor_tree.get_children():
            self.motor_tree.delete(item)
        
        # Add actual motor data or simulated data
        if self.motors:
            for motor in self.motors:
                values = (
                    motor['id'],
                    motor.get('model', 'Unknown'),
                    f"{motor.get('position', 2048)}",
                    f"{motor.get('velocity', 0)} rad/s",
                    f"{motor.get('torque', 0)} Nm",
                    f"{motor.get('temperature', 30)}C",
                    " Active" if motor.get('temperature', 30) < 60 else "sHot"
                )
                self.motor_tree.insert('', 'end', values=values)
        else:
            # Simulated data
            import random
            for i in range(1, 7):
                temp_c = random.randint(28, 35)
                values = (
                    i,
                    "XM430-W350" if i <= 3 else "XL430-W250",
                    f"{2048 + random.randint(-500, 500)}",
                    f"{random.uniform(0, 1):.2f} rad/s",
                    f"{random.uniform(0, 0.5):.2f} Nm",
                    f"{temp_c}C",
                    " Active"
                )
                self.motor_tree.insert('', 'end', values=values)
        
    def _launch_rviz(self):
        """Launch RViz visualization"""
        try:
            from arm_bench.visualization import VisualizationLauncher
            launcher = VisualizationLauncher(mode=self.mode)
            if launcher.launch_rviz():
                messagebox.showinfo("RViz", "RViz launched successfully!")
            else:
                messagebox.showwarning("RViz", "Failed to launch RViz.\nMake sure ROS2 is sourced.")
        except Exception as e:
            messagebox.showerror("RViz Error", f"Error launching RViz:\n{e}")
    
    def _launch_pybullet_sim(self, num_robots: int = 2):
        """Launch PyBullet simulation with specified number of robots"""
        try:
            from arm_bench.visualization import PyBulletVisualizer, VisualizationLauncher, PYBULLET_AVAILABLE
            
            if not PYBULLET_AVAILABLE:
                messagebox.showerror(
                    "PyBullet Error",
                    "PyBullet is not installed.\n\nInstall with:\npip install pybullet"
                )
                return
            
            # Determine if we should show leaders
            show_leaders = True
            if hasattr(self, 'show_sim_var'):
                show_leaders = self.show_sim_var.get()
            
            # Launch visualization
            launcher = VisualizationLauncher(mode=self.mode)
            viz = launcher.launch_pybullet(
                num_robots=num_robots,
                robot_type="vx300s",
                show_leaders=show_leaders
            )
            
            if viz:
                self.pybullet_viz = viz
                messagebox.showinfo(
                    "Simulation",
                    f"PyBullet simulation launched with {num_robots} robots!\n\n"
                    "The simulation window is now open.\n"
                    "Robot positions will sync with hardware/control."
                )
            else:
                messagebox.showwarning(
                    "Simulation",
                    "Failed to launch PyBullet simulation.\n"
                    "Check URDF files are available."
                )
                
        except Exception as e:
            import traceback
            traceback.print_exc()
            messagebox.showerror("Simulation Error", f"Error launching simulation:\n{e}")
        
    def _start_joystick_teleop(self):
        """Start joystick teleoperation"""
        try:
            from arm_bench.teleoperation import JoystickTeleoperation, ArmBenchTeleoperator
            
            # Check for pygame
            try:
                import pygame
            except ImportError:
                messagebox.showerror(
                    "Joystick Error",
                    "pygame is not installed.\n\nInstall with:\npip install pygame"
                )
                return
            
            # Create teleoperator with Dynamixel controller if available
            try:
                from arm_bench.dynamixel_control import DynamixelController
                import os
                
                port = '/dev/ttyDXL' if os.path.exists('/dev/ttyDXL') else '/dev/ttyUSB0'
                controller = DynamixelController(port=port)
                
                if controller.connect():
                    controller.scan_motors(range(1, 10))
                    controller.enable_all_torque()
                    teleoperator = ArmBenchTeleoperator(controller=controller)
                else:
                    teleoperator = ArmBenchTeleoperator()
                    messagebox.showwarning("Joystick", "Running without hardware.\nDynamixel controller not available.")
            except Exception as e:
                teleoperator = ArmBenchTeleoperator()
                print(f"Running without hardware: {e}")
            
            # Store reference and start
            self.teleoperator = teleoperator
            self.teleoperator.start_joystick()
            
            messagebox.showinfo(
                "Joystick Control", 
                "Joystick teleoperation started!\n\n"
                "Controls:\n"
                "  Left Stick: XY movement\n"
                "  Right Stick: Z and rotation\n"
                "  Button A: Toggle gripper\n"
                "  Button B: Toggle move enable\n"
                "  Button Back: Stop\n\n"
                "Check terminal for output."
            )
            
        except ImportError as e:
            messagebox.showerror("Joystick Error", f"Failed to import teleoperation module:\n{e}")
        except Exception as e:
            messagebox.showerror("Joystick Error", f"Failed to start joystick control:\n{e}")
        
    def _start_camera_teleop(self):
        """Start camera-based teleoperation (WebXR)"""
        try:
            # Check for FastAPI dependencies
            try:
                import fastapi
                import uvicorn
                import websockets
            except ImportError:
                messagebox.showerror(
                    "WebXR Error",
                    "Required packages not installed.\n\nInstall with:\npip install fastapi uvicorn websockets"
                )
                return
            
            # Import WebXR module
            try:
                from arm_bench.webxr import WebXRTeleoperationServer
            except ImportError as e:
                messagebox.showerror(
                    "WebXR Error",
                    f"Failed to import WebXR module:\n{e}\n\nMake sure the webxr module is installed."
                )
                return
            
            # Get server port - use 8443 for HTTPS (required for WebXR on mobile)
            server_port = 8443
            
            # Create and store WebXR server with hardware controller
            try:
                from arm_bench.dynamixel_control import DynamixelController
                import os
                
                port = '/dev/ttyDXL' if os.path.exists('/dev/ttyDXL') else '/dev/ttyUSB0'
                controller = DynamixelController(port=port)
                
                if controller.connect():
                    controller.scan_motors(range(1, 10))
                    controller.enable_all_torque()
                    self.webxr_server = WebXRTeleoperationServer(port=server_port, controller=controller)
                else:
                    self.webxr_server = WebXRTeleoperationServer(port=server_port)
            except Exception as e:
                print(f"Running WebXR without hardware: {e}")
                self.webxr_server = WebXRTeleoperationServer(port=server_port)
            
            # Start server in background thread
            import threading
            def start_webxr():
                try:
                    self.webxr_server.run()
                except Exception as e:
                    print(f"WebXR server error: {e}")
            
            webxr_thread = threading.Thread(target=start_webxr, daemon=True)
            webxr_thread.start()
            
            # Wait a moment for server to start
            import time
            time.sleep(1.0)
            
            # Get local IP
            import socket
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                local_ip = s.getsockname()[0]
                s.close()
            except:
                local_ip = "localhost"
            
            messagebox.showinfo(
                "WebXR Control", 
                f"WebXR teleoperation server started!\n\n"
                f"Open this URL on your AR/VR device:\n"
                f"https://{local_ip}:{server_port}/static/index.html\n\n"
                f"Requirements:\n"
                f"• Chrome/Edge with WebXR support\n"
                f"• AR-capable device (phone/headset)\n"
                f"• Same WiFi network as this PC\n"
                f"• Accept the security warning (self-signed cert)\n\n"
                f"Controls:\n"
                f"• Click 'Connect to Server' first\n"
                f"• Click 'Start AR' to begin tracking\n"
                f"• Hold MOVE button to track arm position\n"
                f"• Click GRIPPER button to toggle gripper\n\n"
                f"Check terminal for server output."
            )
            
        except ImportError as e:
            messagebox.showerror("WebXR Error", f"Failed to import teleoperation module:\n{e}")
        except Exception as e:
            import traceback
            traceback.print_exc()
            messagebox.showerror("WebXR Error", f"Failed to start WebXR control:\n{e}")
    
    def _load_control_file(self):
        """Load custom control algorithm file"""
        from tkinter import filedialog
        
        filename = filedialog.askopenfilename(
            title="Select Control Algorithm File",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        
        if filename:
            messagebox.showinfo("Control File", f"Loaded: {filename}\nControl mode: {self.control_mode_var.get()}\nFrequency: {self.freq_var.get()} Hz")
            # This will be integrated with ROS in the control module
    
    def _start_custom_control(self):
        """Start custom control algorithm"""
        mode = self.control_mode_var.get()
        freq = self.freq_var.get()
        messagebox.showinfo(
            "Custom Control",
            f"Starting {mode} control at {freq} Hz\n\nPublishing to ROS topic:\n/arm_bench/{mode}_command"
        )
        # TODO: Integrate with ROS control module
    
    def _open_matlab_tuning(self):
        """Open MATLAB parameter tuning interface"""
        try:
            from arm_bench.matlab_integration_enhanced import launch_control_development
            
            # Close current window
            self.root.withdraw()
            
            # Launch MATLAB tuning interface
            messagebox.showinfo(
                "MATLAB Tuning",
                "Launching MATLAB parameter tuning interface...\n\nThis will open a new window for automated parameter optimization."
            )
            
            # Run in separate thread to avoid blocking
            import threading
            thread = threading.Thread(target=launch_control_development)
            thread.daemon = True
            thread.start()
            
        except ImportError as e:
            messagebox.showerror(
                "MATLAB Integration",
                f"MATLAB integration module not found.\n\nError: {e}\n\nPlease ensure matlab_integration_enhanced.py is present."
            )
        except Exception as e:
            messagebox.showerror(
                "MATLAB Tuning Error",
                f"Failed to launch MATLAB tuning interface:\n\n{e}"
            )
        
    def _scan_cameras(self):
        """Scan for available cameras"""
        self.camera_listbox.delete(0, tk.END)
        
        # Use actual detected cameras from initialization
        if self.cameras:
            for camera in self.cameras:
                camera_name = camera.get('name', f"Camera {camera['id']}")
                camera_info = f"{camera_name} - {camera.get('resolution', 'Unknown')}"
                self.camera_listbox.insert(tk.END, camera_info)
            messagebox.showinfo("Camera Scan", f"Found {len(self.cameras)} cameras")
        else:
            messagebox.showwarning("Camera Scan", "No cameras detected. Try running 'arm-bench cameras' in terminal.")
        
    def _rename_camera(self):
        """Rename selected camera"""
        selection = self.camera_listbox.curselection()
        if not selection:
            messagebox.showwarning("Warning", "Please select a camera first")
            return
        
        # Simple rename dialog
        new_name = simpledialog.askstring("Rename Camera", "Enter new camera name:")
        if new_name:
            idx = selection[0]
            self.camera_listbox.delete(idx)
            self.camera_listbox.insert(idx, new_name)
            
    def _start_recording(self):
        """Start RLDS dataset recording"""
        if self.is_recording:
            messagebox.showwarning("Recording", "Already recording!")
            return
        
        # Check if cameras are available - start them if not already started
        if not OPENCV_AVAILABLE:
            messagebox.showwarning("Recording Warning", 
                "OpenCV not available. Recording will only capture motor data, not camera frames.\n\n"
                "Install opencv-python: pip install opencv-python")
        elif not self.camera_frames and self.cameras:
            # Auto-start cameras for recording
            response = messagebox.askyesno(
                "Start Cameras?",
                "Cameras are not running. Start cameras for recording?\n\n"
                "Click 'Yes' to start cameras and record video.\n"
                "Click 'No' to record motor data only."
            )
            if response:
                self._start_all_cameras()
                # Wait a moment for cameras to initialize
                time.sleep(0.5)
        
        # Ask user for output directory
        output_dir = filedialog.askdirectory(title="Select Output Directory for Dataset")
        if not output_dir:
            return
        
        # Create timestamped subdirectory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.recording_output_dir = os.path.join(output_dir, f"arm_bench_dataset_{timestamp}")
        os.makedirs(self.recording_output_dir, exist_ok=True)
        
        # Create subdirectories for camera frames
        os.makedirs(os.path.join(self.recording_output_dir, "camera_frames"), exist_ok=True)
        
        # Initialize recording cameras dictionary for separate capture
        self.recording_cameras = {}
        if OPENCV_AVAILABLE and self.cameras:
            for camera in self.cameras:
                camera_id = camera['id']
                # Check if we already have a capture from the display
                if camera_id in self.camera_frames and self.camera_frames[camera_id] is not None:
                    # Reuse existing capture
                    self.recording_cameras[camera_id] = self.camera_frames[camera_id]
                    print(f"Recording: Using existing capture for camera {camera_id}")
        
        # Initialize recording
        self.is_recording = True
        self.recording_data = []
        self.recording_start_time = time.time()
        self.recording_frame_count = 0
        self.frames_with_camera_data = 0
        
        # Update UI
        camera_status = f"{len(self.recording_cameras)} cameras" if self.recording_cameras else "No cameras"
        self.recording_status.config(
            text=f"Status: Recording ({camera_status})", 
            fg="red"
        )
        
        # Start recording loop
        self._record_frame()
        
        messagebox.showinfo("Recording", 
            f"Started RLDS dataset recording\n\n"
            f"Output: {self.recording_output_dir}\n"
            f"Cameras: {len(self.recording_cameras)}"
        )
        
    def _stop_recording(self):
        """Stop RLDS dataset recording"""
        if not self.is_recording:
            messagebox.showwarning("Recording", "Not currently recording!")
            return
        
        self.is_recording = False
        
        # Calculate frames with camera data
        frames_with_cameras = sum(1 for f in self.recording_data if f.get('camera_frames'))
        
        # Save metadata
        metadata = {
            "start_time": self.recording_start_time,
            "end_time": time.time(),
            "duration": time.time() - self.recording_start_time,
            "num_frames": self.recording_frame_count,
            "frames_with_camera_data": frames_with_cameras,
            "mode": self.mode,
            "num_motors": len(self.motors),
            "num_cameras": len(self.cameras),
            "motor_ids": [m['id'] for m in self.motors],
            "camera_ids": [c['id'] for c in self.cameras],
            "recording_cameras": list(self.recording_cameras.keys()) if hasattr(self, 'recording_cameras') else []
        }
        
        metadata_path = os.path.join(self.recording_output_dir, "metadata.json")
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        # Save trajectory data
        data_path = os.path.join(self.recording_output_dir, "trajectory_data.json")
        with open(data_path, 'w') as f:
            json.dump(self.recording_data, f, indent=2)
        
        # Count camera frames saved
        camera_frames_dir = os.path.join(self.recording_output_dir, "camera_frames")
        saved_frames = len([f for f in os.listdir(camera_frames_dir) if f.endswith('.jpg')]) if os.path.exists(camera_frames_dir) else 0
        
        # Update UI
        self.recording_status.config(text="Status: Not Recording", fg=self.colors['fg'])
        
        # Clear recording cameras reference
        if hasattr(self, 'recording_cameras'):
            self.recording_cameras = {}
        
        messagebox.showinfo("Recording", 
                          f"Stopped recording. Dataset saved.\n\n"
                          f"Location: {self.recording_output_dir}\n"
                          f"Total Frames: {self.recording_frame_count}\n"
                          f"Camera Frames Saved: {saved_frames}\n"
                          f"Duration: {metadata['duration']:.2f}s")
        
    def _record_frame(self):
        """Record a single frame of data"""
        if not self.is_recording:
            return
        
        # Capture current timestamp
        timestamp = time.time() - self.recording_start_time
        
        # Collect motor data
        motor_states = {}
        for motor in self.motors:
            motor_states[motor['id']] = {
                'position': motor.get('position', 2048),
                'velocity': motor.get('velocity', 0),
                'torque': motor.get('torque', 0),
                'temperature': motor.get('temperature', 30)
            }
        
        # Save camera frames - use recording_cameras if available, otherwise camera_frames
        camera_frame_paths = {}
        cameras_to_use = getattr(self, 'recording_cameras', {}) or self.camera_frames
        
        if OPENCV_AVAILABLE and cameras_to_use:
            for camera_id, cap in cameras_to_use.items():
                if cap is not None and cap.isOpened():
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        try:
                            frame_filename = f"camera_{camera_id}_frame_{self.recording_frame_count:06d}.jpg"
                            frame_path = os.path.join(self.recording_output_dir, "camera_frames", frame_filename)
                            
                            # Use higher quality JPEG compression
                            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 95]
                            success = cv2.imwrite(frame_path, frame, encode_params)
                            
                            if success:
                                camera_frame_paths[camera_id] = frame_filename
                            else:
                                print(f"Warning: Failed to write frame for camera {camera_id}")
                        except Exception as e:
                            print(f"Error saving frame from camera {camera_id}: {e}")
        
        # Store frame data
        frame_data = {
            'timestamp': timestamp,
            'frame_number': self.recording_frame_count,
            'motor_states': motor_states,
            'camera_frames': camera_frame_paths
        }
        
        self.recording_data.append(frame_data)
        self.recording_frame_count += 1
        
        # Track frames with camera data
        if camera_frame_paths:
            if not hasattr(self, 'frames_with_camera_data'):
                self.frames_with_camera_data = 0
            self.frames_with_camera_data += 1
        
        # Update status with camera frame count
        camera_info = f" | Cam Frames: {getattr(self, 'frames_with_camera_data', 0)}" if cameras_to_use else ""
        self.recording_status.config(
            text=f"Recording... Frame {self.recording_frame_count} | Time: {timestamp:.2f}s{camera_info}",
            fg="red"
        )
        
        # Schedule next frame capture (10 Hz recording rate)
        if self.is_recording:
            self.root.after(100, self._record_frame)
        
    def show(self):
        """Show the monitoring interface"""
        try:
            print("Opening monitoring dashboard window...")
            print("(Close the GUI window to exit)")
            
            # Force window to front and update
            self.root.lift()
            self.root.attributes('-topmost', True)
            self.root.after_idle(self.root.attributes, '-topmost', False)
            self.root.update_idletasks()
            self.root.update()
            
            # Don't auto-start cameras - let user start them manually
            # Auto-starting can cause segfaults with some camera configurations
            # if self.cameras and OPENCV_AVAILABLE and Image is not None and ImageTk is not None:
            #     self.root.after(500, self._start_all_cameras)
            
            print(" Monitoring interface is now running")
            
            # Start mainloop (this blocks until window is closed)
            self.root.mainloop()
            
            print("Monitoring interface closed")
        except Exception as e:
            print(f"Error in monitoring interface: {e}")
            import traceback
            traceback.print_exc()


def launch_mode_selection(callback: Optional[Callable] = None) -> str:
    """Launch mode selection GUI"""
    gui = ModeSelectionGUI(callback)
    return gui.show()


def launch_arm_selection(callback: Optional[Callable] = None) -> str:
    """Launch arm selection GUI"""
    gui = ArmSelectionGUI(callback)
    return gui.show()


def launch_operation_mode_selection(arm_type: str, callback: Optional[Callable] = None) -> dict:
    """Launch operation mode selection GUI (bimanual/bilateral)"""
    gui = OperationModeGUI(arm_type, callback)
    return gui.show()


def launch_full_setup() -> Dict[str, Any]:
    """Launch the full setup flow: Arm -> Operation Mode"""
    # Step 1: Select arm type
    arm_selection_gui = ArmSelectionGUI()
    arm_type = arm_selection_gui.show()
    
    if not arm_type:
        return None
    
    # Step 2: Select operation mode
    operation_gui = OperationModeGUI(arm_type)
    config = operation_gui.show()
    
    return config


def launch_monitoring_interface(mode: str, motors: list = None, cameras: list = None):
    """Launch monitoring interface"""
    interface = MonitoringInterface(mode, motors, cameras)
    interface.show()

