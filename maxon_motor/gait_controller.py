#!/usr/bin/env python
import tkinter as tk
from tkinter import messagebox, filedialog, StringVar, IntVar, DoubleVar
import threading
import time
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from ctypes import *
import os

class GaitCycleController:
    """Gait cycle controller for EPOS2 motor controller"""
    def __init__(self, epos_controller):
        """Initialize with existing EPOS controller"""
        self.epos = epos_controller
        self.gait_data = None
        self.position_data = None
        self.running = False
        self.paused = False
        self.thread = None
        self.cycle_duration = 2.0  # Default 2 seconds per gait cycle
        self.current_index = 0
        self.gear_ratio = 1.0  # Adjust based on your mechanical setup
        self.angle_offset = 0.0  # Offset for zero position
        self.angle_scale = 1.0  # Scale factor for angle conversion
        self.encoder_cpt = 500  # Encoder counts per turn
        # 기존 초기화 코드...
        self.lock = threading.Lock()
        
    def load_gait_data(self, filepath):
        """Load gait data from CSV file"""
        try:
            # Read CSV file
            data = pd.read_csv(filepath)
            
            # Check if required columns exist
            if 'time' not in data.columns or 'angle' not in data.columns:
                raise ValueError("CSV file must contain 'time' and 'angle' columns")
            
            # Store the data
            self.gait_data = data
            
            # Convert angles to encoder positions
            self.calculate_position_profile()
            return True
        except Exception as e:
            messagebox.showerror("Data Loading Error", f"Failed to load gait data: {str(e)}")
            return False
            
    def calculate_position_profile(self):
        """Convert joint angles to motor positions"""
        if self.gait_data is None:
            return False
            
        # Calculate encoder counts from angles
        # Formula: counts = (angle + offset) * scale * (encoder_cpt / 360) * gear_ratio
        angles = self.gait_data['angle'].values
        positions = ((angles + self.angle_offset) * self.angle_scale * 
                     (self.encoder_cpt / 360.0) * self.gear_ratio)
        
        # Round to integers for position control
        self.position_data = np.round(positions).astype(int)
        return True
    
    def start_gait_cycle(self, continuous=False, cycles=1):
        """보행 주기 실행 시작"""
        if self.position_data is None or len(self.position_data) == 0:
            messagebox.showerror("오류", "보행 데이터가 로드되지 않았습니다")
            return False
        # 중요: EPOS 연결 상태 확인
        if not self.epos.key_handle:
            messagebox.showerror("연결 오류", "EPOS 컨트롤러가 연결되지 않았습니다")
            return False
        
        if self.running:
            messagebox.showinfo("Already Running", "Gait cycle is already running")
            return False
            
        # Enable the motor if not already enabled
        if not self.epos.set_enable_state():
            return False
        
        self.running = True
        self.paused = False
        self.current_index = 0
        
        # Start in a separate thread to not block the UI
        self.thread = threading.Thread(
            target=self._run_gait_cycle, 
            args=(continuous, cycles),
            daemon=True
        )
        self.thread.start()
        return True
        
    def _run_gait_cycle(self, continuous, cycles):
        """보행 주기 실행 (별도 스레드에서 실행)"""
        try:
            cycle_count = 0
            interval = self.cycle_duration / len(self.position_data)
            
            while self.running and (continuous or cycle_count < cycles):
                if not self.paused:
                    with self.lock:  # 스레드 동기화 추가
                        # 확인 - EPOS가 여전히 연결되어 있는지
                        if not self.epos.key_handle:
                            print("EPOS controller disconnected")
                            break
                            
                        # 현재 위치로 이동
                        position = self.position_data[self.current_index]
                        
                        # 절대 위치로 이동
                        if not self.epos.move_to_position(position, absolute=True):
                            break
                    
                    # 다음 위치로 이동
                    self.current_index = (self.current_index + 1) % len(self.position_data)
                    
                    # 주기가 완료되면
                    if self.current_index == 0 and not continuous:
                        cycle_count += 1
                        
                # 적절한 간격 대기
                time.sleep(interval)
                
            self.running = False
        except Exception as e:
            self.running = False
            print(f"Error in gait cycle execution: {str(e)}")
            # 오류 발생 시 모터 비활성화 시도
            try:
                self.epos.set_disable_state()
            except:
                pass
    
    def stop_gait_cycle(self):
        """Stop the gait cycle execution"""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)
        return True
        
    def pause_gait_cycle(self):
        """Pause the gait cycle execution"""
        self.paused = not self.paused
        return self.paused
    
    def set_cycle_duration(self, duration):
        """Set the duration of one gait cycle in seconds"""
        if duration > 0:
            self.cycle_duration = duration
            return True
        return False
        
    def set_angle_parameters(self, offset, scale):
        """Set parameters for angle conversion"""
        self.angle_offset = offset
        self.angle_scale = scale
        # Recalculate position profile if we have data
        if self.gait_data is not None:
            self.calculate_position_profile()
        return True
        
    def set_gear_ratio(self, ratio):
        """Set the gear ratio for position calculation"""
        if ratio > 0:
            self.gear_ratio = ratio
            # Recalculate position profile if we have data
            if self.gait_data is not None:
                self.calculate_position_profile()
            return True
        return False
    
class GaitControllerGUI:
    """GUI extension for gait cycle control"""
    def __init__(self, master, epos_controller):
        """Initialize the GUI with a parent window and EPOS controller"""
        self.master = master
        self.window = tk.Toplevel(master)
        self.window.title("Gait Cycle Controller")
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Create the gait controller
        self.epos = epos_controller
        self.gait_controller = GaitCycleController(epos_controller)
        
        # Initialize variables
        self.file_path_var = StringVar(value="No file loaded")
        self.cycle_duration_var = DoubleVar(value=2.0)
        self.cycles_var = IntVar(value=1)
        self.continuous_var = IntVar(value=0)
        self.gear_ratio_var = DoubleVar(value=1.0)
        self.angle_offset_var = DoubleVar(value=0.0)
        self.angle_scale_var = DoubleVar(value=1.0)
        
        # Create the GUI layout
        self._create_widgets()
        
        # Center the window
        self.window.update_idletasks()
        w = self.window.winfo_width()
        h = self.window.winfo_height()
        x = (self.window.winfo_screenwidth() - w) // 2
        y = (self.window.winfo_screenheight() - h) // 2
        self.window.geometry(f"{w}x{h}+{x}+{y}")
        
    def _create_widgets(self):
        """Create GUI widgets"""
        # File selection frame
        file_frame = tk.LabelFrame(self.window, text="Gait Data File")
        file_frame.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(file_frame, textvariable=self.file_path_var, wraplength=300).pack(side=tk.LEFT, padx=5, pady=5)
        tk.Button(file_frame, text="Load File", command=self.on_load_file).pack(side=tk.RIGHT, padx=5, pady=5)
        
        # Graph frame for data visualization
        graph_frame = tk.LabelFrame(self.window, text="Gait Cycle Visualization")
        graph_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        # Create matplotlib figure for data visualization
        self.fig, self.ax = plt.subplots(figsize=(5, 3), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Settings frame
        settings_frame = tk.LabelFrame(self.window, text="Settings")
        settings_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Cycle duration
        tk.Label(settings_frame, text="Cycle Duration (s):").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        tk.Entry(settings_frame, textvariable=self.cycle_duration_var, width=8).grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        tk.Button(settings_frame, text="Set", command=self.on_set_duration).grid(row=0, column=2, sticky=tk.W, padx=5, pady=2)
        
        # Number of cycles
        tk.Label(settings_frame, text="Number of Cycles:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        tk.Entry(settings_frame, textvariable=self.cycles_var, width=8).grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)
        tk.Checkbutton(settings_frame, text="Continuous", variable=self.continuous_var).grid(row=1, column=2, sticky=tk.W, padx=5, pady=2)
        
        # Gear ratio
        tk.Label(settings_frame, text="Gear Ratio:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        tk.Entry(settings_frame, textvariable=self.gear_ratio_var, width=8).grid(row=2, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Angle offset and scale
        tk.Label(settings_frame, text="Angle Offset (°):").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        tk.Entry(settings_frame, textvariable=self.angle_offset_var, width=8).grid(row=3, column=1, sticky=tk.W, padx=5, pady=2)
        
        tk.Label(settings_frame, text="Angle Scale:").grid(row=4, column=0, sticky=tk.W, padx=5, pady=2)
        tk.Entry(settings_frame, textvariable=self.angle_scale_var, width=8).grid(row=4, column=1, sticky=tk.W, padx=5, pady=2)
        
        tk.Button(settings_frame, text="Apply Settings", command=self.on_apply_settings).grid(row=4, column=2, sticky=tk.W, padx=5, pady=2)
        
        # Control frame
        control_frame = tk.Frame(self.window)
        control_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.start_btn = tk.Button(control_frame, text="Start", command=self.on_start, width=8)
        self.start_btn.pack(side=tk.LEFT, padx=5)
        
        self.pause_btn = tk.Button(control_frame, text="Pause", command=self.on_pause, width=8, state=tk.DISABLED)
        self.pause_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = tk.Button(control_frame, text="Stop", command=self.on_stop, width=8, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Status bar
        self.status_var = StringVar(value="Ready")
        status_bar = tk.Label(self.window, textvariable=self.status_var, bd=1, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
    def on_load_file(self):
        """Load gait data file"""
        filepath = filedialog.askopenfilename(
            title="Select Gait Data File",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")]
        )
        
        if not filepath:
            return
            
        if self.gait_controller.load_gait_data(filepath):
            self.file_path_var.set(os.path.basename(filepath))
            self.status_var.set(f"Loaded {os.path.basename(filepath)}")
            self.update_graph()
        else:
            self.status_var.set("Failed to load file")
    
    def update_graph(self):
        """Update the graph with loaded data"""
        if self.gait_controller.gait_data is not None:
            # Clear the plot
            self.ax.clear()
            
            # Plot the angle data
            self.ax.plot(self.gait_controller.gait_data['time'], 
                         self.gait_controller.gait_data['angle'], 
                         'b-', linewidth=2)
            
            # Add labels and grid
            self.ax.set_xlabel('Gait Cycle (%)')
            self.ax.set_ylabel('Hip Angle (degrees)')
            self.ax.set_title('Hip Joint Angle Profile')
            self.ax.grid(True)
            
            # Redraw the canvas
            self.fig.tight_layout()
            self.canvas.draw()
    
    def on_set_duration(self):
        """Set cycle duration"""
        try:
            duration = self.cycle_duration_var.get()
            if duration <= 0:
                messagebox.showwarning("Invalid Value", "Cycle duration must be greater than 0")
                return
                
            self.gait_controller.set_cycle_duration(duration)
            self.status_var.set(f"Cycle duration set to {duration} seconds")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to set duration: {str(e)}")
    
    def on_apply_settings(self):
        """Apply the settings"""
        try:
            # Update gear ratio
            gear_ratio = self.gear_ratio_var.get()
            if gear_ratio <= 0:
                messagebox.showwarning("Invalid Value", "Gear ratio must be greater than 0")
                return
            
            # Update angle parameters
            offset = self.angle_offset_var.get()
            scale = self.angle_scale_var.get()
            
            # Apply settings
            self.gait_controller.set_gear_ratio(gear_ratio)
            self.gait_controller.set_angle_parameters(offset, scale)
            
            # Recalculate position profile
            if self.gait_controller.gait_data is not None:
                self.gait_controller.calculate_position_profile()
                
            self.status_var.set("Settings applied successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to apply settings: {str(e)}")
    
    def on_start(self):
        """Start the gait cycle"""
        if self.gait_controller.gait_data is None:
            messagebox.showwarning("No Data", "Please load gait data first")
            return
            
        continuous = bool(self.continuous_var.get())
        cycles = self.cycles_var.get()
        
        if self.gait_controller.start_gait_cycle(continuous, cycles):
            self.start_btn.config(state=tk.DISABLED)
            self.pause_btn.config(state=tk.NORMAL)
            self.stop_btn.config(state=tk.NORMAL)
            
            if continuous:
                self.status_var.set("Running continuous gait cycles")
            else:
                self.status_var.set(f"Running {cycles} gait cycle(s)")
        else:
            self.status_var.set("Failed to start gait cycle")
    
    def on_pause(self):
        """Pause/resume the gait cycle"""
        paused = self.gait_controller.pause_gait_cycle()
        if paused:
            self.pause_btn.config(text="Resume")
            self.status_var.set("Gait cycle paused")
        else:
            self.pause_btn.config(text="Pause")
            self.status_var.set("Gait cycle resumed")
    
    def on_stop(self):
        """Stop the gait cycle"""
        if self.gait_controller.stop_gait_cycle():
            self.start_btn.config(state=tk.NORMAL)
            self.pause_btn.config(state=tk.DISABLED, text="Pause")
            self.stop_btn.config(state=tk.DISABLED)
            self.status_var.set("Gait cycle stopped")
    
    def on_closing(self):
        """Handle window closing"""
        # Stop the gait cycle if running
        if self.gait_controller.running:
            self.gait_controller.stop_gait_cycle()
        
        # Close the window
        self.window.destroy()