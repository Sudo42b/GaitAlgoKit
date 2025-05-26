#!/usr/bin/env python
import tkinter as tk
from tkinter import messagebox
import os
import sys

# Import your original EPOS controller classes
from examples.maxon_motor.epos_controller import EPOSController, EPOSControllerGUI, EPOSConstants

# Import the new gait controller classes
from examples.maxon_motor.gait_controller import GaitCycleController, GaitControllerGUI

class MainApplication:
    """Main application that integrates EPOS controller with gait controller"""
    def __init__(self, root):
        """Initialize the main application"""
        self.root = root
        self.root.title("EPOS Motor Control System")
        
        # Set icon if available
        try:
            self.root.iconbitmap("Demo_WinDLL.ico")
        except:
            print("Icon file not found.")
        
        # Create main menu
        self.create_menu()
        
        # Create EPOS controller GUI
        self.epos_gui = EPOSControllerGUI(self.root)
        
        # Add gait controller button
        gait_frame = tk.Frame(self.root)
        gait_frame.pack(fill=tk.X, padx=10, pady=10)
        
        self.gait_btn = tk.Button(gait_frame, text="Hip Gait Controller", 
                                  command=self.open_gait_controller,
                                  height=2, bg="#e6f2ff")
        self.gait_btn.pack(fill=tk.X)
        
        # Status bar
        self.status_var = tk.StringVar(value="Ready")
        status_bar = tk.Label(self.root, textvariable=self.status_var, 
                              bd=1, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Center the window
        self.root.update_idletasks()
        width = max(self.root.winfo_reqwidth(), 400)
        height = max(self.root.winfo_reqheight(), 300)
        x = (self.root.winfo_screenwidth() - width) // 2
        y = (self.root.winfo_screenheight() - height) // 2
        self.root.geometry(f"{width}x{height}+{x}+{y}")
        
        # Reference to the gait controller GUI (will be created when needed)
        self.gait_gui = None
    
    def create_menu(self):
        """Create the application menu"""
        menubar = tk.Menu(self.root)
        
        # File menu
        file_menu = tk.Menu(menubar, tearoff=0)
        file_menu.add_command(label="Exit", command=self.on_exit)
        menubar.add_cascade(label="File", menu=file_menu)
        
        # Tools menu
        tools_menu = tk.Menu(menubar, tearoff=0)
        tools_menu.add_command(label="EPOS Controller", command=self.focus_epos_controller)
        tools_menu.add_command(label="Gait Controller", command=self.open_gait_controller)
        menubar.add_cascade(label="Tools", menu=tools_menu)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        help_menu.add_command(label="About", command=self.show_about)
        menubar.add_cascade(label="Help", menu=help_menu)
        
        self.root.config(menu=menubar)
    
    def open_gait_controller(self):
        """Open the gait controller window"""
        if not self.epos_gui.controller.key_handle:
            messagebox.showwarning("Not Connected", 
                                 "Please connect to the EPOS controller first.")
            return
        
        if self.gait_gui is None or not self.gait_gui.window.winfo_exists():
            self.gait_gui = GaitControllerGUI(self.root, self.epos_gui.controller)
            self.status_var.set("Gait controller opened")
    
    def focus_epos_controller(self):
        """Focus on the EPOS controller part of the UI"""
        # Just raise the main window since EPOS controller is embedded
        self.root.lift()
        self.status_var.set("EPOS controller focused")
    
    def show_about(self):
        """Show the about dialog"""
        about_text = """EPOS Motor Control System with Gait Cycle Controller

This application allows control of maxon EPOS2 motor controllers,
with special features for implementing hip joint gait cycles.

Motor: EC 90 flat (90W)
Encoder: HEDL 5540 (500 CPT)
Controller: EPOS2
"""
        messagebox.showinfo("About", about_text)
    
    def on_exit(self):
        """Exit the application"""
        if messagebox.askyesno("Exit", "Are you sure you want to exit?"):
            # Close the gait controller if open
            if self.gait_gui is not None and self.gait_gui.window.winfo_exists():
                self.gait_gui.on_closing()
            
            # Close the EPOS controller
            self.epos_gui.on_closing()
            
            # Exit the application
            self.root.quit()

if __name__ == "__main__":
    # Set working directory to script location
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    
    # Create the main application
    root = tk.Tk()
    app = MainApplication(root)
    root.mainloop()