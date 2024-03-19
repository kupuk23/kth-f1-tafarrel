import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np


class DataVisualizerApp:
    def __init__(self, master):
        self.master = master
        self.master.title("KTH F1 Exercise 2 - Data Visualizer")
        self.period = 50  # Period in ms

        # Variables
        self.exp_name = "Experiment"
        
        self.running = False
        self.time = np.arange(0, 100, self.period / 1000.0)
        self.current_time = np.array([])
        self.data = np.array([])
        self.idx = 0  # Index for time array

        self.h_values = self.compute_values()
        # Setup GUI

        self.setup_gui()

    def lambda_t(self, t):
        return 5 * np.sin(2 * np.pi * 1 * t)

    def h_t(self, t):
        return 3 * np.pi * np.exp(-self.lambda_t(t))

    def compute_values(self):
        return self.h_t(self.time)

    def setup_gui(self):
        # Plotting area
        self.fig, self.ax = plt.subplots()
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.grid(row=0, column=0, columnspan=4)

        # Start btn
        self.start_btn = ttk.Button(self.master, text="Start", command=self.start)
        self.start_btn.grid(row=1, column=0)

        # Stop btn
        self.stop_btn = ttk.Button(self.master, text="Stop", command=self.stop)
        self.stop_btn.grid(row=1, column=1)

        # Reset btn
        self.reset_btn = ttk.Button(self.master, text="Reset", command=self.reset)
        self.reset_btn.grid(row=1, column=2)

        # Save btn
        self.save_btn = ttk.Button(self.master, text="Save", command=self.save_data)
        self.save_btn.grid(row=1, column=3, pady=10)

        # Experiment name field
        self.exp_name_label = ttk.Label(self.master, text="Experiment Name:")
        self.exp_name_label.grid(row=2, column=0)
        self.exp_name_entry = ttk.Entry(self.master)
        self.exp_name_entry.grid(row=2, column=1, columnspan=2)

        self.update_title_btn = ttk.Button(
            self.master, text="Update Title", command=self.update_title
        )
        self.update_title_btn.grid(row=2, column=2,columnspan=3, pady=10)

        # show the canvas
        self.ax.grid(True)
        self.ax.set_title(self.exp_name)
        self.canvas.draw()
        # self.master.grid_rowconfigure(4, minsize=10)

    def update_title(self):
        # Retrieve the experiment name from the entry
        self.exp_name = self.exp_name_entry.get()

        # Update the chart title
        self.ax.set_title(self.exp_name)
        self.canvas.draw()

    def start(self):
        if self.running:  # to prevent updating the plot twice
            return
        self.running = True
        self.update_plot()

    def stop(self):
        self.running = False

    def reset(self):
        self.current_time = np.array([])
        self.data = np.array([])
        self.idx = 0  # Index for time array
        self.ax.clear()
        self.ax.grid(True)
        self.canvas.draw()

    def save_data(self):
        if self.data is not None and len(self.data) > 0:
            exported_data = np.vstack((self.current_time, self.data)).T
            filename = "exported_data.csv"
            np.savetxt(filename, exported_data, delimiter=",")
            tk.messagebox.showinfo(f"Data saved as {filename}")

    def update_plot(self):
        if self.running:
            self.idx += 1
            # Simulate data acquisition
            self.current_time = np.append(self.current_time, self.time[self.idx])
            self.data = np.append(self.data, self.h_values[self.idx])

            # Update plot
            self.ax.clear()
            self.ax.set_xlabel("Time (s)")
            self.ax.set_ylabel("h(t)")
            self.ax.set_ylim(0, max(self.h_values) + 500) #adjust axis based on min max value of h_values
            self.ax.set_xlim(0, max(self.current_time) + 10)
            self.ax.set_title(self.exp_name)
            self.ax.plot(self.current_time, self.data, linestyle="-",linewidth=2, color="red")
            self.ax.grid(True)
            
            self.canvas.draw()

            # Schedule next update
            self.master.after(self.period, self.update_plot)


if __name__ == "__main__":
    root = tk.Tk()
    app = DataVisualizerApp(root)
    root.mainloop()
