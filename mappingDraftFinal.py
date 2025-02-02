import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.widgets import Button
from math import radians
import time

class PolarLidarVisualizer:
    def __init__(self, port='COM4', baud=9600):
        # Serial connection
        self.serial = serial.Serial(port, baud, timeout=1)
        
        # Setup the main plot
        plt.ion()  # Turn on interactive mode
        self.fig = plt.figure(figsize=(10, 10))
        
        # Adjust the main plot area to make room for buttons
        self.fig.subplots_adjust(bottom=0.15)
        
        self.ax = self.fig.add_subplot(111, projection='polar')
        
        # Initialize data storage with fixed size for one complete scan
        self.buffer_size = 360  # One point per degree
        self.angles = np.linspace(0, 2*np.pi, self.buffer_size)  # Pre-allocate angles in radians
        self.distances = np.full(self.buffer_size, np.nan)  # Pre-allocate with NaN
        
        # Plot elements
        self.line = None
        self.scatter = None
        self.colorbar = None
        
        # Zoom settings
        self.current_range = 500  # Initial range in mm
        self.min_range = 100     # Minimum range
        self.max_range = 5000    # Maximum range
        
        # Custom colormap
        colors = ['blue', 'green', 'yellow', 'red']
        self.colormap = LinearSegmentedColormap.from_list('custom', colors)
        
        # Setup plot and buttons
        self.setup_plot()
        self.create_buttons()
        self.initialize_plot_elements()
        print("Waiting for Arduino data...")

    def create_buttons(self):
        """Create zoom in/out buttons"""
        # Create axes for buttons
        zoom_in_ax = plt.axes([0.7, 0.05, 0.1, 0.04])
        zoom_out_ax = plt.axes([0.81, 0.05, 0.1, 0.04])
        reset_ax = plt.axes([0.59, 0.05, 0.1, 0.04])
        
        # Create button objects
        self.zoom_in_button = Button(zoom_in_ax, 'Zoom In')
        self.zoom_out_button = Button(zoom_out_ax, 'Zoom Out')
        self.reset_button = Button(reset_ax, 'Reset')
        
        # Connect button events
        self.zoom_in_button.on_clicked(self.zoom_in)
        self.zoom_out_button.on_clicked(self.zoom_out)
        self.reset_button.on_clicked(self.reset_view)

    def zoom_in(self, event):
        """Zoom in button callback"""
        new_range = max(self.current_range * 0.7, self.min_range)
        if new_range != self.current_range:
            self.current_range = new_range
            self.ax.set_rlim(0, self.current_range)
            self.update_distance_markers()
            plt.draw()

    def zoom_out(self, event):
        """Zoom out button callback"""
        new_range = min(self.current_range * 1.4, self.max_range)
        if new_range != self.current_range:
            self.current_range = new_range
            self.ax.set_rlim(0, self.current_range)
            self.update_distance_markers()
            plt.draw()

    def reset_view(self, event):
        """Reset view button callback"""
        self.current_range = 500  # Reset to default range
        self.ax.set_rlim(0, self.current_range)
        self.update_distance_markers()
        plt.draw()

    def update_distance_markers(self):
        """Update the distance markers based on current range"""
        num_markers = 4
        step = self.current_range / num_markers
        markers = np.arange(step, self.current_range + step, step)
        self.ax.set_rticks(markers)

    def setup_plot(self):
        """Initialize plot settings"""
        self.ax.set_title('360° LiDAR Scan', pad=20)
        self.ax.set_theta_direction(-1)  # Clockwise
        self.ax.set_theta_zero_location('N')  # 0 degrees at top
        
        # Set up the grid
        self.update_distance_markers()
        self.ax.set_rlabel_position(0)
        self.ax.grid(True)
        
        # Set initial range
        self.ax.set_rlim(0, self.current_range)

    def initialize_plot_elements(self):
        """Initialize the line and scatter plots"""
        # Initialize with NaN data to create empty plots
        self.line, = self.ax.plot(self.angles, self.distances, '-', linewidth=2, color='blue', alpha=0.8)
        
        # Initialize scatter with empty data and larger points
        self.scatter = self.ax.scatter(self.angles, self.distances, 
                                     c=self.distances, cmap=self.colormap, 
                                     s=100, alpha=0.6)
        
        # Initialize colorbar
        self.colorbar = plt.colorbar(self.scatter)
        self.colorbar.set_label('Distance (mm)')

    def parse_arduino_data(self, line):
        """Parse the Arduino data format: 'Angle: X°, Distance: Y mm'"""
        try:
            parts = line.split(',')
            angle = float(parts[0].split(':')[1].strip().rstrip('°'))
            distance = float(parts[1].split(':')[1].strip().rstrip(' mm'))
            return angle, distance
        except (ValueError, IndexError):
            return None, None

    def update_data(self, angle, distance):
        """Update the data arrays at the specified angle"""
        # Convert angle to index in our arrays
        index = int(round(angle)) % 360
        self.distances[index] = distance

    def process_scan(self):
        """Process the scanning data from Arduino"""
        try:
            last_update = time.time()
            update_interval = 0.05  # Update every 50ms for smoother visualization
            
            while True:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode().strip()
                    angle, distance = self.parse_arduino_data(line)
                    
                    if angle is not None and distance is not None:
                        # Update data at this angle
                        self.update_data(angle, distance)
                        
                        # Update visualization periodically
                        current_time = time.time()
                        if current_time - last_update >= update_interval:
                            self.update_visualization()
                            last_update = current_time
                        
        except KeyboardInterrupt:
            self.serial.close()
            plt.close()

    def update_visualization(self):
        """Update the visualization with new data"""
        # Update line data
        valid_mask = ~np.isnan(self.distances)
        angles_valid = self.angles[valid_mask]
        distances_valid = self.distances[valid_mask]
        
        if len(distances_valid) > 0:
            # Update line
            self.line.set_data(angles_valid, distances_valid)
            
            # Update scatter
            self.scatter.set_offsets(np.column_stack((angles_valid, distances_valid)))
            self.scatter.set_array(distances_valid)
            
            # Update colorbar limits if we have valid data
            valid_distances = distances_valid[~np.isnan(distances_valid)]
            if len(valid_distances) > 0:
                self.scatter.set_clim(np.min(valid_distances), np.max(valid_distances))
            
            # Draw updates
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()

    def save_scan(self, filename):
        """Save current scan data to file"""
        valid_mask = ~np.isnan(self.distances)
        data = np.column_stack((
            np.degrees(self.angles[valid_mask]),
            self.distances[valid_mask]
        ))
        np.savetxt(filename, data, delimiter=',', 
                   header='angle,distance', comments='')

def main():
    # List available ports
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    print("Available ports:")
    for p in ports:
        print(f"- {p}")
    
    try:
        viz = PolarLidarVisualizer(port='COM4')  # Change to your correct port
        viz.process_scan()
    except KeyboardInterrupt:
        print("\nStopping visualization...")
    except serial.SerialException as e:
        print(f"Error with serial connection: {e}")
    finally:
        plt.close('all')

if __name__ == "__main__":
    main()