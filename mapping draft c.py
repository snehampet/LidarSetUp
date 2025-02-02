import serial
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from scipy.ndimage import uniform_filter1d
import time
from matplotlib.widgets import Button

class LidarVisualizer:
    def __init__(self, port='COM4', baud=9600):
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
        except Exception as e:
            print(f"Error opening serial port {port}: {e}")
            exit(1)
        
        # Data arrays: 720 points for 0.5° resolution
        self.buffer_size = 720  
        self.angles = np.linspace(0, 2*np.pi, self.buffer_size, endpoint=False)
        self.distances = np.full(self.buffer_size, np.nan)
        
        # Create the figure with polar projection
        self.fig = plt.figure(figsize=(12, 10))
        
        # Create main polar plot
        self.ax = self.fig.add_subplot(111, projection='polar')
        
        # Adjust subplot to make room for buttons
        self.fig.subplots_adjust(bottom=0.15)
        
        # Initialize zoom level tracking
        self.current_range = 1000  # Initial range in mm
        self.min_range = 100      # Minimum range
        self.max_range = 5000     # Maximum range
        
        # Create buttons
        self.create_buttons()
        
        self.setup_plot()
        
        # Turn on interactive mode after creating the figure
        plt.ion()
        plt.show()
    
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
    
    def setup_plot(self):
        """Initialize the plot settings"""
        # Set the direction to clockwise and 0° at the top
        self.ax.set_theta_direction(-1)
        self.ax.set_theta_zero_location('N')
        
        # Set grid and labels
        self.ax.set_title('LiDAR Polar Map')
        self.ax.grid(True)
        
        # Set default radial limits (in mm)
        self.default_range = 1000
        self.current_range = self.default_range
        self.ax.set_rlim(0, self.current_range)
        
        # Update distance markers based on current range
        self.update_distance_markers()
        
        # Initialize plot elements
        self.line, = self.ax.plot([], [], '-', lw=2, color='black', alpha=0.7)
        self.scatter = self.ax.scatter([], [], s=80, alpha=0.9, c=[])
    
    def update_distance_markers(self):
        """Update the distance markers based on current range"""
        num_markers = 6
        step = self.current_range / num_markers
        markers = np.arange(step, self.current_range + step, step)
        self.ax.set_rticks(markers)
        self.ax.set_rlabel_position(0)
    
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
        self.current_range = self.default_range
        self.ax.set_rlim(0, self.current_range)
        self.update_distance_markers()
        plt.draw()
    
    def update_data(self, angle, distance):
        """Update the data array based on the given angle (in degrees)"""
        if not np.isnan(distance) and 0 <= distance <= 5000:  # Basic sanity check for distance
            index = int(round(angle * 2)) % self.buffer_size
            self.distances[index] = distance
    
    def smooth_data(self):
        """Smooth the distances using cubic interpolation and a moving average"""
        valid_mask = ~np.isnan(self.distances)
        valid_points = np.sum(valid_mask)
        
        if valid_points < 4:  # Need at least 4 points for cubic interpolation
            return self.distances
            
        try:
            # Get valid points for interpolation
            valid_angles = self.angles[valid_mask]
            valid_distances = self.distances[valid_mask]
            
            # Perform cubic interpolation
            interp_func = interp1d(valid_angles, valid_distances, kind='cubic', 
                                 bounds_error=False, fill_value=np.nan)
            smoothed = interp_func(self.angles)
            
            # Apply moving average filter
            smoothed = uniform_filter1d(smoothed, size=5)
            return smoothed
        except Exception as e:
            print(f"Smoothing error: {e}")
            return self.distances

    def get_colors(self, distances):
        """Return an array of colors based on distance ranges"""
        colors = []
        for d in distances:
            if np.isnan(d):
                colors.append('#808080')  # gray
            elif d <= 300:
                colors.append('#FF0000')  # red
            elif d <= 1000:
                colors.append('#FFFF00')  # yellow
            else:
                colors.append('#00FF00')  # green
        return colors

    def update_visualization(self):
        """Update the visualization with new data"""
        smoothed = self.smooth_data()
        valid_mask = ~np.isnan(smoothed)
        
        if np.sum(valid_mask) == 0:
            return
        
        # Get valid angles and distances
        angles_valid = self.angles[valid_mask]
        distances_valid = smoothed[valid_mask]
        
        # Update line and scatter plots
        self.line.set_data(angles_valid, distances_valid)
        self.scatter.set_offsets(np.column_stack((angles_valid, distances_valid)))
        
        # Update colors using hex codes
        colors = self.get_colors(distances_valid)
        self.scatter.set_facecolors(colors)
        self.scatter.set_edgecolors(colors)
        
        # Force a redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
    
    def process_scan(self):
        """Process the LiDAR scan data"""
        try:
            last_update = time.time()
            update_interval = 0.1  # Update interval for visualization
            
            while True:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode(errors='ignore').strip()
                    try:
                        parts = line.split(',')
                        angle = float(parts[0].split(':')[1].strip().rstrip('°'))
                        distance = float(parts[1].split(':')[1].strip().rstrip(' mm'))
                        self.update_data(angle, distance)
                    except (ValueError, IndexError) as e:
                        print(f"Error parsing data: {e}")
                
                # Update visualization periodically
                current_time = time.time()
                if current_time - last_update >= update_interval:
                    self.update_visualization()
                    last_update = current_time
                    
        except KeyboardInterrupt:
            print("\nStopping visualization...")
            self.serial.close()
            plt.close('all')

def main():
    # List available ports
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    print("Available ports:")
    for p in ports:
        print(f" - {p.device}")
    
    # You can change the port here if needed
    port = 'COM4'  # Change this to match your Arduino port
    
    try:
        viz = LidarVisualizer(port=port)
        viz.process_scan()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        plt.close('all')

if __name__ == "__main__":
    main()