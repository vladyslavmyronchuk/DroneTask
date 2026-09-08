import pandas as pd
import matplotlib.pyplot as plt

class FlightLogPlotter:
    def __init__(self, filepath):
        self.filepath = filepath
        self.df = None

    def load_data(self):
        try:
            self.df = pd.read_csv(self.filepath)
            print("Data loaded successfully.")
        except FileNotFoundError:
            print("Error: File not found.")
            
    def display_graph(self):
        if self.df is None:
            return

        plt.figure(figsize=(10, 6))
        
        plt.plot(self.df['Time'], self.df['Altitude'], label='Altitude (m)', color='blue', linewidth=2)
        plt.plot(self.df['Time'], self.df['Setpoint'], label='Setpoint (m)', color='red', linestyle='--', linewidth=2)
        
        plt.title('PID Altitude Control (ArduPilot SITL)')
        plt.xlabel('Time (s)')
        plt.ylabel('Height (m)')
        plt.legend()
        plt.grid(True, linestyle=':', alpha=0.7)
        
        plt.show()

def main():
    plotter = FlightLogPlotter('flight_log.csv')
    plotter.load_data()
    plotter.display_graph()

if __name__ == "__main__":
    main()