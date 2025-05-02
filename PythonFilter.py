from marvelmind import MarvelmindHedge
from time import sleep, time  # Import time()
import sys
import pandas as pd 
import matplotlib.pyplot as plt

# Function to graph the collected data
def graph(data):
    # Convert Series to NumPy arrays before plotting
    x_vals = data["X"].to_numpy()
    y_vals = data["Y"].to_numpy()
    z_vals = data["Z"].to_numpy()
    time_vals = data["Time"].to_numpy()

    # Plot the data
    plt.plot(time_vals, x_vals, label="X values", marker='o')
    plt.plot(time_vals, y_vals, label="Y values", marker='s')
    plt.plot(time_vals, z_vals, label="Z values", marker='^')

    # Add labels and legend
    plt.xlabel("Time")
    plt.ylabel("Position")
    plt.title("X, Y, and Z positions over Time for Marvelmind Drone Querying with Position Averaging")
    plt.legend()
    plt.show()

def main():
    hedge = MarvelmindHedge(tty="/dev/ttyACM0", adr=None, debug=False)  # Create MarvelmindHedge thread
    
    if len(sys.argv) > 1:
        hedge.tty = sys.argv[1]
    
    hedge.start()  # Start thread

    hedge_pos = pd.DataFrame(columns=["X", "Y", "Z", "Time"])  # Store positional data
    hedge_pos_filter = pd.DataFrame(columns=["X", "Y", "Z", "Time"])  # Store filtered data

    sum_count = 5  # Number of last values to average
    initial_time = time()  # Capture the starting time

    while True:
        try:
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()

            if hedge.positionUpdated:
                x = hedge.position()[1]
                y = hedge.position()[2]
                z = hedge.position()[3]
                t = time() - initial_time  # Compute time relative to start
                
                new_row = pd.DataFrame({"X": [x], "Y": [y], "Z": [z], "Time": [t]})
                hedge_pos = pd.concat([hedge_pos, new_row], ignore_index=True)

                if len(hedge_pos) >= sum_count:
                    avg = hedge_pos.iloc[-sum_count:].mean()  # Compute the mean of the last `sum_count` rows
                    avg_df = pd.DataFrame([avg])  # Convert the Series to DataFrame for proper concatenation
                    hedge_pos_filter = pd.concat([hedge_pos_filter, avg_df], ignore_index=True)

                    print(avg)  # Print the averaged values

        except KeyboardInterrupt:
            hedge.stop()  # Stop and close serial port
            graph(hedge_pos_filter)
            sys.exit()

# Call the main function
main()
