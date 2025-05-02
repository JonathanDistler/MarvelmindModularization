from marvelmind import MarvelmindHedge
from time import sleep, time
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
    
# Main function to interact with the Marvelmind Hedge and collect data
def main():
    hedge = MarvelmindHedge(tty="/dev/ttyACM0", adr=None, debug=False)  # Create MarvelmindHedge thread
    
    if len(sys.argv) > 1:
        hedge.tty = sys.argv[1]  # If a different tty is provided, use it
    
    hedge.start()  # Start the thread

    # DataFrame to store position data [x, y, z, time]
    hedge_pos = pd.DataFrame(columns=["X", "Y", "Z", "Time"])

    # Initialize the time tracker
    start_time = time()  # Get the starting time (timestamp)

    while True:
        try:
            hedge.dataEvent.wait(1)  # Wait for data with a 1-second timeout
            hedge.dataEvent.clear()  # Clear the event after waiting for data

            if hedge.positionUpdated:  # If new position data is available
                # Fetch the data (x, y, z)
                x = hedge.position()[1]
                y = hedge.position()[2]
                z = hedge.position()[3]
                
                # Calculate the elapsed time from start
                elapsed_time = time() - start_time  # Elapsed time since the program started
                
                # Debugging: Print the current position data and time
                print(f"Position: X={x}, Y={y}, Z={z}, Time={elapsed_time}")

                # Create a new row to append to the DataFrame
                new_row = pd.DataFrame({"X": [x], "Y": [y], "Z": [z], "Time": [elapsed_time]})

                # Append the new row to the existing DataFrame
                hedge_pos = pd.concat([hedge_pos, new_row], ignore_index=True)

            # To prevent overloading the CPU, add a short sleep (0.1 seconds)
            sleep(0.1)

        except KeyboardInterrupt:
            # Handle the keyboard interrupt to stop the program
            hedge.stop()  # Stop and close the serial port connection
            graph(hedge_pos)  # Call the graph function to plot the collected data
            sys.exit()  # Exit the program

# Run the main function
main()
