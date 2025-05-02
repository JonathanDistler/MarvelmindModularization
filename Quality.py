#goal is to query quality functions
#credit to @fictionlab and the Marvelmind team

from marvelmind import MarvelmindHedge
from time import sleep
import sys
import pandas as pd 
import matplotlib.pyplot as plt

def graph(data):
    # Extract individual columns
    address = data["Address"]  # First column (address)
    quality = data["Quality"]  # Second column (quality)

    plt.plot(address, quality, label="Quality", marker='o')

    # Add labels and legend
    plt.xlabel("Address")
    plt.ylabel("Quality")
    plt.title("Address vs. Quality for Marvelmind Drone Querying")
    plt.legend()  # Show legend

    plt.show()

def main():
    hedge = MarvelmindHedge(tty = "/dev/ttyACM0", adr=None, debug=False) # create MarvelmindHedge thread
    
    if (len(sys.argv)>1):
        hedge.tty= sys.argv[1]
    
    hedge.start() # start thread
    #creating an array to store quality data
    quality=pd.DataFrame(columns=["Address", "Quality"])  # List to quality position data
    while True:
        try:
            hedge.dataEvent.wait(1)
            hedge.dataEvent.clear()
            if (hedge.qualityUpdated):
                quality=pd.concat([quality, pd.DataFrame([hedge.quality], columns=["Address","Quality"])], ignore_index=True)
                hedge.print_quality()
                
        except KeyboardInterrupt:
            hedge.stop()  # stop and close serial port
            sys.exit()
            graph(quality)
main()
