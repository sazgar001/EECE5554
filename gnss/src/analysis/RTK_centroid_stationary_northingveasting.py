import pandas as pd
import matplotlib.pyplot as plt

# Load datasets 
occluded_ods_path = '/home/sakib/gnss/src/data/Occluded_RTK_Sakib.num.ods'
open_ods_path = '/home/sakib/gnss/src/data/Open_RTK_Sakib.num.ods'

# this cleaning process is needed and referenced it on lab report paper due to CSV and ods data issues. 
def load_and_process_data(occluded_path, open_path):
    # Load the datasets
    occluded_data = pd.read_excel(occluded_path, engine='odf')
    open_data = pd.read_excel(open_path, engine='odf')
    
    # Calculate centroids using the mean of easting and northing
    centroid_occluded = (occluded_data['UTM Easting'].mean(), occluded_data['UTM Northing'].mean())
    centroid_open = (open_data['UTM Easting'].mean(), open_data['UTM Northing'].mean())

    # Re-center data by subtracting the centroid -> need to make centroid coordinates (0,0) for both plots
    occluded_data['UTM Easting'] -= centroid_occluded[0]
    occluded_data['UTM Northing'] -= centroid_occluded[1]
    open_data['UTM Easting'] -= centroid_open[0]
    open_data['UTM Northing'] -= centroid_open[1]

    return occluded_data, open_data, centroid_occluded, centroid_open

def plot_data(occluded_data, open_data, centroid_occluded, centroid_open):

    fig, ax = plt.subplots(1, 2, figsize=(14, 7), sharex=True, sharey=True)    
    # Plot centroids and include centroid coordinates in the legend
    occluded_centroid_label = f'Occluded Centroid (Purple X): (0,0)\nTotal Offset: Easting - {int(centroid_occluded[0])}, Northing - {int(centroid_occluded[1])}'
    open_centroid_label = f'Open Centroid (Purple Dot): (0,0)\nTotal Offset: Easting - {int(centroid_open[0])}, Northing - {int(centroid_open[1])}'
    
    # Plot parameters
    ax[0].scatter(occluded_data['UTM Easting'], occluded_data['UTM Northing'], color='red', label='Occluded Data', alpha=0.7, marker='x') # dot customizations for plot 1 - googled this
    ax[0].set_title('RTK Stationary Occluded: Northing vs Easting Plot') # title for plot 1
    ax[0].plot(0, 0, 'x', color='purple', markersize=10, label=occluded_centroid_label) # centroid specification dots
    ax[0].set_xlabel('Easting (m)') # x-axis label for plot 1
    ax[0].set_ylabel('Northing (m)') #y-axis label for plot 1
    ax[0].legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), shadow=True, ncol=2) # legend customizations for plot 1 - googled this
    ax[0].grid(True) # had grid lines on for plot 1
    ax[1].scatter(open_data['UTM Easting'], open_data['UTM Northing'], color='darkblue', label='Open Data', alpha=0.7, marker='o') # dot customizations for plot 2 - googled this
    ax[1].set_title('RTK Stationary Open: Northing vs Easting Plot') # title for plot 2
    ax[1].plot(0, 0, 'o', color='purple', markersize=10, label=open_centroid_label) # centroid specification dots
    ax[1].set_xlabel('Easting (m)') # x-axis label for plot 2
    ax[1].set_ylabel('Northing (m)') # y-axis label for plot 2
    ax[1].legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), shadow=True, ncol=2) # legend customizations for plot 2 - googled this
    ax[1].grid(True) # had grid lines on for plot 2
    plt.tight_layout()
    plt.show()

# Main function to run the process
def main():
    # Load and process data from occluded and open ODS files, storing results in variables
    occluded_data, open_data, centroid_occluded, centroid_open = load_and_process_data(occluded_ods_path, open_ods_path)
    # Plot the processed data, including occluded and open data, and their centroids
    plot_data(occluded_data, open_data, centroid_occluded, centroid_open)
    
if __name__ == '__main__': # If this script is being executed directly (not imported as a module), call the main function
    main()
