import pandas as pd
from geopy.distance import geodesic
import time
from searching_algorithms import *
# import matplotlib.pyplot as plt
import gmplot 
import webbrowser

# read coordinates file: import pandas as pd
coordinates_df = pd.read_csv("coordinates.csv", header=None)
town_coordinates = {}
index_town = {}
for index, row in coordinates_df.iterrows():
  town_coordinates[row.iloc[0]] = (row.iloc[1], row.iloc[2])
  index_town[index] = row.iloc[0]

#read adjacencies file to create graph:
graph={}
with open('adjacencies.txt', 'r') as f:
    lines = f.readlines()
    for line in lines:
      towns = line.split()
      if towns[0] not in graph:
        graph[towns[0]] = [towns[1]]
      elif towns[1] not in graph[towns[0]]:
        graph[towns[0]].append(towns[1])
      if towns[1] not in graph:
        graph[towns[1]] = [towns[0]]
      elif towns[0] not in graph[towns[1]]:
        graph[towns[1]].append(towns[0])

#find distance between 2 towns: from geopy.distance import geodesic
def find_distance(town1, town2):
  return geodesic(town_coordinates[town1], town_coordinates[town2]).miles

#find total_distance:
def total_distance(path):
  total_distance = 0
  for index, town in enumerate(path):
    if index + 1 < len(path):
      next_town = path[index+1]
      total_distance += find_distance(town, next_town)
  return total_distance

#find path and processing time:
def find_path_and_processing_time(find_path_func, *args, **kwargs):
    start_time = time.perf_counter()
    path = find_path_func(*args, **kwargs)
    end_time = time.perf_counter()
    execution_time = end_time - start_time
    # Formatting execution_time to display up to 9 decimal places
    execution_time = "{:.9f}".format(execution_time)
    return path, execution_time
  
# def displayGraph(locations,adjacency, path):
#   fig, ax = plt.subplots()

#   for location, (latitude, longitude) in locations.items():
#       ax.scatter(longitude, latitude)  # Plot points
#       ax.text(longitude, latitude, location)  # Label points


#   for town in adjacency:
#       start_point = locations[town]
#       for adjacent_town in adjacency[town]:
#           end_point = locations[adjacent_town]
#           ax.plot([start_point[1], end_point[1]], [start_point[0], end_point[0]], color='lightgrey', lw=1)  # 'k-' means black line, lw is line width

#   # Draw the line
#   # Note: You need to pass the longitudes as the first list and latitudes as the second list
#   for index, town in enumerate(path):
#       if index != 0:
#           start_point = locations[path[index-1]]  # (lat, long)
#           end_point = locations[path[index]]  # (lat, long)
#           ax.plot([start_point[1], end_point[1]], [start_point[0], end_point[0]], color='red', lw=2)  # 'k-' means black line, lw is line width

#   # Set labels and title
#   ax.set_xlabel('Longitude')
#   ax.set_ylabel('Latitude')
#   ax.set_title(f'Path from {path[0]} to {path[-1]}')

#   # Show the plot
#   plt.show()

def generate_google_map_with_path(path):
  latitude_list =[]
  longitude_list = []
  for town in path:
    latitude_list.append(town_coordinates[town][0])
    longitude_list.append(town_coordinates[town][1])
  # Calculate the average latitude and longitude
  center_latitude = sum(latitude_list) / len(latitude_list)
  center_longitude = sum(longitude_list) / len(longitude_list)
  gmap = gmplot.GoogleMapPlotter(center_latitude, 
                                center_longitude, zoom=9,apikey='AIzaSyDM-2NAnk6FmhY5xt1v4RWWl4QA9JvcGPw') 

  gmap.scatter( latitude_list, longitude_list, 'red', 
                              size = 40, marker = False ) 
  
  gmap.plot(latitude_list, longitude_list,  
           'cornflowerblue', edge_width = 2.5)
  
  gmap.draw('map.html')
  
  webbrowser.open('map.html')
  
# program startup: display all available towns:
print("Available towns:")
counter = 0
for key in town_coordinates.keys():
    print(counter, key, end='\t')
    counter += 1
    # If five keys have been printed, start a new row
    if counter % 5 == 0:
        print()  # Start a new line

if counter % 5 != 0:
    print()  # Start a new line

# prompt user for source and destination input
source = input("\nPlease select source town: ")
source = index_town[int(source)]
print(source)
destination = input("\nPlease select destination town: ")
destination = index_town[int(destination)]
print(destination)

while True:
  # prompt user for search method
  search_method_options = ['Depth First Search','Breadth First Search', 'Iterative Deepening Depth-First Search','Best First Search','A * Search']
  print("\nAvailable search methods:")
  for index,method in enumerate(search_method_options):
    print(index+1,method)
  method_number = input("\nPlease select a search method: ")
  print(search_method_options[int(method_number) - 1])
  search_method = {1 : depth_first_search,
                  2: breadth_first_search,
                  3:id_depth_first_search,
                  4: best_first_search,
                  5:astar_search}
  
  method = search_method[int(method_number)]
  print(method)
  #use user selected method to find path
  if method_number not in ['4','5']:
    path, processing_time = find_path_and_processing_time(method, graph, source, destination,path=[], visited=set())  # for dfs, bfs, id-dfs
  else:
    path, processing_time = find_path_and_processing_time(method, graph, source, destination, find_distance) # for best-fs

  #calculate distance
  if path:
    distance = total_distance(path)

  #output result
  if path:
      print(f"\nPath from {source} to {destination}: {path}")
      print(f"Total distance is: {distance} miles")
      #displayGraph(town_coordinates, graph, path)
      generate_google_map_with_path(path)
  else:
      print(f"No path found from {source} to {destination}.")
      
  print(f"Execution time is: {processing_time}")
  continue_choice = input('\nDo you want to try another search method? 1 for Yes/ any number for No:')
  if continue_choice != '1': break


print("\nProgram ended")

