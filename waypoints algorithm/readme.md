# Information
  - the following codes work together to parse shp files and let operator choose waypoints based on drone flight. 
  - The program will ensure that the operator pick optimized waypoints that give the most overlap in drone imagery.
  - Files are in shp format so conver to .json before running algorithm MapChooseWP.py
  - Video Demo is at https://www.youtube.com/watch?v=haW9aEzZedo
  - Order of execution is run python3 MapChooseWP.py
  - After picking WP in MapChooseWP.py, operator can run MappingJSON.py to see the resulting map. Must run MapChooseWP.py before MappingJSON.py so the local folder can contain the xpath and ypath of the waypoints. IF operator want to see if the chosen waypoints fits with drone trajectory then run realtimeplotSHP
  - Run this on computer before going on the field because most computer has python3.
# Choosing Waypoints on Jetson Embedded Computers
  - If operator want to choose WPs out on the field and only have access to the jetson nano. python2 version of this algorithm has not been made. Kha Dan will post that version soon.
  
