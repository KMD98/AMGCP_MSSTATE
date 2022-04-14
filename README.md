# Autonomous Off-road 4WD Robot
  - This is a code repository for Mississippi State University ABE and GRI department for the autonomous mobile ground control point robot that works in collaboration with drone flight plan.
  - The robot runs 2 sensor fusion node. Visual inertial (ZED) in its local frame. GPSRTK and IMU in the global frame. All global frame trajectory is translate and rotate into local frame instead of the opposite direction to reduce uncertainty and computational cost. 
  - The robot perform collision avoidance with A* algorithm on a 2D occupancy map created from stereo 3D Point Cloud
  - The robot navigates based on chosen waypoints
  - The robot perform object detection
  - The robot design DOES NOT HAVE BRAKES. It is at the mercy of inertia
  - The robot weighs 400lbs
## Support
  - If you run into problems or have questions. Reach out to me at khadan.manh@gmail.com or dan@gri.msstate.edu
  - Reach out to Collin Mcleod if I do not answer on time. cm2928@msstate.edu
  - Our principal investigator is Alex Thomasson. athomassons@abe.msstate.edu
