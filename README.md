# Assignment 1: Pathfinding in Artificial Intelligence and Multi-Agent Systems
## Course Details
- Course: Artificial Intelligence and Multi-Agent Systems
- Institution: KTH Royal Institute of Technology
Project Overview
This project involves developing efficient pathfinding algorithms for a drone and a car, aiming to reach their designated goals as swiftly as possible. The focus is on leveraging advanced algorithms to handle the navigation challenges posed by different terrains and obstacles typically encountered in autonomous navigation.

## Implementation
Hybrid A-Star Algorithm
The core of the pathfinding logic is built around the Hybrid A-Star algorithm. This algorithm is particularly effective for navigating in spaces with complex constraints, as it combines the heuristic approach of A* with the practical dynamics of vehicles. The Hybrid A-Star was tailored to account for:
- Vehicle dimensions
- Turning radius constraints for both drone and car
- Dynamic obstacles and varied terrains

## PD Controller for Path Tracking
To ensure that both the drone and the car accurately follow the computed paths:

A Proportional-Derivative (PD) controller was implemented.
This controller adjusts the steering and speed based on the deviation from the path, allowing for real-time corrections and smooth navigation.
