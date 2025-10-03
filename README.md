````markdown
# Floorplan Door Placement Algorithm

This project implements an algorithm to automatically place doors in floorplans.  
The input is a JSON file describing a house boundary and rooms (each with walls or polygons), and the output is a set of door placements that ensure the house is **coherent** (doors are placed along shared walls) and **traversable** (all rooms are connected).

The repository includes:
- An algorithm to detect room adjacencies.
- A door placement strategy.
- Visualization tools for rendering the floorplan with door locations using `matplotlib`.

---

## Features
- ✅ Parse floorplans in JSON format (walls or polygons).  
- ✅ Detect shared walls between adjacent rooms.  
- ✅ Place doors at appropriate positions (midpoints of shared walls).  
- ✅ Handles floating-point inaccuracies in input coordinates.  
- ✅ Visualize results with labeled rooms and door markers.

---

## Installation

Clone the repository and install the required dependencies:

```bash
git clone https://github.com/yourusername/floorplan-doors.git
cd floorplan-doors
pip install -r requirements.txt
````

Dependencies
