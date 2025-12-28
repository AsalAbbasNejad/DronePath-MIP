Multi Drone Building Inspection Routing

This project solves a routing optimization problem for a consulting company that inspects buildings using drones. The company must visit a predefined set of 3D measurement points on the exterior of a building. There are 4 drones, they all start at a given base position, they take off simultaneously, each drone visits a subset of points, and all drones must return to base. The goal is to minimize the time taken by the slowest drone to complete its route.

Two building instances are provided:

Edificio1.csv

Edificio2.csv

Each file contains a collection of spatial coordinates that must be visited exactly once, and the program computes optimal drone tours according to the constraints and speed model of the drones 

Small_Project_Description

Problem Summary

Exactly 4 drones are available.

All drones start from a fixed base coordinate.

Each drone may travel through a limited connectivity graph defined by geometric rules.

Every measurement point must be visited once by exactly one drone.

Every drone must return to the base at the end of its tour.

Movement time depends on vertical and horizontal velocity, including different speeds for upward and downward motion.

The objective is to minimize the maximum completion time among all drones (minimax routing).

Connectivity between points is allowed only if they meet geometric criteria from the problem statement. Entry points to the building grid are restricted as required in the assignment specification 

Small_Project_Description

Approach

The problem is modeled as a mixed integer optimization problem using the mip Python library. The code includes:

Calculation of valid movement arcs based on problem rules.

Travel time computation based on direction and movement type.

Filtering of reachable nodes from the base.

A minimax multi drone variant of the Traveling Salesman Problem.

MTZ constraints to eliminate subtours.

Extraction of drone routes after optimization.

All logic is implemented in main.py 

main

Requirements

You need Python and the following packages:

numpy

pandas

mip

Install with:

pip install numpy pandas mip

How to Run

Unzip the submission folder, then run:

python main.py Edificio1.csv


or

python main.py Edificio2.csv


The program automatically detects which instance is being solved, applies the correct base location and entry conditions, builds the optimization model, solves it, and prints the result.

Output Format

For each drone i, the program prints:

Drone i: 0-a-b-c-...-0


Where:

0 is the base point.

Intermediate numbers are point indices in the visited order.

The route always returns to 0.

Example:

Drone 1: 0-4-11-17-2-0
Drone 2: 0-5-6-3-7-0
Drone 3: 0-9-0
Drone 4: 0-12-0


This format follows the exact requirement of the assignment specification so that evaluators can automatically verify correctness 

Small_Project_Description

File Structure

main.py solver and routing implementation

Edificio1.csv building instance 1

Edificio2.csv building instance 2

Notes

If no feasible solution exists, the code safely handles it and avoids printing incorrect output.

Time limit and solver configuration can be adjusted in the code if needed.
