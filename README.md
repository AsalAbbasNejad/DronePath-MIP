Multi Drone Building Inspection Routing

This project implements an optimization model for coordinating multiple autonomous drones to inspect the exterior of buildings. Each building instance provides a set of 3D measurement points that must be visited. Four drones start simultaneously from a fixed base point, each visits a distinct subset of locations, and all must return to the base. The objective is to minimize the total operation time, defined as the time of the last drone to complete its route.

The project solves two provided problem instances:

Edificio1.csv

Edificio2.csv

Problem Overview

A consulting company deploys drones equipped with sensors and cameras to scan building surfaces. The inspection must satisfy these key constraints.

Exactly four drones are available

All drones start from a shared base point

Every point must be visited exactly once by one drone

Each drone must return to the base

Movement speed is direction dependent

Upward flight: 1 m/s

Downward flight: 2 m/s

Horizontal flight: 1.5 m/s

Oblique movement time is determined based on dominant motion

Drones may only travel between grid points that satisfy geometric connectivity rules

Only specific entry points allow access between the base and the building grid

The objective is to minimize the maximum completion time among all drones

Solution Approach

The problem is formulated as a Mixed Integer Programming model using the mip library. The implementation includes:

Reading 3D point data from CSV files

Constructing a connectivity graph based on geometric constraints

Computing travel time between reachable coordinates according to vertical and horizontal speed limitations

Filtering unreachable points through graph traversal

Solving a minimax version of the multi drone Traveling Salesman Problem

Using MTZ constraints to eliminate subtours

Extracting and printing final drone routes

The solver returns feasible routes that collectively cover all measurement points while optimizing task duration.

Requirements

Python 3.8 or newer is recommended.

Install dependencies:

pip install numpy pandas mip

Running the Program

Run the solver by providing one of the building CSV files as argument:

python main.py Edificio1.csv


or

python main.py Edificio2.csv


The program automatically detects the instance, loads building geometry, builds the optimization model, solves it, and prints the resulting drone paths.

Output Format

The program prints one line per drone showing its route, starting from base point 0 and returning to it.

Example:

Drone 1: 0-4-11-17-2-0
Drone 2: 0-5-6-3-7-0
Drone 3: 0-9-0
Drone 4: 0-12-0


Each sequence represents the order of visited measurement nodes.

Repository Structure

main.py
Core solver script containing optimization model, time calculations, connectivity logic, and output formatting.

Edificio1.csv
First building inspection dataset.

Edificio2.csv
Second building inspection dataset.

Notes

The solver automatically limits to reachable nodes

If no feasible solution exists, the script exits without incorrect output

Computation time depends on instance complexity and solver limits
