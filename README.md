![photo_2025-12-14_17-17-00](https://github.com/user-attachments/assets/ab25302f-7dd3-4bb2-b95d-4c41586eda74)

🚁 ##Multi Drone Building Inspection Routing

This project implements an optimization model for coordinating four autonomous drones to inspect building exteriors. Each building file provides a set of 3D measurement points that must be visited. All drones start together from a base, split the workload, and return to base. The goal is to minimize the total mission duration, meaning the time when the last drone lands back at base. ⏱️

Supported datasets
📄 Edificio1.csv

📄 Edificio2.csv

🔍 Problem Overview

A consulting company uses drones to scan building surfaces with cameras and sensors. The mission must respect these conditions:

✔️ Four drones available

✔️ Shared starting base point

✔️ Every point visited exactly once

✔️ Each drone must return to base

✔️ Direction dependent speeds

⬆️ Upward 1 m/s
⬇️ Downward 2 m/s
➡️ Horizontal 1.5 m/s

✔️ Oblique motion handled by dominant movement time

✔️ Strict geometric connectivity rules

✔️ Restricted entry points between base and grid

✔️ Objective is to minimize the slowest drone completion time

Optimization + geometry + physics working together. Nice. 🤓

🧠 Solution Approach

This project uses a Mixed Integer Programming model powered by the mip library. Key steps include:

🗂️ Reading 3D coordinates from CSV

🧭 Building connectivity graph from geometric rules

🕑 Computing travel time from speed constraints

🧹 Filtering unreachable nodes

📐 Solving a minimax multi drone Traveling Salesman Problem

🚫 Preventing subtours using MTZ constraints

📤 Extracting and printing final drone paths


The solver produces efficient and valid drone routes.

⚙️ Requirements

Python 3.8 or newer recommended.

Install required packages:

pip install numpy pandas mip

▶️ Running the Program

Run using one of the building datasets:

python main.py Edificio1.csv


or

python main.py Edificio2.csv


The script automatically detects the correct base setup and entry conditions for each building.

📌 Output Format

Each drone prints its full tour starting and ending at base point 0.

Example output:

Drone 1: 0-4-11-17-2-0

Drone 2: 0-5-6-3-7-0

Drone 3: 0-9-0

Drone 4: 0-12-0



Clean, readable, easy to evaluate. ✔️

📂 Repository Structure

🧠 main.py optimization solver and routing logic

🏢 Edificio1.csv dataset 1

🏙️ Edificio2.csv dataset 2


💡 Notes

🔎 Automatically ignores unreachable nodes

❌ Safe handling of infeasible solutions

⏳ Runtime depends on solver limits and geometry complexity

