# Behavior Tree GUI Application for Cobots in Manufacturing

## Overview

This repository contains a **Behavior Tree GUI application** designed for controlling collaborative robots (cobots) in a manufacturing environment. The application leverages behavior trees—a modular and hierarchical approach—to manage complex robot behaviors efficiently. This tool allows users to visually create, manage, and execute behavior trees that guide cobot actions, incorporating both synchronous and asynchronous logic.

## Features

### 1. Drag-and-Drop Interface
- **Ease of Use:** Users can drag and drop nodes onto the canvas to construct behavior trees visually.
- **Node Types:** Includes different node types, such as synchronous and asynchronous actions, represented by different shapes and colors.

### 2. History Management
- **Version Control:** The History Widget allows users to save different versions of behavior trees, making it easy to revert to or modify previous configurations.
- **Preview:** Users can preview saved behavior trees and load them back onto the canvas with a single click.

### 3. Synchronous and Asynchronous Logic
- **Color-Coded Nodes:** Nodes are color-coded based on their execution logic—blue for synchronous and green for asynchronous actions.
- **Custom Colors:** Users can customize the color of nodes for better visual management.

### 4. SVG Graphics
- **Scalable Graphics:** SVGs are used for node representation, ensuring clear and sharp visuals at any zoom level.

### 5. Save and Load Functionality
- **Persistence:** Save behavior trees as JSON files, including all node properties, connections, and custom settings.
- **Compound Nodes:** Combine multiple behavior trees into a single compound node for more complex operations.

## Challenges Faced and Solutions

### 1. **Introducing SVG for Sharp Graphics**
   - **Challenge:** The initial graphics for nodes were pixelated at different zoom levels.
   - **Solution:** Implemented SVG graphics to ensure scalability without losing quality.

### 2. **Handling Synchronous and Asynchronous Logic**
   - **Challenge:** Differentiating between synchronous and asynchronous tasks within the tree.
   - **Solution:** Implemented color-coded nodes and enhanced the logic to handle asynchronous tasks efficiently.

### 3. **Maintaining Node State Across History Versions**
   - **Challenge:** Ensuring that when behavior trees are saved and reloaded, the node states (like colors) remain consistent.
   - **Solution:** Extended the save/load functionality to include all relevant node attributes, including manual color settings.

## Future Plans

- **Enhanced Node Types:** Introduce more node types for handling different logical operations.
- **Integration with ROS:** Direct integration with ROS for real-time cobot control.
- **User Collaboration:** Enable multi-user collaboration on behavior tree design in real-time.

## To-Do

- [ ] Implement real-time ROS integration.
- [ ] Add more complex node types and behaviors.
- [ ] Enable export of behavior trees to other formats.
- [ ] Improve user collaboration features.
- [ ] Optimize performance for larger trees with many nodes.

