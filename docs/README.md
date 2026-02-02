# OpenAMRobot UI – Architecture, Structure, and Modules

This document is the single, comprehensive description of the OpenAMRobot UI repository.
It explains the full structure, responsibilities, modules, data flow, and ownership model
of the system.

This file is intended to be:
- readable by beginners
- precise for developers
- the single source of truth for repository structure

---

## System overview

The OpenAMRobot UI system consists of two tightly coupled parts:

1. Frontend (React)
2. Backend (ROS2)

The frontend provides the user interface and visualization.
The backend provides robot interaction, navigation orchestration, persistence,
and serves the frontend as static files.
```
Communication between frontend and backend is done exclusively via:
- rosbridge_server
- roslibjs (WebSocket)
```
The frontend is built into static files and served by the ROS2 backend using Flask.

---

## Top-level repository structure

.
├── docs/    
├── scripts/    
├── web/    
├── ros2/    
├── compile_ui.bash    
├── install_openamr_deps.bash    
└── launch.bash    

---

## docs/

**Purpose:**
- architecture definition
- repository structure explanation
- system-level documentation

This file (`docs/README.md`) is the single source of truth for system architecture.

---

## scripts/

**Purpose:**
- standardize workflows
- automate build and launch
- reduce manual steps and errors

Scripts in this folder are the canonical way to work with the project.

Root-level scripts (`compile_ui.bash`, `install_openamr_deps.bash`, `launch.bash`)
exist for backward compatibility only.

---

## web/ – Frontend

**Purpose:**
- user interface
- visualization
- operator interaction
- sending commands to ROS2 backend

### Frontend structure

web/    
├── package.json    
├── public/    
└── src/    
├── index.js    
├── app/    
│ ├── App.jsx    
│ └── store/    
├── pages/    
├── components/    
├── stores/    
└── shared/    

### Frontend entry points

**index.js**  
- React application bootstrap.

**app/App.jsx**  
- Application root.
- Creates ROS WebSocket connection.
- Provides RosContext.
- Wires Redux store and router.

### Frontend folder responsibilities

**pages/**  
- Page-level UI components.
- Examples: Map, Route, Control, Info.

**components/**  
- Reusable UI widgets and visual components.

**stores/**  
- Redux global state.
- State that must persist across pages.

**shared/**  
- Constants, helpers, UI primitives.
- No direct ROS logic.

**app/**  
- Application bootstrap.
- Providers, routing, store wiring.

---

## Frontend ROS communication

**React Component**
-> RosContext
-> ROSLIB.Topic / ROSLIB.Service
-> rosbridge_server
-> ROS2 backend

---

## ros2/ – Backend

**Purpose:**
- UI backend logic
- robot interaction
- navigation coordination
- persistence
- serving frontend UI

### ROS2 packages

ros2/    
├── openamr_ui_msgs/    
└── openamr_ui_package/    

---

## openamr_ui_msgs

**Purpose:**
- custom ROS2 interfaces used by the UI backend

**Contents:**
- msg/
- srv/
- action/
- package.xml
- CMakeLists.txt

These interfaces define structured communication for:
- UI commands
- UI status updates
- route and navigation feedback

---

## openamr_ui_package

**Purpose:**
- main UI backend implementation
- filesystem-backed map and route handling
- navigation orchestration
- battery monitoring
- serving frontend UI

### Package structure

openamr_ui_package/    
├── launch/    
├── maps/    
├── paths/    
├── param/    
└── openamr_ui_package/    
├── flask_app.py    
├── folders_handler.py    
├── waypoint_nav.py    
└── battery.py    

---

## Backend modules

**flask_app.py**  
- HTTP server for built React UI.
- Serves static frontend files.
- Supports single-page application routing.

**folders_handler.py**  
- Central UI backend coordinator.
- Manages maps, routes, and waypoints.
- Bridges UI commands to ROS services and topics.
- Persists UI state to filesystem.

**waypoint_nav.py**  
- Executes waypoint and route sequences.
- Sends navigation goals.
- Reports execution progress.

**battery.py**  
- Monitors battery state.
- Triggers docking logic.
- Publishes battery status to the UI.

---

## Backend resources

**maps/**  
- Navigation maps.

**paths/**  
- Route and waypoint definitions.

**param/**  
- Configuration files.
- Persistent UI state.

**Key file:**
- param/current_map_route.yaml  
  Stores currently selected map and route.

---

## Dependencies

**Frontend:**
- Node.js
- React
- roslibjs

**Backend:**
- ROS2
- Python 3
- Flask
- rosbridge_server
- nav2 stack (optional but typical)

---

## Ownership model

**Frontend owns:**
- layout
- visualization
- user interaction

**Backend owns:**
- robot control
- navigation logic
- persistence
- system safety

  ---
