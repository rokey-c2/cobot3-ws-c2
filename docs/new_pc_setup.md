# New PC setup

This repository uses more than one dependency system. Do not put every dependency in a single Python `requirements.txt`.

## What is managed where

- Frontend: `frontend/package.json` and `frontend/package-lock.json`
- FastAPI Backend: `backend/requirements.txt` (installed by `backend/Dockerfile`)
- ROS2 packages: each real ROS package's `package.xml`, installed with `rosdep`
- P3020 vision extra Python dependency: `requirements/vision.txt`
- ROS2 MQTT Adapter extra Python dependency: `requirements/ros2_adapter.txt`
- PostgreSQL / Mosquitto / Backend services: `compose.yaml`
- Isaac Sim itself: external prerequisite, expected at `$HOME/isaacsim` unless `ISAAC_SIM_DIR` is set
- NVIDIA IW Hub ROS workspace: external prerequisite, normally `$HOME/IsaacSim-ros_workspaces/jazzy_ws`

## Platform prerequisites

Install these before running the project setup scripts:

- Ubuntu 24.04
- Isaac Sim 5.1.0
- ROS 2 Jazzy
- Docker + Docker Compose plugin
- Node.js 18+ and npm 9+
- `python3.12-venv`, `python3-rosdep`, `python3-colcon-common-extensions`

The setup scripts intentionally do not edit `~/.bashrc` and do not silently install Isaac Sim or ROS2.

## First setup

From the repository root:

```bash
cp .env.example .env
./scripts/setup_all.sh
./scripts/check_environment.sh
```

`setup_all.sh` installs project-level npm, rosdep, vision, and ROS2 MQTT Adapter dependencies after the platform prerequisites are present.

For individual setup:

```bash
./scripts/setup_frontend.sh
./scripts/install_ros_dependencies.sh
./scripts/setup_vision_env.sh
./scripts/setup_adapter_env.sh
```

## Frontend

```bash
cd frontend
npm run dev
```

The Vite development server runs on port 5173 and proxies `/api` and `/health` to the FastAPI server on port 8000.

The first successful `npm install` creates `frontend/package-lock.json`. Commit that lockfile so every PC resolves the same dependency tree.

## Backend / database / MQTT

```bash
sudo docker compose up -d --build
sudo docker compose ps
```

On a brand-new PostgreSQL volume, Docker automatically runs:

1. `backend/db/init.sql`
2. `backend/db/seed.sql`

Existing PostgreSQL volumes are not re-initialized by these scripts.

## ROS2

`./scripts/install_ros_dependencies.sh` installs dependencies declared by actual ROS packages under `ros2_ws/src`.

Some directories under `ros2_ws/src` are currently project scaffolding rather than installable ROS packages. In particular, `logistics_bringup`, `mission_manager`, and `sorter_controller` do not currently contain `package.xml`/build metadata. Their source is still placeholder/TODO-level, so this setup change does not invent package metadata for them. Add proper metadata when those components become real runnable ROS packages.

The NVIDIA `iw_hub_navigation` package is not part of this repository. `scripts/run_ros2.sh` expects the separate Isaac ROS Jazzy workspace at `$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash` when the package is not already sourced.

## Isaac Sim

Use:

```bash
./scripts/run_isaac_mission.sh
```

or the single-navigation entry point:

```bash
./scripts/run_isaac.sh
```

Both use `$HOME/isaacsim/python.sh` by default. If Isaac Sim is elsewhere:

```bash
export ISAAC_SIM_DIR=/path/to/isaacsim
```

No shell alias or `.bashrc` edit is required.

## ROS2 MQTT Adapter

After `./scripts/setup_adapter_env.sh`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
.venv/bin/python3 scripts/ros2_mqtt_adapter.py
```

## Vision

After `./scripts/setup_vision_env.sh`:

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
export ROS_DOMAIN_ID=110
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
python3 isaac_sim/robots/p3020/vision/box_detector_node.py
```
