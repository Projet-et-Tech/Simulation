# ROS Docker Development Environment

This repository provides a Docker-based development setup for working with ROS (Robot Operating System) using GUI support. Follow the steps below to install, build, and enable GUI for your ROS development environment.

## Prerequisites

- Ensure you have Docker installed on your system. For installation instructions, see the [Docker documentation](https://docs.docker.com/get-docker/).
- If you'll be working with a GUI, you **MUST** have **Linux installed**. (instructions under __Enable GUI (Linux)__)

## Setup Instructions

### 1. Install Docker
If Docker is not installed, follow the instructions to install Docker for your operating system from the [Docker website](https://docs.docker.com/get-docker/).

### 2. Build Docker Image

Build the Docker image using the provided Dockerfile:
```bash
docker build -t ros-docker .
```

This command creates a Docker image named `ros-docker` using the configurations in the Dockerfile

### 3. Enable GUI 

To use GUI applications within the Docker container on Linux, run the following command on the host:
```bash
xhost +SI:localuser:$(whoami)
```
This command configures the X11 display server to allow the container to access the host machine's display for GUI functionality.

### 4. Open the Project in Container

Make sure you have the `Dev Container` extension installed

Open the project in a Docker container using your IDE’s command palette. In Visual Studio Code:

1. Open the Command Palette (`Ctrl + Shift + P` or `Cmd + Shift + P`).
2. Select `Remote-Containers: Reopen in Container.`

## Troubleshooting 

If you encounter issues:

- Ensure Docker is correctly installed and running.
- Verify that your X11 display server is correctly configured (for GUI on Linux).
- Rebuild the Docker image if there are changes to the Dockerfile.
