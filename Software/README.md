# Project Setup Instructions

This README will guide you through setting up the project using Docker and Docker Compose. Follow the steps below to ensure everything is configured correctly.

## Project Purpose

The aim for the software of this project is to provide an easily configurable environment to interface with data gloves running micro-ros middleware for communication. The system uses a Node.js back-end to process post requests in order to change the configuration of the micro-ros agent running inside its own container. While in this project a simple web-gui using nginx alpine is used to showcase this functionality the goal is that any other gui can be used and through post requests one can manage the micro-ros communication. At the same time the integration of ROS allows for the use of ROS-Bridge which allows the accessing of ROS nodes through WebSockets.

## Prerequisites

- Docker
- Docker Compose

## Installing Docker

1. **Docker Installation for Windows/Mac:**

   - Download and install Docker Desktop from the [official Docker website](https://www.docker.com/products/docker-desktop).
   - Follow the installation instructions for your operating system.

2. **Docker Installation for Linux:**

   - Follow the official Docker installation guide for your Linux distribution:
     - [Ubuntu](https://docs.docker.com/engine/install/ubuntu/)
     - [Debian](https://docs.docker.com/engine/install/debian/)
     - [CentOS](https://docs.docker.com/engine/install/centos/)
   - Install Docker Compose by following the [official installation guide](https://docs.docker.com/compose/install/).

## Project Structure

- `.vscode/`: Contains workspace settings for Visual Studio Code.
- `Docker_Image/`: Contains Docker-related files and configurations.
- `README.md`: Project documentation (this file).
- `Software Repo.code-workspace`: Workspace configuration for Visual Studio Code.

## Configuration

1. **Change Necessary IP Addresses:**

   - Navigate to the `Docker_Image` directory and open the `docker-compose.yml` file.
   - Replace any placeholder IP addresses with the appropriate IP addresses for your network setup.

   Example:
   ```yaml
   networks:
     network:
       driver: bridge
       ipam:
         config:
           - subnet: "192.168.1.0/24" 
    ```

   - Change the ipv4_address under the services section if necessary:

   Example:
   ```yaml
   services:
    ros-bridge:
        networks:
            network:
                ipv4_address: "192.168.1.100"
    ```

2. Review Dockerfile(s):

Open the Docker_Image directory and review the Dockerfile for each service.
Ensure there are no hardcoded paths or sensitive information.

## Running the Project

1. Open a Terminal:

    - Navigate to the Docker_Image directory where the docker-compose.yml file is located.

2. Build and Run the Docker Containers:

    - Execute the following command to build and start the containers:
    ```sh
        docker-compose up --build
    ```
    This command will build the Docker images and start the services defined in the docker-compose.yaml file.
    

3. Accessing the Services:

    - Once the services are up and running, you can access them using the defined ports in the docker-compose.yml file.
    - For example, if the web server is mapped to port 8080, you can access it at http://localhost:8080.

4. Stopping the Project

    - To stop the running Docker containers, use the following command:

    ```sh
    docker-compose down
    ```
    or simply keyboard interrupt with ctrl+c and the containers and services will shutdown gracefully

5. Additional Notes
    - Make sure to check the logs for any errors or issues during the build and startup process.
    - For any further customization or configuration, refer to the official documentation for Docker and Docker Compose.