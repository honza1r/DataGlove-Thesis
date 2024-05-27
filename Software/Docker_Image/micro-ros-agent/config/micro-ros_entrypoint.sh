#!/bin/bash
AGENT_PID=""

. "/opt/ros/$ROS_DISTRO/setup.sh"
. "/uros_ws/install/local_setup.sh"

CONFIG_FILE="/shared-data/micro_ros_config.json"

# Default configuration content
DEFAULT_CONFIG='{
    "agent": "default",
    "configurations": {
        "default": {
            "transports": []
        }
    }
}'

# Check if the micro_ros_config.json file exists
if [ ! -f "$CONFIG_FILE" ]; then
    echo "micro_ros_config.json not found. Creating new with default configuration..."
    echo $DEFAULT_CONFIG > $CONFIG_FILE
fi

check_and_kill_orphaned_agent() {
    echo "Checking for orphaned process on port $PORT..."
    local orphaned_pid=$(lsof -ti udp:$PORT)
    if [ ! -z "$orphaned_pid" ]; then
        echo "Found orphaned micro-ROS agent process ($orphaned_pid) on port $PORT. Stopping it..."
        kill -9 $orphaned_pid || true
        sleep 1  # Give it a moment to release the port
    else
        echo "No orphaned process found on port $PORT."
    fi
}

# Function to start the micro-ROS agent with the configuration
start_agent() {
    # Default configurations
    PORT="50000"
    TRANSPORT="udp4"
    STARTED="false"
    echo "Starting Micro-ROS agent"
    # Read configuration from the JSON file if it exists
    if [ -f "$CONFIG_FILE" ]; then
        PORT=$(jq '.port' $CONFIG_FILE | tr -d '"')
        TRANSPORT=$(jq -r '.transport' $CONFIG_FILE)
        STARTED=$(jq -r '.started' $CONFIG_FILE)
        echo "Read port: $PORT, transport: $TRANSPORT, started: $STARTED"
    fi

    # Check if the agent should be started
    if [ "$STARTED" = "true" ]; then
        # Start the micro-ROS agent with the read configurations
        ros2 run micro_ros_agent micro_ros_agent $TRANSPORT --port $PORT &
        AGENT_PID=$!
        echo "Micro-ROS agent started on port $PORT and transport $TRANSPORT with PID $AGENT_PID "
    else
        echo "Micro-ROS agent not started due to configuration."
    fi
}

stop_agent() {
    # Use lsof to dynamically find the PID of the process using the port
    local agent_pid=$(sudo lsof -ti udp:$PORT)

    if [ ! -z "$agent_pid" ]; then
        echo "Attempting to stop micro-ROS agent with PID: $agent_pid"
        
        # Attempt to gracefully terminate the process
        sudo kill -SIGTERM "$agent_pid" > /dev/null 2>&1
        sleep 5  # Give it time to terminate gracefully

        # Force terminate if it's still running
        if sudo kill -0 "$agent_pid" > /dev/null 2>&1; then
            echo "micro-ROS agent did not terminate gracefully, forcing stop..."
            sudo kill -SIGKILL "$agent_pid" > /dev/null 2>&1
        fi
        
        echo "micro-ROS agent stopped."
    else
        echo "No micro-ROS agent found running on port $PORT."
    fi

    sleep 2  # Additional safety sleep

    # Check again to ensure the port is free
    if sudo lsof -ti udp:$PORT > /dev/null; then
        echo "Port $PORT is still in use. Please check for lingering processes."
    else
        echo "Port $PORT is free. The micro-ROS agent has stopped correctly."
    fi
}


# Invoke this function before starting a new agent
check_and_kill_orphaned_agent

# Initial check to start or not start the agent based on the configuration
start_agent

# Monitor the configuration file for changes
while inotifywait -e close_write $CONFIG_FILE; do
    echo "Configuration change detected, restarting..."
    # Always stop the agent if it's running
    stop_agent
    # Then decide to start it again based on the updated configuration
    start_agent
done

