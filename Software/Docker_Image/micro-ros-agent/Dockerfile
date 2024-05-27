FROM microros/micro-ros-agent:foxy

# Install inotify-tools for inotifywait and jq for JSON parsing
RUN apt-get update && \
    apt-get install -y inotify-tools jq lsof iputils-ping netcat-openbsd && \
    rm -rf /var/lib/apt/lists/* 

# Copy your entrypoint script into the container
COPY config/micro-ros_entrypoint.sh /

# Ensure the entrypoint script is executable
RUN chmod +x /micro-ros_entrypoint.sh

# Set the entrypoint script to run when the container starts
ENTRYPOINT ["/bin/sh", "/micro-ros_entrypoint.sh"]


