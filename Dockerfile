FROM ros:humble-ros-base

WORKDIR /app

# Install system utilities, python3-pip, and nav2-simple-commander
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    python3-pip \
    ros-humble-nav2-simple-commander \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements and install them using pip3
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy the entire codebase (excluding folders ignored in .dockerignore)
COPY . .

# Set the entrypoint to run via the ROS 2 setup environment wrapper
ENTRYPOINT ["/ros_entrypoint.sh", "python3", "main.py"]

# Default command if none is provided
CMD ["--prompt", "Patrol the warehouse loop once at speed 1.2", "--robot", "turtlebot3", "--ros"]
