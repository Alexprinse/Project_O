# 🤖 Omokai Robotics: Natural Language Mission Pipeline

A production-quality, modular robotics pipeline that translates natural language instructions into validated, deterministic JSON mission plans and executes them on physical or simulated robots.

This project implements the core task of the Omokai Robotics Engineering Task, demonstrating full support for multiple ground robot platforms in simulation: **TurtleBot3** (using ROS 2 Nav2) and the **e-Yantra Krishi Cobot** (using direct odometry/LiDAR velocity controls).

---

## 🏗️ System Architecture

The pipeline processes user commands through a sequential, decoupled chain:

```mermaid
flowchart TD
    %% Custom Styling Definitions
    classDef inputNode fill:#2d1a00,stroke:#ff9e00,stroke-width:2px,color:#ffe0b3;
    classDef configNode fill:#0a2000,stroke:#38b000,stroke-width:2px,color:#e1f9d0;
    classDef parserNode fill:#1f0033,stroke:#9d4edd,stroke-width:2px,color:#e8dbfd;
    classDef validatorNode fill:#2a0010,stroke:#ff0054,stroke-width:2px,color:#ffe0eb;
    classDef executorNode fill:#001d2a,stroke:#00d2ff,stroke-width:2px,color:#e0f7fc;

    %% Elements
    Prompt["🗣️ User Prompt (Plain Text)<br/>e.g., 'Patrol the warehouse twice'"]:::inputNode

    subgraph ConfigLayer ["⚙️ Configuration Layer"]
        Waypoints["📍 config/waypoints_*.yaml<br/>(Allowed route coordinates)"]:::configNode
        Settings["🛡️ config/settings.yaml<br/>(Speed & Loop constraints)"]:::configNode
    end

    subgraph TranslationLayer ["🧠 Translation Layer (Gemini LLM)"]
        Parser["🤖 Mission Parser (llm/client.py)<br/>Compiles System Prompt"]:::parserNode
        LLM["✨ gemini-2.5-flash API<br/>Outputs structured JSON"]:::parserNode
    end

    subgraph GuardLayer ["🛡️ Safety Validation (Interceptor Guard)"]
        Validator["🛡️ Mission Validator (validator/validator.py)<br/>Pydantic schema check & boundaries"]:::validatorNode
        Schema["📋 Pydantic Schema (validator/schema.py)<br/>MissionPlan Type Constraints"]:::validatorNode
    end

    subgraph ControlLayer ["🚦 Deterministic Control Loop"]
        Executor["🚦 Mission Executor (executor/executor.py)<br/>Translates coordinates to path steps"]:::executorNode
        Robot["🚀 Robot Controller (robot/)<br/>Direct odom / Nav2 driving"]:::executorNode
    end

    %% Flow Connections
    Prompt -->|1. CLI Prompt Input| Parser
    Waypoints -.->|2. Inject route names| Parser
    Parser -->|3. Compile dynamic prompt context| LLM
    LLM -->|4. Strict JSON output| Parser
    Parser -->|5. Unvalidated JSON| Validator
    Waypoints -.->|6. Verify route exists| Validator
    Settings -.->|7. Verify safety bounds| Validator
    Validator -->|8. Validate models| Schema
    Validator -->|9. Safe MissionPlan Object| Executor
    Executor -->|10. Sequence coordinate poses| Robot
```

### Architectural Breakdown

1. **Dynamic Context Ingestion**:
   At initialization, [main.py](file:///home/alex/Documents/Omokai_Project/main.py) reads the selected robot parameter (`--robot`) and loads the corresponding waypoint file (e.g. `config/waypoints_ebot.yaml`). The allowed route keys are extracted and fed to `MissionParser`, which dynamically injects them into the system prompt. This forces the LLM to choose matching keys (e.g., mapping "warehouse" to `warehouse_loop`).
2. **Translation & Structured Generation**:
   The plain-text user command is sent to the Gemini API (`gemini-2.5-flash`) along with the dynamically compiled prompt context, forcing the LLM to output a strict JSON structure matching our schema.
3. **Active Validation Shielding**:
   Before execution, `MissionValidator` catches the raw output. It runs structural validation using Pydantic, and checks logical limits defined in `config/settings.yaml` (such as speed boundaries and loop count). If any violations occur, execution is blocked immediately.
4. **Deterministic Motor Loop**:
   Once validation succeeds, the safe `MissionPlan` is parsed by `MissionExecutor`. It coordinates step-by-step route poses and navigates the robot via the selected controller interface. Since execution is strictly separated from AI reasoning, running the same output always guarantees the exact same path.

---

## 🛠️ Step-by-Step Setup & Execution Guide

Follow these steps sequentially to set up and run the simulation and controller pipeline.

### Step 1: Clone the Repository
Clone the codebase and navigate to the project directory:
```bash
git clone https://github.com/Alexprinse/Omokai_Project.git
cd Omokai_Project
```

### Step 2: Run the Simulation Host Setup Script
Execute the automated installer to set up ROS 2 Humble, Gazebo, Nav2, all Python dependencies, and build the workspace on Ubuntu 22.04 LTS:
```bash
chmod +x setup_simulation.sh
./setup_simulation.sh
```
*(This script also automatically sets up the Python virtual environment `.venv` with access to ROS system packages).*

### Step 3: Build the Simulation Workspace (Manual Re-compilation)
The setup script compiles the workspace automatically. However, if you modify code in `eyrc_ws` or want to rebuild manually, run:
```bash
cd eyrc_ws
colcon build
cd ..
```

### Step 4: Run the Simulation & Pipeline (Native Host Setup)

To execute the pipeline natively on your host machine:

#### 1. Start the Simulation (Terminals 1 & 2)
*   **For e-Yantra Krishi Cobot (ebot):**
    *   **Terminal 1 (Launch World):**
        ```bash
        source eyrc_ws/install/setup.bash
        ros2 launch eyantra_warehouse task2.launch.py
        ```
    *   **Terminal 2 (Spawn Robot):**
        ```bash
        source eyrc_ws/install/setup.bash
        ros2 launch ebot_description spawn_ebot.launch.py
        ```
*   **For TurtleBot3 (AWS Warehouse):**
    *   **Terminal 1 (Launch Simulator with Map):**
        ```bash
        # 1. Setup environment and software rendering
        export LIBGL_ALWAYS_SOFTWARE=1
        export TURTLEBOT3_MODEL=waffle_pi
        source /usr/share/gazebo/setup.sh
        source /opt/ros/humble/setup.bash

        # 2. Launch the AWS warehouse world with the saved static map
        ros2 launch nav2_bringup tb3_simulation_launch.py \
          world:=/home/alex/Documents/Omokai_Project/config/worlds/warehouse.world \
          map:=/home/alex/Documents/Omokai_Project/config/maps/warehouse_map.yaml \
          slam:=False \
          headless:=False
        ```

#### 2. Run the Controller Pipeline (Terminal 2)
Activate the virtual environment, export your API key, and launch the pipeline:
```bash
source .venv/bin/activate
export GEMINI_API_KEY="your_gemini_api_key_here"
```

##### Example Prompts to Try:
*   **ebot Serpentine Sweep:**
    ```bash
    python3 main.py --prompt "Patrol the serpentine path at speed 0.4" --robot ebot --ros
    ```
*   **ebot Central Corridor:**
    ```bash
    python3 main.py --prompt "Drive through the central corridor at speed 0.8" --robot ebot --ros
    ```
*   **TurtleBot3 Delivery Route (with custom safety speed):**
    ```bash
    python3 main.py --prompt "patrol the delivery line for once at maximum speed of 2.5m/s" --ros --robot turtlebot3
    ```
*   **Challenge 2: SLAM Coordinate Navigation & Stuck Recovery:**
    ```bash
    # Command the robot to navigate in SLAM mode. If it hits planning failures, 
    # it recursively plans via intermediate poses (2.0m steps) to recover.
    python3 main.py --prompt "go to coordinates x=1.83, y=9.72 and dont return home" --ros --robot turtlebot3
    ```
*   **Challenge 3: Vision AI Target Detection & Follow (YOLOv8 & HSV):**
    First, spawn a target object in the corridor using our helper script:
    ```bash
    # Spawn a standing person at x=1.8, y=7.0 (locally cached, loads instantly offline)
    python3 scripts/swap_object.py --object person --x 1.8 --y 7.0
    ```
    Then run the tracking command:
    ```bash
    python3 main.py --prompt "find a person and follow it" --ros --robot turtlebot3
    ```
*   **Combined Sequential Navigate & Follow:**
    You can command the robot to drive to a location first, and then automatically start tracking:
    ```bash
    python3 main.py --prompt "go to x=1.8 and y=9.0 and find a person and follow" --ros --robot turtlebot3
    ```
    *(Open RViz, click "Add -> Image", and subscribe to `/camera/image_annotated` to view the live camera feed with green bounding boxes!).*

---

### Step 5: Run EVERYTHING inside Docker (with X11 Display Forwarding)

You can run the entire simulation, navigation stack, and python pipeline completely inside Docker. The 3D simulator window (Gazebo/RViz) will be rendered on your host display using X11 forwarding.

#### 1. Grant Docker access to your X Server (Run on your Host)
Before launching the containers, authorize local docker containers to connect to your host GUI window server:
```bash
xhost +local:docker
```

#### 2. Build and Start the Simulator (Terminal 1)
Build the unified container and start the Gazebo AWS warehouse world + Nav2 navigation server:
```bash
docker compose up simulation
```
*(The simulation will launch instantly, load the custom map, and show up on your host screen).*

#### 3. Run Commands Interactively (Terminal 2)
In a separate terminal, start the interactive controller shell:
```bash
# Export your API key on the host
export GEMINI_API_KEY="your_gemini_api_key_here"

# Start the controller container and open a bash shell
docker compose run controller
```

Once inside the container shell, you can issue natural-language prompts to command the robot:
```bash
python3 main.py --prompt "patrol the delivery line for once at maximum speed of 2.5m/s" --ros --robot turtlebot3
```

---

### Step 6: Run in Mock Mode (Zero-Dependency Offline Test)
If you do not have Gazebo or ROS 2 installed and want to run the pipeline offline:

*   **Using Docker (Fastest, requires no local Python packages):**
    ```bash
    docker build -t omokai-mission-pipeline .
    docker run -it --rm omokai-mission-pipeline --prompt "Patrol the warehouse loop twice" --robot turtlebot3 --mock-llm
    ```
*   **Using local Python:**
    ```bash
    source .venv/bin/activate
    python main.py --prompt "Patrol the warehouse loop twice" --robot turtlebot3 --mock-llm
    ```

## 🏆 Senior Challenges Implemented

This repository implements two of the three senior-level challenges:

### 1. Challenge 2: SLAM / Autonomous Navigation
* **Online Mapping & Localization**: The pipeline is designed to work in an unknown or partially known environment. If you launch the simulation in SLAM mode (with SLAM Toolbox mapping on-the-fly), the controller automatically detects `/slam_toolbox/get_state` and bypasses the static map AMCL localizer lifecycle checks, adapting to the dynamically growing map.
* **Arbitrary Coordinate Navigation**: Rather than navigating only to predefined routes, the user can command the robot to drive to arbitrary coordinates (e.g. `"go to coordinates x=1.5, y=-2.0"`). The LLM extracts the target coordinates directly, and the validator ensures the coordinates are within configured safety bounds (e.g. `x_min`/`x_max` limits in `config/settings.yaml`). The deterministic executor then commands Nav2 to plan a safe path to the custom pose.

### 2. Challenge 3: Vision AI Target Detection & Follow
* **Dual Vision Modalities**: 
  1. **HSV Color Segmentation**: Highly robust and fast tracking for distinctively colored objects (e.g., `"red"`, `"green"`, `"blue"`, `"yellow"`) in simulated worlds.
  2. **YOLOv8 deep learning network**: Integrates `ultralytics` YOLOv8 nano model (fully cached locally) to detect and follow complex target objects (like `person`, `car`, `bus`, `bottle`, etc.).
* **Dynamic Follow Control Loop**: Subscribes to `/camera/image_raw`, processes the frames, and runs target detection. It adjusts the robot's angular velocity using smooth proportional steering control (`max 0.4 rad/s`) to keep the target centered in the frame, and adjusts linear velocity to drive towards it, stopping automatically when it gets close.
* **Live Annotated RViz Feed**: Publishes annotated frames with green bounding boxes and labels directly to ROS 2 on the topic `/camera/image_annotated` for real-time visualization in RViz.
* **Operator Alerts**: Captures a snapshot of the camera frame with bounding boxes and labels when the target is first acquired and saves it to `detection.jpg` in the project root to send back to the human operator.
* **Fully Portable**: Converts the raw ROS Image bytes to OpenCV BGR matrices manually inside the script. This eliminates the dependency on the `cv_bridge` package, making the image processor work flawlessly inside custom python virtual environments on any host machine or container.

---

## ⚙️ Configuration Files
* **[config/settings.yaml](file:///home/alex/Documents/Omokai_Project/config/settings.yaml)**: Safety boundaries (min/max speeds, loop limits).
* **[config/waypoints_turtlebot3.yaml](file:///home/alex/Documents/Omokai_Project/config/waypoints_turtlebot3.yaml)**: Coordinates for the TurtleBot3 simulation.
* **[config/waypoints_ebot.yaml](file:///home/alex/Documents/Omokai_Project/config/waypoints_ebot.yaml)**: Serpentine and corridor coordinates for the ebot simulation.

---

## 📈 Real-World Scaling Story

To adapt this local mock and simulation architecture to a real-world, high-reliability deployment, we would implement:
1. **Dynamic Path Planning & Local SLAM**: Instead of using direct coordinate waypoints, the executor should send the waypoints to local trajectory planner nodes (e.g. ROS 2 Nav2 DWA or TEB local planners) that utilize active Costmaps. This allows the robot to dynamically steer around unexpected obstacles (such as humans or boxes) in real time while maintaining progress toward the global target.
2. **Deterministic Fail-safe Guardrails**:
   * **LLM Fallback**: If the API call times out or returns malformed JSON, the pipeline immediately falls back to a deterministic regex/keyword parser local module to ensure the robot can still execute safety-critical operations offline.
   * **Telemetry Limits**: The validator continuously monitors state topics. If a velocity exceeding safe bounds is published (due to an LLM hallucination or executor bug), a low-level hardware watchdog node intercepts the message and triggers an emergency stop.
3. **Multi-Agent Coordination (Central Fleet Dispatch)**: Scale the Pydantic JSON schema to accept squad-level actions (e.g. `formation: wedge`). A fleet manager node then allocates sub-waypoints to individual robots using algorithms like Hungarian assignment and orchestrates synchronized execution.

---

## 📚 Cited Sources

This project builds on and references the following open-source resources:
* **Pydantic v2** ([pydantic/pydantic](https://github.com/pydantic/pydantic)) - *License: MIT*. Used to define and validate the structural JSON mission schema.
* **Google GenAI SDK** ([google/generative-ai-python](https://github.com/google/generative-ai-python)) - *License: Apache-2.0*. Used to interface with the Gemini API.
* **ROS 2 Nav2 Simple Commander** ([ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)) - *License: Apache-2.0*. Used as reference for the programmatic interface (`BasicNavigator`) to Nav2.
* **AWS RoboMaker Small Warehouse World** ([aws-robotics/aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world)) - *License: Apache-2.0*. Used as the 3D Gazebo warehouse world layout.
* **SLAM Toolbox** ([SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox)) - *License: LGPL-2.1*. Used for 2D mapping and real-time loop-closure localization.
* **eYRC Krishi Cobot Serpentine Navigation** - *License: Custom/Educational*. The proportional control and LiDAR obstacle avoidance algorithms in `robot/ebot_controller.py` are adapted from my past work in the e-Yantra Robotics Competition 2025-26.
