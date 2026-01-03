/**
 * TurtleBot3 Example Scenarios - Single Source of Truth
 * All examples with official ROS/ROS2/TurtleBot3/Nav2 documentation links
 */

export const EXAMPLES_DATA = [
  // ============= BASIC CATEGORY =============
  {
    id: 'teleop-keyboard',
    title: 'Teleoperation (Keyboard)',
    category: 'BASIC',
    difficulty: 'EASY',
    description: 'Control TurtleBot3 using keyboard commands via web interface',
    enabled: true,
    launchCommand: 'teleop',
    links: {
      docs: 'https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html',
      tutorial: 'https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation',
      code: 'https://github.com/ROBOTIS-GIT/turtlebot3'
    }
  },
  {
    id: 'turtlesim-demo',
    title: 'Turtlesim Web Demo',
    category: 'BASIC',
    difficulty: 'EASY',
    description: 'Interactive turtle control with real-time VNC visualization',
    enabled: true,
    launchCommand: 'turtlesim',
    links: {
      docs: 'https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html',
      tutorial: 'https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html',
      code: 'https://github.com/ros/ros_tutorials/tree/humble/turtlesim'
    }
  },
  {
    id: 'rviz-web-view',
    title: 'RViz Web Viewer (noVNC)',
    category: 'DEBUG',
    difficulty: 'EASY',
    description: 'Access RViz 3D visualization through browser-based noVNC',
    enabled: true,
    launchCommand: 'rviz',
    links: {
      docs: 'https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html',
      tutorial: 'https://github.com/ros2/rviz/blob/humble/README.md',
      code: 'https://github.com/ros2/rviz'
    }
  },
  {
    id: 'robot-state-publisher',
    title: 'Robot State & URDF Visualization',
    category: 'DEBUG',
    difficulty: 'EASY',
    description: 'Visualize robot model, joints, and TF frames in real-time',
    enabled: false,
    launchCommand: null,
    links: {
      docs: 'https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html',
      tutorial: 'https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html',
      code: 'https://github.com/ros/robot_state_publisher'
    }
  },

  // ============= MAPPING CATEGORY =============
  {
    id: 'slam-cartographer',
    title: 'SLAM with Cartographer',
    category: 'MAPPING',
    difficulty: 'MEDIUM',
    description: 'Real-time 2D/3D SLAM using Google Cartographer',
    enabled: true,
    launchCommand: 'slam_cartographer',
    links: {
      docs: 'https://google-cartographer-ros.readthedocs.io/en/latest/',
      tutorial: 'https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam',
      code: 'https://github.com/cartographer-project/cartographer_ros'
    }
  },
  {
    id: 'slam-toolbox',
    title: 'SLAM Toolbox Demo',
    category: 'MAPPING',
    difficulty: 'MEDIUM',
    description: 'Build 2D maps with slam_toolbox for Nav2 integration',
    enabled: true,
    launchCommand: 'slam_toolbox',
    links: {
      docs: 'https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md',
      tutorial: 'https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html',
      code: 'https://github.com/SteveMacenski/slam_toolbox'
    }
  },
  {
    id: 'laserscan-viz',
    title: 'LaserScan Visualization',
    category: 'PERCEPTION',
    difficulty: 'EASY',
    description: 'Real-time LiDAR scan data visualization and obstacle detection',
    enabled: false,
    launchCommand: null,
    links: {
      docs: 'https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html',
      tutorial: 'https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/',
      code: 'https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_node'
    }
  },

  // ============= NAVIGATION CATEGORY =============
  {
    id: 'navigation-basic',
    title: 'Navigation2 Basic',
    category: 'NAVIGATION',
    difficulty: 'MEDIUM',
    description: 'Autonomous navigation with obstacle avoidance using Nav2 stack',
    enabled: true,
    launchCommand: 'navigation',
    links: {
      docs: 'https://navigation.ros.org/',
      tutorial: 'https://navigation.ros.org/getting_started/index.html',
      code: 'https://github.com/ros-navigation/navigation2'
    }
  },
  {
    id: 'nav2-waypoints',
    title: 'Nav2 Waypoint Following',
    category: 'NAVIGATION',
    difficulty: 'MEDIUM',
    description: 'Follow predefined waypoints with Nav2 waypoint follower',
    enabled: false,
    launchCommand: null,
    links: {
      docs: 'https://navigation.ros.org/configuration/packages/configuring-waypoint-follower.html',
      tutorial: 'https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html',
      code: 'https://github.com/ros-navigation/navigation2/tree/humble/nav2_waypoint_follower'
    }
  },
  {
    id: 'amcl-localization',
    title: 'AMCL Localization',
    category: 'NAVIGATION',
    difficulty: 'MEDIUM',
    description: 'Adaptive Monte Carlo Localization on pre-built maps',
    enabled: false,
    launchCommand: null,
    links: {
      docs: 'https://navigation.ros.org/configuration/packages/configuring-amcl.html',
      tutorial: 'https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation',
      code: 'https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl'
    }
  },

  // ============= DEBUG/TOOLS CATEGORY =============
  {
    id: 'tf-frames-debug',
    title: 'TF/Frames Debugging',
    category: 'DEBUG',
    difficulty: 'EASY',
    description: 'Inspect transformation frames and coordinate relationships',
    enabled: false,
    launchCommand: null,
    links: {
      docs: 'https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html',
      tutorial: 'https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html',
      code: 'https://github.com/ros2/geometry2'
    }
  },
  {
    id: 'rosbridge-diagnostics',
    title: 'ROSBridge WebSocket Diagnostics',
    category: 'DEBUG',
    difficulty: 'EASY',
    description: 'Test ROS-Web communication via rosbridge_suite',
    enabled: false,
    launchCommand: null,
    links: {
      docs: 'https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md',
      tutorial: 'https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/README.md',
      code: 'https://github.com/RobotWebTools/rosbridge_suite'
    }
  },
  {
    id: 'gazebo-world-preview',
    title: 'Gazebo World Preview',
    category: 'BASIC',
    difficulty: 'EASY',
    description: 'Explore TurtleBot3 simulation worlds and environments',
    enabled: false,
    launchCommand: null,
    links: {
      docs: 'https://classic.gazebosim.org/tutorials',
      tutorial: 'https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation',
      code: 'https://github.com/ROBOTIS-GIT/turtlebot3_simulations'
    }
  }
]

/**
 * Get examples by category
 */
export const getExamplesByCategory = (category) => {
  if (category === 'ALL') return EXAMPLES_DATA
  return EXAMPLES_DATA.filter(ex => ex.category === category)
}

/**
 * Get examples by difficulty
 */
export const getExamplesByDifficulty = (difficulty) => {
  if (difficulty === 'ALL') return EXAMPLES_DATA
  return EXAMPLES_DATA.filter(ex => ex.difficulty === difficulty)
}

/**
 * Get enabled examples only
 */
export const getEnabledExamples = () => {
  return EXAMPLES_DATA.filter(ex => ex.enabled)
}

/**
 * Validate example object structure
 */
export const validateExample = (example) => {
  const required = ['id', 'title', 'category', 'difficulty', 'description', 'enabled', 'links']
  const linkFields = ['docs', 'tutorial', 'code']
  
  for (const field of required) {
    if (!(field in example)) {
      console.warn(`Example missing required field: ${field}`, example)
      return false
    }
  }
  
  if (example.links) {
    for (const linkField of linkFields) {
      if (!(linkField in example.links)) {
        console.warn(`Example links missing field: ${linkField}`, example)
        return false
      }
    }
  }
  
  return true
}

// Validate all examples on load (development only)
if (import.meta.env.DEV) {
  EXAMPLES_DATA.forEach(ex => {
    if (!validateExample(ex)) {
      console.error('Invalid example:', ex)
    }
  })
}

export default EXAMPLES_DATA

