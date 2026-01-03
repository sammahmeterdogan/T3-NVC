package com.samma.rcp.app.controller;

import com.samma.rcp.app.domain.model.ScenarioType;
import com.samma.rcp.base.controller.BaseController;
import com.samma.rcp.base.dto.ResponseDTO;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.*;

@RestController
@RequestMapping("/api/examples")
public class ExampleController extends BaseController {

    @GetMapping
    public ResponseEntity<ResponseDTO<List<Map<String, Object>>>> list() {
        // Enhanced example scenarios with official ROS/ROS2 links
        List<Map<String, Object>> items = new ArrayList<>();
        
        // BASIC Category
        items.add(createExample(
            "teleop-keyboard", "Teleoperation (Keyboard)", "BASIC", "EASY",
            "Control TurtleBot3 using keyboard commands via web interface",
            true,
            "https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html",
            "https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/#basic-operation",
            "https://github.com/ROBOTIS-GIT/turtlebot3"
        ));
        
        items.add(createExample(
            "turtlesim-demo", "Turtlesim Web Demo", "BASIC", "EASY",
            "Interactive turtle control with real-time VNC visualization",
            true,
            "https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html",
            "https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html",
            "https://github.com/ros/ros_tutorials/tree/humble/turtlesim"
        ));
        
        items.add(createExample(
            "rviz-web-view", "RViz Web Viewer (noVNC)", "DEBUG", "EASY",
            "Access RViz 3D visualization through browser-based noVNC",
            true,
            "https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html",
            "https://github.com/ros2/rviz/blob/humble/README.md",
            "https://github.com/ros2/rviz"
        ));
        
        items.add(createExample(
            "robot-state-publisher", "Robot State & URDF Visualization", "DEBUG", "EASY",
            "Visualize robot model, joints, and TF frames in real-time",
            false,
            "https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html",
            "https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html",
            "https://github.com/ros/robot_state_publisher"
        ));
        
        // MAPPING Category
        items.add(createExample(
            "slam-cartographer", "SLAM with Cartographer", "MAPPING", "MEDIUM",
            "Real-time 2D/3D SLAM using Google Cartographer",
            true,
            "https://google-cartographer-ros.readthedocs.io/en/latest/",
            "https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#slam",
            "https://github.com/cartographer-project/cartographer_ros"
        ));
        
        items.add(createExample(
            "slam-toolbox", "SLAM Toolbox Demo", "MAPPING", "MEDIUM",
            "Build 2D maps with slam_toolbox for Nav2 integration",
            true,
            "https://github.com/SteveMacenski/slam_toolbox/blob/humble/README.md",
            "https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html",
            "https://github.com/SteveMacenski/slam_toolbox"
        ));
        
        items.add(createExample(
            "laserscan-viz", "LaserScan Visualization", "PERCEPTION", "EASY",
            "Real-time LiDAR scan data visualization and obstacle detection",
            false,
            "https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html",
            "https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_lds_01/",
            "https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_node"
        ));
        
        // NAVIGATION Category
        items.add(createExample(
            "navigation-basic", "Navigation2 Basic", "NAVIGATION", "MEDIUM",
            "Autonomous navigation with obstacle avoidance using Nav2 stack",
            true,
            "https://navigation.ros.org/",
            "https://navigation.ros.org/getting_started/index.html",
            "https://github.com/ros-navigation/navigation2"
        ));
        
        items.add(createExample(
            "nav2-waypoints", "Nav2 Waypoint Following", "NAVIGATION", "MEDIUM",
            "Follow predefined waypoints with Nav2 waypoint follower",
            false,
            "https://navigation.ros.org/configuration/packages/configuring-waypoint-follower.html",
            "https://navigation.ros.org/tutorials/docs/navigation2_with_gps.html",
            "https://github.com/ros-navigation/navigation2/tree/humble/nav2_waypoint_follower"
        ));
        
        items.add(createExample(
            "amcl-localization", "AMCL Localization", "NAVIGATION", "MEDIUM",
            "Adaptive Monte Carlo Localization on pre-built maps",
            false,
            "https://navigation.ros.org/configuration/packages/configuring-amcl.html",
            "https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation",
            "https://github.com/ros-navigation/navigation2/tree/humble/nav2_amcl"
        ));
        
        // DEBUG/TOOLS Category
        items.add(createExample(
            "tf-frames-debug", "TF/Frames Debugging", "DEBUG", "EASY",
            "Inspect transformation frames and coordinate relationships",
            false,
            "https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html",
            "https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html",
            "https://github.com/ros2/geometry2"
        ));
        
        items.add(createExample(
            "rosbridge-diagnostics", "ROSBridge WebSocket Diagnostics", "DEBUG", "EASY",
            "Test ROS-Web communication via rosbridge_suite",
            false,
            "https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md",
            "https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/README.md",
            "https://github.com/RobotWebTools/rosbridge_suite"
        ));
        
        items.add(createExample(
            "gazebo-world-preview", "Gazebo World Preview", "BASIC", "EASY",
            "Explore TurtleBot3 simulation worlds and environments",
            false,
            "https://classic.gazebosim.org/tutorials",
            "https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#simulation",
            "https://github.com/ROBOTIS-GIT/turtlebot3_simulations"
        ));
        
        return success(items);
    }
    
    private Map<String, Object> createExample(
            String id, String title, String category, String difficulty,
            String description, boolean enabled,
            String docsUrl, String tutorialUrl, String codeUrl) {
        Map<String, Object> example = new HashMap<>();
        example.put("id", id);
        example.put("title", title);
        example.put("category", category);
        example.put("difficulty", difficulty);
        example.put("description", description);
        example.put("enabled", enabled);
        
        Map<String, String> links = new HashMap<>();
        links.put("docs", docsUrl);
        links.put("tutorial", tutorialUrl);
        links.put("code", codeUrl);
        example.put("links", links);
        
        return example;
    }

    @GetMapping("/types")
    public ResponseEntity<ResponseDTO<ScenarioType[]>> types() {
        return success(ScenarioType.values());
    }
    
    @PostMapping("/{id}/launch")
    public ResponseEntity<ResponseDTO<String>> launch(@PathVariable String id) {
        // Placeholder for launch logic - to be implemented with actual ROS orchestration
        return success("Example '" + id + "' launch initiated");
    }
}
