ros2 topic pub /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: base_link}, joint_names: [Joint1, Joint2, Joint3, Joint4, Joint5, Joint6], points: [{positions: [0, 0, 0, 0, 0, 0], velocities: [0, 0, 0, 0, 0, 0], accelerations: [], effort: [], time_from_start: {sec: 0, nanosec: 0}}, {positions: [1, 0, 0, 0, 0, 0], velocities: [0, 0, 0, 0, 0, 0], accelerations: [], effort: [], time_from_start: {sec: 1, nanosec: 0}}, {positions: [0, 0, 0, 0, 0, 0], velocities: [0, 0, 0, 0, 0, 0], accelerations: [], effort: [], time_from_start: {sec: 2, nanosec: 0}}]}"