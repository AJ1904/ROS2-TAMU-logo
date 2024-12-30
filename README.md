# Create TAMU logo using ROS2

This project involved creating a program that coordinates multiple turtle robots to draw specified patterns in parallel. The program utilizes ROS (Robot Operating System) and includes functionalities for spawning, controlling, and navigating multiple turtles.



https://github.com/user-attachments/assets/d1bcef6b-cb9d-4a9d-8ac2-2476c6a4bb47



The execution begins with a launch file that initializes the `LogoDrawer` node, taking the parameter `num_turtles`. The program then creates `num_turtles` instances of the `TurtleController` node, enabling simultaneous drawing by multiple turtles.

### Key Functionalities

1. **Turtle Movement and Drawing**:
   - Each `TurtleController` node manages its turtle's movement to draw assigned lines.
   - Logic for spawning, killing, and navigating turtles is implemented for synchronized control.

2. **Distributed Control**:
   - Individual publishers, subscribers, and service clients control each turtle independently.
   - Parallel execution of nodes allows for efficient drawing.

---

## Functions and Components

### Functions

- **`line_length`**:
  Calculates the Euclidean distance between two points (endpoints of a line segment).

- **`divide_lines_into_parts`**:
  Distributes lines among turtles for approximately equal workload.

### Components

- **`cmd_publisher`**:
  Publishes velocity commands to control the turtle's movement.

- **`pose_subscriber`**:
  Subscribes to the turtle's pose updates for position and orientation tracking.

- **`pen_client`**:
  Controls the turtle's pen state (up or down) via the `SetPen` service.

- **`spawn_turtle`**:
  Spawns a turtle at a specific position using the `Spawn` service.

- **`timer_callback`**:
  Periodically triggers movement logic to draw lines.

---

## Design Choices

1. **Parameterization**:
   - The number of turtles can be specified as a parameter, adding flexibility.

2. **Parallel Execution**:
   - Each turtle operates independently through its `TurtleController` instance.

3. **Modular Design**:
   - Tasks are divided into classes and functions for maintainability.

---

## Multi-Robot Handling

- **Individual Control**:
  Each turtle has its own publisher, subscriber, and service client.
- **Synchronized Execution**:
  Timer callbacks ensure periodic and coordinated actions across turtles.

---

## Movement Control

- **Timers**:
  Trigger periodic updates for each turtle's movement.
- **Pose Updates**:
  Real-time updates on position and orientation allow precise control.
- **Services**:
  Enable switching between drawing and non-drawing states.

---

## Specific Values

- **Timer Interval**: `0.1` seconds for `timer_callback`.
- **Distance Threshold**: `0.01` to determine when to stop moving.
- **Angle Threshold**: `0.001` for orientation adjustments.
- **Initial Position**: `(5.544445, 5.544445, initial_angle)`.

These values were fine-tuned through testing to achieve desired movement behavior.

---

## Observations

### Results
- **Consistency**: Turtle movements are inconsistent. The logo is drawn successfully in some runs, while in others, the turtles go out of bounds.
- **Challenges**: Significant effort was made to improve reliability and accuracy.

### Performance with Different Numbers of Turtles

1. **Two Turtles**:
   The drawing is relatively stable and accurate.

   <img width="399" alt="image" src="https://github.com/user-attachments/assets/984bf6f5-7346-47ca-a179-fac797b658d6" />


3. **Ten Turtles**:
   Increased complexity often leads to issues with synchronization and boundary constraints.
   
   <img width="399" alt="image" src="https://github.com/user-attachments/assets/105d87ac-6d60-4f0b-b644-ca1c0784aea7" />


---

## Conclusion

This project showcased the challenges and intricacies of coordinating multiple robots in a distributed system. Despite some inconsistencies, the implementation demonstrated the ability to achieve synchronized drawing with parallel execution.
