## INTRODUCTION

We are the members of team Magnus Maximus. We are a part of the Robotronics Club of Ahmedabad dedicated to supporting and nurturing the growth of future innovators and leaders. We have a well-equipped engineering room supported by experienced teachers and mentors. With strong focus on STEM education, allowing us to push the boundaries of robotic innovation. Together having different skillsets, we have been able to put together this robot for the WRO competition.

## ABOUT THE TEAM

[Devansh Harivallabhdas - Grade 10] Devansh is a skilled coder with a deep passion for technology. He also excels as a hardware designer, bringing ideas to life with precision. His strength lies in combining coding and electronics seamlessly. Devansh loves solving real-world problems through innovation. He is driven by curiosity, creativity, and a vision to build the future.

[Dhvanil Shah - Grade 11] Dhvanil is the hardware and Creo expert, responsible for designing and executing the chassis. With prior involvement and victories in STEM competitions he ensures that the robot is efficient and easy to use. He brings lively spirit, bringing the machine to life.

[Nisheel Patel - Grade 10] Nisheel is a passionate coder who writes clean and efficient code. He also manages GitHub, ensuring smooth project collaboration. By handling version control, he keeps work organized and up to date. Alongside this, he actively supports the team in debugging and problem-solving. His coding and management skills make every project more effective.

ENGINEERING PROCESSES
====
## PROCESSES AND ELECTRICALS

In this project, the Raspberry Pi 4 functions as the central processor, responsible for handling both sensing inputs and motor control while also carrying out advanced image-processing tasks. Unlike systems that divide responsibilities between multiple controllers, here the Raspberry Pi alone manages both low-level and high-level operations, making the architecture compact and efficient.

For the open round, the robot makes use of VL53L1X Time-of-Flight (ToF) sensors, mounted on the front, left, and right sides of the chassis. These sensors work by emitting laser pulses and calculating the time taken for the reflections to return, providing accurate distance measurements. The Raspberry Pi continuously monitors this data to detect objects in its path in real time. Based on proximity readings, the robot decides whether to move forward, steer left, or turn right to avoid collisions. Each sensor is assigned a separate I2C address to ensure smooth, conflict-free communication, enabling the system to track multiple directions simultaneously. This precise and reliable obstacle awareness allows the robot to navigate open spaces effectively.

The robot also takes on the Open Challenge round, where it must navigate an arena bounded primarily by walls. In this scenario, the VL53L1X sensors play a crucial role by constantly scanning the distances between the robot and the walls around it. The sensors provide steady feedback that allows the Raspberry Pi to calculate the robot’s relative position within the arena and make decisions to avoid drifting too close to the walls or colliding with them. By using these continuous distance readings, the robot is able to maintain smooth motion while staying safely within the boundaries, demonstrating controlled navigation in constrained environments.

In the obstacle round, the system shifts its reliance toward vision through the PiCamera3, mounted at the front of the chassis. The camera captures live video of the environment, which is processed on the Raspberry Pi using computer vision libraries such as OpenCV. This allows the robot to recognize objects, detect obstacles, follow paths, or interpret lines drawn on the ground. Unlike the open round where laser sensors dominate, here visual input enables the robot to make more intelligent navigation decisions, adapting dynamically to unpredictable or complex challenges. By integrating both sensor data and visual cues, the robot demonstrates the ability to respond to its environment in a flexible and autonomous way.

The motion control system is managed through a custom PCB, which integrates the motor driver circuitry. The Raspberry Pi generates PWM (Pulse Width Modulation) signals to regulate the speed and direction of the motors, ensuring smooth forward motion, precise reversing, and accurate turns. The PCB not only simplifies wiring but also improves reliability by organizing signal flow and power distribution.

Power is supplied by a 2200mAh 3.7V lithium-ion battery, which provides consistent energy to the Raspberry Pi, sensors, and motors. A voltage regulator integrated into the PCB ensures stable output to each component, preventing fluctuations and maintaining dependable operation throughout the runs. The battery capacity supports extended performance without frequent recharging, which is vital for competition rounds.

All components—sensors, PiCamera3, PCB, battery, and motors—are securely mounted onto the robot’s chassis. The design balances weight and functionality, ensuring stability even when navigating tight spaces or avoiding obstacles. The forward-mounted PiCamera3 provides an unobstructed view of the environment, while the strategically placed VL53L1X sensors maximize coverage of the robot’s immediate surroundings.

By combining the laser-based precision of the VL53L1X sensors for the open and wall-bounded challenge rounds with the visual intelligence of the PiCamera3 for the obstacle round, the robot achieves a strong balance of accuracy and adaptability. It not only detects objects directly in its path but also maintains spatial awareness in wall-enclosed environments and intelligently navigates more complex obstacle courses.

This integration of sensors, vision, and motor control results in a robot that is both robust and versatile. It can reliably detect and avoid objects, maintain safe navigation between walls, and adapt dynamically to visually complex challenges. Altogether, the system demonstrates smooth, autonomous operation across varied environments, making it highly effective in meeting the demands of all competition rounds.

## ELECTRICAL COMPONENTS

Raspberry Pi 4b

Vl53l1x(Tof)Sensor

EV3 Chassis

3.7V 2200 mah battery

Pi camera 3

THE DETAILS

## EV3 Chassis

The EV3 chassis is a modular and sturdy framework designed primarily for use in robotics projects. It is part of the LEGO Mindstorms series, which allows for versatile construction and easy reconfiguration. The chassis can be fitted with multiple components, such as motors and sensors, to suit specific tasks. Its lightweight yet robust design makes it an ideal base for mobile robots, providing both stability and flexibility. The ability to easily attach or detach parts allows for quick modifications, which is particularly useful during testing and development phases.

## VL53L1X Distance Sensors

The VL53L1X is a time-of-flight laser-ranging sensor that measures the distance between the sensor and an object. Unlike traditional infrared sensors, the VL53L1X uses laser pulses to measure distances with high accuracy over a long range, even in low-light conditions. This sensor is crucial for wall-following, as it can sense distances up to 4 meters. It provides reliable distance data to the robot’s controller, allowing for smooth navigation and precise adjustments in tight spaces. Additionally, its compact size makes it easy to integrate into small or complex designs without taking up much space.

## Raspberry Pi 4

The Raspberry Pi 4 is a powerful and versatile single-board computer (SBC) that serves as the brain of our robot. It is equipped with a quad-core ARM Cortex-A72 processor, 4 GB of RAM, and multiple USB and GPIO ports, making it capable of handling complex tasks such as image processing, sensor fusion, and motor control. In your robot, the Raspberry Pi 4 processes the data from the VL53L1X sensors and the Raspberry Pi Camera V2, making real-time decisions about movement and obstacle avoidance. Its ability to run full-fledged operating systems like Raspberry Pi OS allows for easy programming and integration of various libraries for computer vision and robotics applications.

## Raspberry Pi Camera V3

The Raspberry Pi Camera V3 is a high-resolution camera module that provides the robot with the ability to see and interpret its surroundings. Featuring an 8-megapixel sensor, it captures high-quality images that the Raspberry Pi can use for color detection and object recognition. In our setup, the camera is crucial for detecting red and green obstacles, which guide the robot’s turns. The camera supports video capture at 1080p, allowing for real-time image processing and analysis. Its compact design ensures that it can be easily mounted on the robot without adding significant weight or bulk.

## 3.7V BATTERY

The robot is powered by a 3.7V 2200mAh lithium-ion battery, which serves as its primary energy source. Its compact size and high energy density make it ideal for lightweight robotic applications, while the 2200mAh capacity allows the robot to operate for extended periods without frequent recharging. The battery provides a consistent current sufficient for driving the motors, powering the sensors, and supporting image-processing tasks with the PiCamera3. Being rechargeable, it offers an economical and sustainable solution for repeated use. Its reliable performance ensures smooth and uninterrupted operation of the robot, allowing it to navigate, detect objects, and complete tasks efficiently in both open and obstacle rounds.

## SERVO MOTOR

The robot uses a servo motor to achieve precise angular movement for various tasks, such as adjusting sensors or steering mechanisms. Unlike standard DC motors, servo motors are equipped with a built-in feedback system that allows them to reach and hold specific positions accurately. They are controlled using PWM (Pulse Width Modulation) signals, where the duration of the pulse determines the angle of rotation. This precise control makes servo motors ideal for applications requiring reliable positioning and smooth motion. They are compact, lightweight, and consume low power, which makes them suitable for mobile robots. The torque generated by the servo is sufficient to move attached components without overloading the motor. Servos also respond quickly to control signals, allowing for real-time adjustments during navigation or object detection tasks. Their durability and consistent performance make them a key component in robotics projects. Overall, the servo motor provides efficient, precise, and controllable movement, enhancing the robot’s overall functionality and responsiveness.

MOBILITY MANAGEMENT:
====
Due to the unreliability of our initial 3D-printed design, we transitioned to a more robust and customizable approach by utilizing Lego components. The Lego Technic chassis provided an ideal foundation due to its flexibility and durability, allowing us to fine-tune our robot’s design based on evolving requirements.

## Core Design
The core of the robot’s movement is driven by an EV3 motor connected to a system of gears and a differential. This setup ensures smooth and controlled turns, crucial for maintaining stability during sharp maneuvers. The differential plays a key role in preventing tire slipping when the robot navigates tight turns, allowing for precise and consistent movements. We implemented a two-gear system to prioritize speed over torque, as the robot is required to traverse at least fourteen meters within a short period. The gearing ratio was chosen specifically to enhance speed without compromising stability, based on the demands of the competition.

## Sensor Placement and Functionality
The robot is equipped with two VL53L1X laser sensors, placed at the front of the chassis to handle the wall-following aspect of the task. This strategic placement allows the robot to make sharp turns as soon as the sensors detect a wall, ensuring quick and efficient navigation. Additionally, a third VL53L1X sensor is mounted at the front to measure the distance from the wall directly ahead, further improving precision when navigating along straight paths. The Raspberry Pi Camera V2 adds another layer of sensing by detecting obstacles marked in red or green, enhancing the robot's ability to identify and avoid objects effectively.

## Steering Mechanism
The steering mechanism is controlled by a servo motor, which allows for real-time adjustments to the robot’s direction. This motor is connected to the front axle, enabling smooth and responsive steering. The servo motor’s accuracy ensures that the robot maintains its intended course, especially during sharp turns or when avoiding obstacles.

## Power Supply
The entire system is powered by a 7.4V battery, providing sufficient energy to sustain the robot’s operations for approximately 60 minutes. This battery life ensures that the robot can complete multiple rounds of the challenge without needing frequent recharging, offering reliable performance throughout the competition.

## Assembly Instructions:
Chassis Construction: Build any Lego Technic chassis as per your choice. Ensure that the beams are aligned to offer a sturdy base for mounting motors and sensors.

## Motor Mounting: 
Attach the EV3 motor to the chassis (You will have to modify the cable).

## Gear Assembly: 
Connect the two gears to the EV3 motor to achieve the 1:1 speed-to-torque ratio. Attach the differential to the front axle for smoother turning.

## Sensor Installation: 
Mount the two VL53L1X sensors at the front of the chassis, ensuring they are angled appropriately for accurate wall detection. The third sensor should be placed facing forward to monitor the distance to walls.

## Camera and Servo Installation: 
Attach the Raspberry Pi Camera V2 to the front of the robot for obstacle detection. Install the servo motor to control the front axle for steering..

POWER AND SENSE MANAGEMENT
====
Our robot is powered by a 7.2V battery, which is selected to ensure stable and long-lasting performance throughout the competition. This power source is critical to running the EV3 motor, the servo motor for steering, and the three VL53L1X laser sensors. The battery can power the robot for more than sufficient operational time for multiple challenge rounds without requiring recharging.

## Sensor Management
The robot uses three VL53L1X laser sensors to accurately detect and measure distances during wall-following. Two of these sensors are mounted on the sides of the chassis to track distances to the nearest walls, while the third is placed at the front to monitor the distance from the wall directly ahead. These sensors allow the robot to maintain a precise distance from the inner, outer, and front walls, which is crucial for ensuring smooth and consistent navigation.

## Initial Readings and Wall Following
At the beginning of each round, the side-mounted VL53L1X sensors take initial distance measurements from the surrounding walls. These values guide the robot's wall-following behavior. As the robot moves, it continuously compares the current sensor readings with the initial measurements. When the robot detects a significant difference in these readings (i.e., the inner wall has ended), it triggers a turn in the direction of the sensor with the largest discrepancy.

## Final Stage and Path Completion
As the robot continues following the inner wall, it makes turns based on PiCam3 visual feedback, detecting the edges and orientation of the path in real time. Whenever the camera identifies the end of a wall, the robot executes the appropriate turn to remain aligned with the course. If a wall is directly sensed ahead, the robot immediately stops, makes the necessary turn, and then resumes navigation to avoid collision.

In addition to wall-following, the PiCam3 is also responsible for object detection in the obstacle round. When a red or green object is detected in the robot’s path, the system measures its distance and decides the direction of the turn accordingly. Based on this distance-aware decision-making, the robot either steers left or right to avoid the object while maintaining its overall course.

The robot relies completely on the PiCam3 for detection and decision-making, the robot combines wall sensing, object recognition, and distance-based responses to complete the obstacle round accurately and efficiently.

## Wiring and Sensor Integration
The wiring and integration of components in the robot are carefully designed to ensure stable power delivery and reliable data communication. The VL53L1X sensors are connected through the I2C interface to the Raspberry Pi 4, allowing accurate distance readings to be captured in real time without communication conflicts. The servo motor and EV3 motor, mounted on the EV3 chassis, are powered directly by the 3.7V 2200mAh lithium-ion battery through the custom PCB, which manages connections and distributes power efficiently. The PiCam3, also linked to the Raspberry Pi 4, receives its supply through the PCB, enabling continuous video capture for vision-based navigation. All wiring paths are routed systematically on the chassis to prevent interference between power and signal lines, ensuring smooth communication between components. This integration of sensors, motors, and the camera through the PCB provides a robust framework that allows the robot to operate reliably and maintain precise control, accurate sensing, and stable performance across all rounds.

While we believe that the task could be accomplished effectively using only two sensors, incorporating a third sensor offers several advantages. Not only does it make the overall process more efficient, but it also simplifies navigation, allowing the robot to react more quickly to environmental changes. By using three sensors, the robot can save valuable time during the task, resulting in smoother and more precise performance. These sensors are carefully positioned to ensure they can accurately detect nearby walls without interfering with the robot’s mobility. This strategic placement strikes a balance between optimal functionality and preventing any disruptions during movement

## Obstacle Management

In designing our robot’s obstacle management system, we adopted a multi-sensor strategy that ensures smooth navigation while maintaining efficiency and accuracy. The robot employs both the Raspberry Pi Camera 3 and VL53L1X laser distance sensors, with each serving a distinct purpose depending on the competition round.

During the Open Challenge round, the robot relies on VL53L1X sensors for precise distance measurement and wall-following, ensuring it can navigate turns and corridors with accuracy. Meanwhile, in the Obstacle round, only the PiCamera3 is used. Here, the camera enables color-based object detection (such as identifying red and green objects) and distance estimation, allowing the robot to intelligently decide whether to stop, turn, or change direction based on the detected obstacle.

By integrating these specialized approaches, the system demonstrates effective sensor fusion, combining distance sensing in one phase with vision-based detection in another. This ensures the robot can avoid collisions, detect colors, and stay aligned with the course, making obstacle management both robust and adaptable to different challenges.
