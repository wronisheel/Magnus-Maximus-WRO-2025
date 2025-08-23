## INTRODUCTION

We are the members of team Magnus Maximus. We are a part of the Robotronics Club of Ahmedabad dedicated to supporting and nurturing the growth of future innovators and leaders. We have a well-equipped engineering room supported by experienced teachers and mentors. With strong focus on STEM education, allowing us to push the boundaries of robotic innovation. Together having different skillsets, we have been able to put together this robot for the WRO competition.

## ABOUT THE TEAM

[Devansh Harivallabhdas - Grade 10] Devansh is a skilled coder with a deep passion for technology. He also excels as a hardware designer, bringing ideas to life with precision. His strength lies in combining coding and electronics seamlessly. Devansh loves solving real-world problems through innovation. He is driven by curiosity, creativity, and a vision to build the future.

[Dhvanil Shah - Grade 11] Dhvanil is the hardware and Creo expert, responsible for designing and executing the chassis. With prior involvement and victories in STEM competitions he ensures that the robot is efficient and easy to use. He brings lively spirit, bringing the machine to life.

[Nisheel Patel - Grade 10] Nisheel is a passionate coder who writes clean and efficient code. He also manages GitHub, ensuring smooth project collaboration. By handling version control, he keeps work organized and up to date. Alongside this, he actively supports the team in debugging and problem-solving. His coding and management skills make every project more effective.

## ENGINEERING PROCESSES
## =============
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
