INTRODUCTION

We are the members of team Magnus Maximus. We are a part of the Robotronics Club of Ahmedabad dedicated to supporting and nurturing the growth of future innovators and leaders. We have a well-equipped engineering room supported by experienced teachers and mentors. With strong focus on STEM education, allowing us to push the boundaries of robotic innovation. Together having different skillsets, we have been able to put together this robot for the WRO competition.

ABOUT THE TEAM

[Devansh Harivallabhdas - Grade 10] Devansh is a skilled coder with a deep passion for technology. He also excels as a hardware designer, bringing ideas to life with precision. His strength lies in combining coding and electronics seamlessly. Devansh loves solving real-world problems through innovation. He is driven by curiosity, creativity, and a vision to build the future.
 
[Dhvanil Shah - Grade 11] Dhvanil is the hardware and Creo expert, responsible for designing and executing the chassis. With prior involvement and victories in STEM competitions he ensures that the robot is efficient and easy to use. He brings lively spirit, bringing the machine to life.

[Nisheel Patel - Grade 10] Nisheel is a passionate coder who writes clean and efficient code. He also manages GitHub, ensuring smooth project collaboration. By handling version control, he keeps work organized and up to date. Alongside this, he actively supports the team in debugging and problem-solving. His coding and management skills make every project more effective.

ENGINEERING PROCESSES

PROCESSES AND ELECTRICALS

In this project, the Raspberry Pi 4 functions as the central processor, responsible for handling both sensing inputs and motor control while also carrying out advanced image-processing tasks. Unlike systems that divide responsibilities between multiple controllers, here the Raspberry Pi alone manages both low-level and high-level operations, making the architecture compact and efficient.

For the open round, the robot makes use of VL53L1X Time-of-Flight (ToF) sensors, mounted on the front, left, and right sides of the chassis. These sensors work by emitting laser pulses and calculating the time taken for the reflections to return, providing accurate distance measurements. The Raspberry Pi continuously monitors this data to detect objects in its path in real time. Based on proximity readings, the robot decides whether to move forward, steer left, or turn right to avoid collisions. Each sensor is assigned a separate I2C address to ensure smooth, conflict-free communication, enabling the system to track multiple directions simultaneously. This precise and reliable obstacle awareness allows the robot to navigate open spaces effectively.

The robot also takes on the Open Challenge round, where it must navigate an arena bounded primarily by walls. In this scenario, the VL53L1X sensors play a crucial role by constantly scanning the distances between the robot and the walls around it. The sensors provide steady feedback that allows the Raspberry Pi to calculate the robot’s relative position within the arena and make decisions to avoid drifting too close to the walls or colliding with them. By using these continuous distance readings, the robot is able to maintain smooth motion while staying safely within the boundaries, demonstrating controlled navigation in constrained environments.

In the obstacle round, the system shifts its reliance toward vision through the PiCamera3, mounted at the front of the chassis. The camera captures live video of the environment, which is processed on the Raspberry Pi using computer vision libraries such as OpenCV. This allows the robot to recognize objects, detect obstacles, follow paths, or interpret lines drawn on the ground. Unlike the open round where laser sensors dominate, here visual input enables the robot to make more intelligent navigation decisions, adapting dynamically to unpredictable or complex challenges. By integrating both sensor data and visual cues, the robot demonstrates the ability to respond to its environment in a flexible and autonomous way.

The motion control system is managed through a custom PCB, which integrates the motor driver circuitry. The Raspberry Pi generates PWM (Pulse Width Modulation) signals to regulate the speed and direction of the motors, ensuring smooth forward motion, precise reversing, and accurate turns. The PCB not only simplifies wiring but also improves reliability by organizing signal flow and power distribution.

Power is supplied by a 2200mAh 3.7V lithium-ion battery, which provides consistent energy to the Raspberry Pi, sensors, and motors. A voltage regulator integrated into the PCB ensures stable output to each component, preventing fluctuations and maintaining dependable operation throughout the runs. The battery capacity supports extended performance without frequent recharging, which is vital for competition rounds.

All components—sensors, PiCamera3, PCB, battery, and motors—are securely mounted onto the robot’s chassis. The design balances weight and functionality, ensuring stability even when navigating tight spaces or avoiding obstacles. The forward-mounted PiCamera3 provides an unobstructed view of the environment, while the strategically placed VL53L1X sensors maximize coverage of the robot’s immediate surroundings.

By combining the laser-based precision of the VL53L1X sensors for the open and wall-bounded challenge rounds with the visual intelligence of the PiCamera3 for the obstacle round, the robot achieves a strong balance of accuracy and adaptability. It not only detects objects directly in its path but also maintains spatial awareness in wall-enclosed environments and intelligently navigates more complex obstacle courses.

This integration of sensors, vision, and motor control results in a robot that is both robust and versatile. It can reliably detect and avoid objects, maintain safe navigation between walls, and adapt dynamically to visually complex challenges. Altogether, the system demonstrates smooth, autonomous operation across varied environments, making it highly effective in meeting the demands of all competition rounds.

 ELECTRICAL COMPONENTS

 Raspberry Pi 4b

 Vl53l1x(Tof)Sensor

 EV3 Chassis

 3.7V 2200 mah battery

 Pi camera 3

 
