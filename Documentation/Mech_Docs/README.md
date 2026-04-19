# Turtlebot Mechanical Documentation

## Turtlebot Components

### Bill of Materials
<img width="799" height="578" alt="image" src="https://github.com/user-attachments/assets/516c00b5-cc09-445e-91a8-6cca89e1db7f" />

### Printed Parts
<img src="../Images/Barrel_solo.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Barrel</p>
</div>

<img width="749" height="730" alt="image" src="https://github.com/user-attachments/assets/d8dd5176-e6f2-4c2a-8928-1e2e82c19bd8" />
<div className="w-full flex items-center justify-center">
<p className="italic">Flywheel</p>
</div>

<img src="../Images/motor_mount.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Motor Mount</p>
</div>

<img src="../Images/Tube_support.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Tube Support</p>
</div>

<img src="../Images/Camera_Mount.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Camera Mount</p>
</div>

<img width="888" height="487" alt="image" src="https://github.com/user-attachments/assets/5ce46832-09a0-4a02-b0f5-ebb5e5999035" />
<div className="w-full flex items-center justify-center">
<p className="italic">Pusher Arm</p>
</div>

## Assembly Instruction
1. Before the firing assembly can be mounted on the turtlebot, the base Burger Turtlebot needs to be modified. 

- Shift the mounting points of the Raspberry Pi towards the rear and remount the Raspberry Pi.
- Shift the mounting points of the Lidar controller* just to the right of the Raspberry Pi and remount the Lidar controller.
- Install the motor driver just in front of the Raspberry Pi, making sure the motor output of the board faces the front of the robot.
- Add M3x20 brass spacers on the posts to extend the height of the layer.
- Add M2.5x20 brass spacers to the lidar to extend the height of the lidar. 

2. Assembly of firing assembly
- Install the motor to the motor mount.
- Install mastic tape on the flywheel and install it onto the motor, making sure to fuse the ends together as much as possible.
- Install the motor mount to the barrel.
- Install the servo pusher arm to the servo arm.
- Install the servo to the barrel, being careful to position the arm so that it does not hit the wall of the barrel.

<img src="../Images/Motor_and_mount.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Motor and mount</p>
</div>

<img src="../Images/Motor_mount_flywheel.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Motor Assembly</p>
</div>

<img src="../Images/Barrel_Iso.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Barrel Assembly</p>
</div>

3. Assemble the firing assembly to the Turtlebot
- Install the barrel assembly to Turtlebot, lining up the mounting tabs between layers 2 and 3.

<img src="../Images/Bot_layers.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Turtlebot Layers</p>
</div>

4. Assembly of tube
- Install the vacuum hose on the intake of the barrel and tighten the hose clamp.
- Wrap the hose around the turtlebot and note where the top of the hose is positioned.
- Install the first tube support at the top of the hose and secure the tube to the support with a cable tie.
- Install 2 additional tube supports to ensure the vacuum tube is properly secured and does not flop.
- Ensure functionality of the tube by ensuring that the ping pong balls roll down the tube and into the barrel.

### Important Installation Notes
1. When installing the motor to the barrel, use a ping pong ball as a guide. The ping pong ball should not be able to roll down the barrel when the motor-flywheel assembly is mounted. Add spacers between the motor mount and barrel as necessary to get the desired compression with the ping pong balls. From our testing, using an M4 nut as a spacer gave us the appropriate compression for our desired firing behaviour. 

2. Coat the solder joints of the motor to protect it from strain. In our case, we used hot glue.

3. Ensure all the cables are secured within the bot's footprint as much as possible, so that it does not drag on the ground, possibly get caught in the tires or get caught on the terrain of the course.

## Launch Calculations
For the launch calculations, we aim to determine what RPM a flywheel of radius 23mm would need to launch a ping pong ball 40cm at a 20 degree angle, with no starting and ending height of y=0cm, assuming 50% efficiency of energy transfer between the flywheel and the ball.
<img width="306" height="186" alt="image" src="https://github.com/user-attachments/assets/57be61f5-51f4-44be-a0a6-94a7d4560331" />
<img width="854" height="93" alt="image" src="https://github.com/user-attachments/assets/b992f956-9525-4244-afcb-2d03a83ce02c" />
<img width="331" height="196" alt="image" src="https://github.com/user-attachments/assets/e8c34f15-610b-40b2-9c14-20b83aec56d7" />
<img width="286" height="119" alt="image" src="https://github.com/user-attachments/assets/bddde288-acb2-4f2d-836d-8b705f872885" />
<img width="372" height="68" alt="image" src="https://github.com/user-attachments/assets/1cbc9642-6616-47ef-aa58-d45a71fe7655" />
<img width="276" height="105" alt="image" src="https://github.com/user-attachments/assets/3e197ae2-279c-4e2d-b0ba-7628a0f0046c" />
<img width="248" height="122" alt="image" src="https://github.com/user-attachments/assets/ee257894-6a97-4f6c-9974-5995fd1ac546" />
Hence, we chose the RS360 Motor as it has a no-load RPM of >10000, to give us more flexibility to fire from a greater distance if need be.

## Design Reasoning
1. A flexible hose was chosen for holding the ping pong balls as its flexibility allows for more room for error and relies on placement of the flexible mounts rather than a rigid container. This allows us to achieve a relatively small footprint compared to other teams.
2. A single motor was chosen to reduce complexity and cost of the project. 2 motors might result in inconsistent trajectory should motor control not be precise.
3. A separate mount for the motor was chosen for added flexibility in turing the contact of the ping pong ball with the flywheel.
4. A servo arm with an extension was chosen to actuate firing the ping pong ball as it provides the range of motion required without the complexity of cams or gears. When the arm is in the firing position, it prevents the next ping pong ball from rolling into the barrel.

<img src="../Images/Barrel_Section_Load.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Arm in loaded position</p>
</div>

<img src="../Images/Barrel_Section_Fire.png" width="400" alt="Barrel">
<div className="w-full flex items-center justify-center">
<p className="italic">Arm in firing position</p>
</div>

5. The mounting holes for the motor mount and barrel are slots, rather than holes. This allows for some flexibility in mounting and positioning to ensure full functionality of the bot.
