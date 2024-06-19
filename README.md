## WQF7010 Alternative Assessment Prototype Development
A human-robot interaction prototype program that applies AI algorithms based on computer vision, including OpenCV face detection, YOLO v3 object and people detection. Additionally, the program incorporates speech-to-text processing. This program runs on the [Jupiter Robot](http://www.jupiterobot.com/), an educational robot platform designed for AI-focused robotics education and service robotics development.

The following demo videos are available at [this link](https://photos.app.goo.gl/pfZ9KLWN3U8vgNVh9):
* Receptionist demo (Main program)
* Delivery detection demo (Subprogram) 

## Installation Instructions
Follow the quick setup guide below to run the prototype program. Note that the project is developed under Ubuntu 18.04, utilizing ROS Melodic and Python 2, as required by compatibility with the Jupiter Robot.
1. Create directories for ROS workspace.
```
mkdir -p ~/catkin_ws/src
```
2. Change the current working directory to the `src` directory inside catkin workspace.
```bash
cd ~/catkin_ws/src
```
3. Download a copy of the prototype program package to your local machine from the remote Git repository.
```bash
git clone https://github.com/23060314/WQF7010-AA-Prototype-Development.git
```
4. Return to `catkin_ws` directory.
```bash
cd ..
```
5. Run the following command to build the workspace. It configures and generates the necessary files for all packages.
```bash
catkin_make
```
6. Configure current terminal session to recognize and use the packages and tools available in the ROS catkin workspace.
```bash
source ~/catkin_ws/devel/setup.bash
```
7. Launch the prototype program.
```bash
roslaunch wqf7010_aa_prototype_development fsktm_receptionist.launch
```
## Authors and acknowledgment
This project was carried out under the supervision of Dr. Zati Hakim, department of Artificial Intelligence, University Malaya, and completed by the following students:
* Ahmad Syahir (22110047)
* Khor Shu Yan (22121327)
* Teo Yin Yin  (22111371)
* Tham Weng Wai (23060314)
