# VAFOR: Active physical world voice assistant for object retrieval
The goal of this project is to use Computer Vision together with Natural Language Processing in order to carry out smart pick and place operations on TurtleBot3. Technologies used are YOLOv3, ROS, spaCy, GPT-3.
The white paper accepted to IEEE RO-MAN 2023 is available [here](https://drive.google.com/file/d/1ZyyFAxzLZrfd8GQL85u2AtmzXa0ZGq2k/view?usp=sharing).

## Table of Contents

- [Introduction](#introduction)
- [How to Use](#how-to-use)
- [Contributing](#contributing)
- [Datasets](#datasets)
- [Results](#results)
- [License](#license)

## Introduction

This repository is created as part of the VAFOR: Proactive Voice Assistant for Object Retrieval in the Physical World project to facilitate transparency, future collaboration, and research on this topic. 

## How to Use

This codebase is configured to run on a Turtlebot3 Waffle Pi robot with Open-Manipulator X, Raspberry Pi 4 SBC for robot control and Jetson Nano DevKit for Object detection. Additionally, a PC with Ubuntu 18 is required to be used as Remote PC. Instructions on how to set up the hardware are [here](https://hyeminahn.notion.site/How-to-use-Turtlebot-2c4bde9b08704de5bdead2aa2cba6f76?pvs=4). If you just want to analyze the results and outputs of NLP module variants these configurations are not necessary. In that case, all you need to do is follow steps 1 and 2. 

To run vafor, first configure your robotic setup as instructed above. Then follow the steps below:

1. Clone the repository to your local machine:

```
git clone https://github.com/bekatan/vafor.git
```
2. Change into the repository directory:

```
cd cse362
```
3. Install OpenAI SDK
```
pip install openai
```
4. Save your OpenAI private key in an environment variable "OPENAI_API_KEY"

5. ssh into your Pi and Jetson on different terminals

6. Start roscore and vafor on separate terminals on Remote PC
```
roscore
```
```
python vafor.py
```
7. Start Yolov3 for ROS on Jetson nano. On separate terminals run
```
roslaunch ros_deep_learning video_source.ros1.launch input:=csi://0 output:=display://0
```
```
python converter.py
```
```
roslaunch darknet_ros darknet_ros.launch 
```
8. Launch the robot on Pi4
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
9. In order to display the output of the object detector on the Remote PC run these commands on separate terminals
```
python converter.py
```
```
rosrun image_view image_view image:=/detection_image
```
10. The commands to operate the robot will be displayed in the terminal where the `vafor.py` is run.

## Datasets

Testing and fine-tuning datasets are available in `vafor_tests.csv`, and `finetuning.csv` files in the `data` folder. Both datasets contain comma-separated values of 'sentence' and its expected 'target'. The fine-tuning dataset is converted into `finetuning-prepared.jsonl`, which is compatible with OpenAI SDK using their utilities.

## Results

The output of each NLP module variant is saved in the `results` folder. Results are saved in comma-separated values of 'sentence', 'target', 'response', and 'mark'. In order to see the accuracy statistics run the `statistics.py` script.

## Contributing

Contributions to this repository are welcome! If you find any issues or want to enhance the code or documentation, feel free to create a pull request. Make sure to follow the general code of conduct when contributing.

## License

This project is licensed under the [MIT License](LICENSE). You are free to use, modify, and distribute the code as long as you provide appropriate attribution to the original authors. See the [LICENSE](LICENSE) file for more details.

---
