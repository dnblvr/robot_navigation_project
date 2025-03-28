# Robot Navigation Project

Principal Authors: &nbsp; **Lucero Aguilar-Larios** & **Gian Fajardo**

Faculty Advisor: &emsp; **Dr. Shahnam Mirzaei**

## Table of Contents

* [Goals](#goals)
* [Methods](#methods)
* [Materials](#materials)
* [Online Resources](#online-resources)
* [Checklist](#checklist)
  * [Main Tasks](#main-tasks)


## Goals

![alt text](<./images/image 1.png>)

&emsp;&emsp; The scope of this project is to explore the fundamentals of autonomous navigation. In a world reliant on more autonomous systems, these systems need to navigate effectively as they can enable efficient and precise movements in complex environments without human intervention. Furthermore, the system needs to prioritize the safety of the human agents around it. In part, it should traverse dangerous places that humans cannot survive. Exploration of our oceans, disasters created from war, and planets can and do benefit from having autonomous systems. To do this, Autonomous Systems need to identify its environment. From there, motion planning algorithms are applied.

&emsp;&emsp; Whereas other work uses other autonomous systems like humanoids, which will have to adopt different mathematical models to describe their states, our project is different. To see if this plan is feasible, we will simplify the scope to make a two-wheeled robot navigator from the TI-RSLK chassis. This navigator is to be equipped with a set of solid-state multi-zone LiDAR sensors placed around the chassis. The goal of this research project is to make an autonomous system that: 
1. scans its environment, and
2. navigates from one user-defined coordinate to another all while avoiding obstacles.

## Methods

Here is how we achieve our goals:

* We will recreate its pose and its environment in memory using the graph-based Simultaneous Localization and Mapping (GraphSLAM) algorithm using the multiple LiDAR ICs mentioned before.

* We will also write a motion planning algorithm called the Rapidly-Exploring Random Trees (RRT*) or some equivalent.

* If there is time, we plan to incorporate sensor fusion via the extended Kalman Filter (EKF) which will hopefully gather a better estimate of its state without the expected drift from graphSLAM alone. We intend to use additional sensors like a MARG sensor and a GPS receiver.

## Materials

The materials involved include:

|Amounts|Name           |Description              |Link  | 
|------:|:-------------:|:-----------------------:|------|
|      1|TI-RSLK Chassis|Two-Wheeled Platform     |[link](https://www.pololu.com/category/268/classic-ti-rslk-parts-and-accessories)| 
|    4-6|VL53L5CX       |SparkFun Qwiic ToF Imager|[link](https://www.sparkfun.com/sparkfun-qwiic-tof-imager-vl53l5cx.html)|
|      1|GPS Receiver   |N/A                      |[link](https://www.amazon.com/Compass-Precision-Receiver-Navigation-Compatible/dp/B08NY9JSZ3/ref=sr_1_2?dib=eyJ2IjoiMSJ9.XT_dNZlid3N-zIIKsz0sS6ufhONAOEd6FSHYqXoP1tgYoVX7mtJPDXYRUguvWZ7W367EmTH3uLescGuIo7sPs-TMsNMaFllqgas-jb2gNPZ1uJGPUgV_eVoV2rAoHWB1nxIc1jXiEHc8nMauXs_k15Q2yoDn5R3qHHB0jo1v-GS-xK5vMUVpUvDs8qQtRbzrj0wwi5eAwvVcUlPNiZev8AdNTxt2Km72APc1Xbaab4FttpjDiZD9_e8H1B8XR71YBkdhbD8iYK01ZLLfikIhatyjCK7LOw4M4rc5JLc5yuoaY-Jowv8odZFbYbH26AwIm8j2M3-KEOD-rTCtMG0HAgbnBJ_h9qgQYffT5arNXN3Qbvv4CajLsvO4z03AARv4hzvp11JcmnTkw8BuDAQrxHz36J_WWOgW0XFisVyl71PsU8URlDe6dpef7FJaf8ui.5AMQjqJR9CL6uwsu181CL24B4M93u4Ul71RllzvcN3M&dib_tag=se&keywords=pixhawk+gps&mfadid=adm&qid=1741988169&sr=8-2)|
|      1|ICM-20948      |MARG Sensor              |[link](http://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html)

## Online Resources

* [libc\.pdf](https://www.gnu.org/software/libc/manual/pdf/libc.pdf "libc.pdf")
* [SparkFun\_ICM\-20948\_ArduinoLibrary\/examples\/PortableC\/Example999\_Portable\/Example999\_Portable\.ino at main · sparkfun\/SparkFun\_ICM\-20948\_ArduinoLibrary](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/examples/PortableC/Example999_Portable/Example999_Portable.ino "SparkFun_ICM-20948_ArduinoLibrary/examples/PortableC/Example999_Portable/Example999_Portable.ino at main · sparkfun/SparkFun_ICM-20948_ArduinoLibrary")
* [\(94\) Path Planning with A\* and RRT \| Autonomous Navigation\, Part 4 \- YouTube](https://www.youtube.com/watch?v=QR3U1dgc5RE "\(94\) Path Planning with A* and RRT | Autonomous Navigation, Part 4 - YouTube")
* [\(94\) Understanding SLAM Using Pose Graph Optimization \| Autonomous Navigation\, Part 3 \- YouTube](https://www.youtube.com/watch?v=saVZtgPyyJQ&t=161s "\(94\) Understanding SLAM Using Pose Graph Optimization | Autonomous Navigation, Part 3 - YouTube")
* [A brief introduction to GraphSLAM \| by Shiva Chandrachary \| Medium](https://shivachandrachary.medium.com/a-brief-introduction-to-graphslam-4204b4fce2f0 "A brief introduction to GraphSLAM | by Shiva Chandrachary | Medium")
  * [Application of Maximum Likelihood Estimation in GraphSLAM \| by Shiva Chandrachary \| Medium](https://shivachandrachary.medium.com/application-of-maximum-likelihood-estimation-in-graphslam-db4897f0083b "Application of Maximum Likelihood Estimation in GraphSLAM | by Shiva Chandrachary | Medium")
  * [An application of Numerical Solution to Maximum Likelihood Estimation in GraphSLAM \| by Shiva Chandrachary \| Medium](https://shivachandrachary.medium.com/an-application-of-numerical-solutions-to-maximum-likelihood-estimation-in-graphslam-31a7284721e3 "An application of Numerical Solution to Maximum Likelihood Estimation in GraphSLAM | by Shiva Chandrachary | Medium")


## Checklist

In general, for all tasks, they should follow the same guidelines:
* [ ] make the code happen in any IDE or simulator
* [ ] simulate it in normal C code in Visual Studio Code, MATLAB, or any other IDE, if possible
  * [ ] if simulating in MATLAB, use its C-code converter
* [ ] optimize the functions once done, if applicable

### Main Tasks

#### Both Gian and Lucy:
* [ ] RRT* simulation on C (Visual Studio Code)
* [ ] multizone LiDAR configuration (wk 9-11)
  * [ ] configure with Timer_Ax interrupt hardware
  * [ ] configure I2C with interrupt!
  * [ ] implement state machine to know when to read

* [ ] combine all our code together (wk 12-13)
     
#### Gian:
* [ ] Odometry  (wk 9-10)
* [ ] GraphSLAM (wk 9-11)
  * [ ] I will look into what the inputs and outputs of this thing will require

* [ ] RRT* implementation on MSP 432 (wk 12)
  * [ ] configure the correct I/O required
