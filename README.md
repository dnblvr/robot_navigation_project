<!-- omit in toc -->
# Autonomous Navigator

Principal Authors: &nbsp; **Lucero Aguilar-Larios** & **Gian Fajardo**

Faculty Advisor: &emsp; **Dr. Shahnam Mirzaei**

<!-- markdownlint-disable MD059 -->
<!-- markdownlint-disable MD033 -->

<!-- omit in toc -->
## Table of Contents

- [1. Goals](#1-goals)
- [2. Methods](#2-methods)
- [3. Materials](#3-materials)
- [4. Online Resources](#4-online-resources)
  - [4.1. Things I've made to explain this stuff](#41-things-ive-made-to-explain-this-stuff)
  - [4.2. C library](#42-c-library)
  - [4.3. Hardware Development](#43-hardware-development)
  - [4.4. Sensor Fusion with MARG sensor](#44-sensor-fusion-with-marg-sensor)
  - [4.5. Iterative Closest Point (ICP)](#45-iterative-closest-point-icp)
  - [4.6. GraphSLAM](#46-graphslam)
  - [4.7. EKF-SLAM](#47-ekf-slam)
  - [4.8. RRT\*](#48-rrt)
- [5. Checklist](#5-checklist)
  - [Task 1: 12 Dec 2025 (3 weeks)](#task-1-12-dec-2025-3-weeks)
  - [Task 2](#task-2)
  - [Ongoing Task](#ongoing-task)

---

## 1. Goals

![alt text](<./images/image 1.png>)

&emsp;&emsp; The scope of this project is to explore the fundamentals of LiDAR-based 2D mapping and autonomous navigation. In a world reliant on more autonomous systems, these systems need to navigate effectively as they can enable efficient and precise movements in complex environments without human intervention. Furthermore, the system needs to prioritize the safety of the human agents around it. In part, it should traverse dangerous places that humans cannot survive. Exploration of our oceans, disasters created from war, and planets can and do benefit from having autonomous systems. To do this, Autonomous Systems need to identify its environment. From there, motion planning algorithms are applied.

&emsp;&emsp; Whereas other work uses other autonomous systems like humanoids, which will have to adopt different mathematical models to describe their states, our project is different. To see if this plan is feasible, we will simplify the scope to make a two-wheeled robot navigator from the TI-RSLK chassis. This navigator is to be equipped with a LiDAR scanner, the RPLiDAR C1. The goal of this research project is to make an autonomous system that:

1. scan its environment through two different modes
   1. GraphSLAM method
   2. EKF-SLAM method
<!-- 2. navigates from one user-defined coordinate to another all while avoiding obstacles. -->

## 2. Methods

Here is how we achieve our goals:

- We will recreate its pose and its environment in-memory using the graph-based Simultaneous Localization and Mapping (GraphSLAM) algorithm and Kalman filter based using the multiple LiDAR ICs mentioned before.

- We perform sensor fusion via the extended Kalman Filter (EKF) which will hopefully gather a better estimate of its state without the expected drift from GraphSLAM alone. We intend to use additional sensors like a MARG sensor and a GPS receiver.

- For methods, we will consider using new approaches. I was considering investigating scan matching methods (ICP) as a second point of measurement for EKF-SLAM.

<!-- - We will also write a motion planning algorithm called the Rapidly-Exploring Random Trees (RRT*) or some equivalent. -->

## 3. Materials

The materials involved include:

|Amounts|Name           |Description              |
|------:|:-------------:|:-----------------------:|
|      1|TI-RSLK Chassis|[Two-Wheeled Platform][classic-ti-rslk-part]|
|      1|RPLiDAR C1     |[2D LiDAR Scanner][rplidar-c1-scanner]|
|      1|ICM-20948      |[MARG Sensor][sparkfun-icm-20948]|
|      1|GPS Receiver   |[N/A][example-com]|
|      6|VL53L5CX       |[SparkFun ToF Imager][tof-imager-vl53l5cx]|

<!-- **Sources**: -->

[classic-ti-rslk-part]: https://www.pololu.com/category/268/classic-ti-rslk-parts-and-accessories

[rplidar-c1-scanner]: https://www.slamtec.com/en/C1

[tof-imager-vl53l5cx]: https://www.sparkfun.com/sparkfun-qwiic-tof-imager-vl53l5cx.html

[example-com]: example.com

[sparkfun-icm-20948]: (http://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html)

## 4. Online Resources

### 4.1. Things I've made to explain this stuff

- the [CSUN-SPaRA presentation][] I spoke at to explain my progress

[CSUN-SPaRA Presentation]: https://docs.google.com/presentation/d/1J_DLSFxBZO6KGPgfmmhxmArGXOe7M399n4DHQqT86TA/edit?usp=sharing

### 4.2. C library

- [The GNU C Library Reference Manual - GNU.org][]

### 4.3. Hardware Development

- [RPLiDAR C1 Support Page][]
  - [RPLiDAR S&C Series Protocol v2.8][rplidar-c1-application-notes]
    - [what NotebookLM says about the protocol][]
<br>

- Alongside this project, I have a self-made API that I've made in reaction to how the given SDK is confusing to read. At the time of writing (10 Dec 2025), this is a fully-functioning driver but I have high hopes that we can further improve this API.
  - there are no plans to make this platform-agnostic. sorry.
  - [accompanying RPLiDAR C1 UART ISR-driven firmware - dnblvr][UART ISR-driven firmware]

---

- This collection of links might be a very interesting use of firmware optimization using the already-existing ARM-v4 optimized CMSIS libraries
- [PowerPoint Presentation - msp432.pdf][]
- [SIMPLELINK-MSP432-SDK Software development kit \(SDK\) | TI.com][]
- [using CMSIS-DSP for msp432 system - Google Search][]
- [CMSIS DSP Software Library][]
- [DSP Libraries — Real Time Digital Signal Processing B Term 2024 documentation][]

---

- This collection of links is in the event that I *did* want to transition to my own boards instead of the (kindly provided) CSUN-supplied `MSP432P401R`
- [RainerKuemmerle/g2o: g2o: A General Framework for Graph Optimization][]
  - [g2o: A General Framework for Graph Optimization - paper.pdf][]
    - [G2o: A general framework for graph optimization | IEEE Conference Publication | IEEE Xplore][]
- [Prepare build with Eigen release v5.0.0 · RainerKuemmerle/g2o@ef80e64][]
- [i.MX RT106x Crossover Processors for EdgeReady™ Off-The-Shelf ML/AI IoT Edge Compute Solutions - IMXRT1060CEC_SUPPLEMENT.pdf][]
- [IMXRT1060CEC_Rev_4 - IMXRT1060CEC.pdf][]

<!-- **Sources**: -->

[RainerKuemmerle/g2o: g2o: A General Framework for Graph Optimization]: https://github.com/RainerKuemmerle/g2o "RainerKuemmerle/g2o: g2o: A General Framework for Graph Optimization"

[g2o: A General Framework for Graph Optimization - paper.pdf]: https://mengwenhe-cmu.github.io/Reading-Reports/Research/Localization/Graph_Optimization/g2o_A_General_Framework_for_Graph_Optimization/paper.pdf "g2o: A General Framework for Graph Optimization - paper.pdf"

[G2o: A general framework for graph optimization | IEEE Conference Publication | IEEE Xplore]: https://ieeexplore.ieee.org/document/5979949 "G2o: A general framework for graph optimization | IEEE Conference Publication | IEEE Xplore"

[Prepare build with Eigen release v5.0.0 · RainerKuemmerle/g2o@ef80e64]: https://github.com/RainerKuemmerle/g2o/commit/ef80e643adeb700536dd282dd4316c90cfc05fe8 "Prepare build with Eigen release v5.0.0 · RainerKuemmerle/g2o@ef80e64"

[i.MX RT106x Crossover Processors for EdgeReady™ Off-The-Shelf ML/AI IoT Edge Compute Solutions - IMXRT1060CEC_SUPPLEMENT.pdf]: https://www.nxp.com/docs/en/data-sheet/IMXRT1060CEC_SUPPLEMENT.pdf "i.MX RT106x Crossover Processors for EdgeReady™ Off-The-Shelf ML/AI IoT Edge Compute Solutions - IMXRT1060CEC_SUPPLEMENT.pdf"

[IMXRT1060CEC_Rev_4 - IMXRT1060CEC.pdf]: https://www.nxp.com/docs/en/nxp/data-sheets/IMXRT1060CEC.pdf "IMXRT1060CEC_Rev_4 - IMXRT1060CEC.pdf"

[PowerPoint Presentation - msp432.pdf]: https://faculty-web.msoe.edu/johnsontimoj/EE2920/files2920/msp432.pdf "PowerPoint Presentation - msp432.pdf"
[SIMPLELINK-MSP432-SDK Software development kit \(SDK\) | TI.com]: https://www.ti.com/tool/SIMPLELINK-MSP432-SDK#downloads "SIMPLELINK-MSP432-SDK Software development kit \(SDK\) | TI.com"
[using CMSIS-DSP for msp432 system - Google Search]: https://www.google.com/search?client=firefox-b-1-d&q=using+CMSIS-DSP+for+msp432+system "using CMSIS-DSP for msp432 system - Google Search"
[CMSIS DSP Software Library]: https://arm-software.github.io/CMSIS_5/DSP/html/index.html "CMSIS DSP Software Library"
[DSP Libraries — Real Time Digital Signal Processing B Term 2024 documentation]: https://schaumont.dyn.wpi.edu/ece4703b23/lecture6.html#data-types "DSP Libraries — Real Time Digital Signal Processing B Term 2024 documentation"

[RPLiDAR C1 Support Page]: https://www.slamtec.com/en/support#rplidar-c1

[UART ISR-driven firmware]: https://github.com/dnblvr/robot_navigation_project/tree/main/RPLiDAR_C1_API "robot_navigation_project/RPLiDAR_C1_API at main · dnblvr/robot_navigation_project"

[what NotebookLM says about the protocol]: https://notebooklm.google.com/notebook/ff66011a-9be7-4914-9a73-cb1ee842dfcc?original_referer=https:%2F%2Fnotebooklm.google%23&pli=1

[rplidar-c1-application-notes]: https://bucket-download.slamtec.com/c5971f2703a8d014f3925694d798ea490a370efa/LR001_SLAMTEC_rplidar_S&C%20series_protocol_v2.8_en.pdf

### 4.4. Sensor Fusion with MARG sensor

- [SparkFun\_ICM\-20948\_ArduinoLibrary\/examples\/PortableC\/Example999\_Portable\/Example999\_Portable\.ino at main · sparkfun\/SparkFun\_ICM\-20948\_ArduinoLibrary][]

### 4.5. Iterative Closest Point (ICP)

- [Iterative Closest Point \(ICP\) - 5 Minutes with Cyrill - YouTube][]
- [Singular value decomposition - Wikipedia][]

[Iterative Closest Point \(ICP\) - 5 Minutes with Cyrill - YouTube]: https://www.youtube.com/watch?v=QWDM4cFdKrE "Iterative Closest Point \(ICP\) - 5 Minutes with Cyrill - YouTube"
[Singular value decomposition - Wikipedia]: https://en.wikipedia.org/wiki/Singular_value_decomposition "Singular value decomposition - Wikipedia"

### 4.6. GraphSLAM

- [Understanding SLAM Using Pose Graph Optimization \| Autonomous Navigation\, Part 3 \- YouTube][]
- [A brief introduction to GraphSLAM \| by Shiva Chandrachary \| Medium][]
  - [Application of Maximum Likelihood Estimation in GraphSLAM \| by Shiva Chandrachary \| Medium][]
  - [An application of Numerical Solution to Maximum Likelihood Estimation in GraphSLAM \| by Shiva Chandrachary \| Medium][]
- [Graph SLAM: From Theory to Implementation | Federico Sarrocco][]

<br>

- [The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures - Thrun et. al.][]

- [A Tutorial on Graph-Based SLAM - Grisetti et. al.][]
  - loading times are longer than usual so please be patient!

<!-- **Sources**: -->

[The GraphSLAM Algorithm with Applications to Large-Scale Mapping of Urban Structures - Thrun et. al.]: https://robots.stanford.edu/papers/thrun.graphslam.pdf

[Graph SLAM: From Theory to Implementation | Federico Sarrocco]: https://federicosarrocco.com/blog/graph-slam-tutorial "Graph SLAM: From Theory to Implementation | Federico Sarrocco"

[A Tutorial on Graph-Based SLAM - Grisetti et. al.]: http://www2.informatik.uni-freiburg.de/~stachnis/pdf/grisetti10titsmag.pdf

### 4.7. EKF-SLAM

- [The Extended Kalman Filter: An Interactive Tutorial][]
- [EKF-SLAM A Very Quick Guide - SLAM course.pdf][]

[The Extended Kalman Filter: An Interactive Tutorial]: https://simondlevy.github.io/ekf-tutorial/ "The Extended Kalman Filter: An Interactive Tutorial"
[EKF-SLAM A Very Quick Guide - SLAM course.pdf]: https://www.iri.upc.edu/people/jsola/JoanSola/objectes/curs_SLAM/SLAM2D/SLAM%20course.pdf "EKF-SLAM A Very Quick Guide - SLAM course.pdf"

### 4.8. RRT\*

- [Path Planning with A\* and RRT \| Autonomous Navigation\, Part 4 \- YouTube][]
- [path finding \- rapid exploring random trees \- Stack Overflow][]

<!-- **Sources**: -->

[The GNU C Library Reference Manual - GNU.org]: https://www.gnu.org/software/libc/manual/pdf/libc.pdf "libc.pdf"

[SparkFun\_ICM\-20948\_ArduinoLibrary\/examples\/PortableC\/Example999\_Portable\/Example999\_Portable\.ino at main · sparkfun\/SparkFun\_ICM\-20948\_ArduinoLibrary]: (https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary/blob/main/examples/PortableC/Example999_Portable/Example999_Portable.ino) "SparkFun_ICM-20948_ArduinoLibrary/examples/PortableC/Example999_Portable/Example999_Portable.ino at main · sparkfun/SparkFun_ICM-20948_ArduinoLibrary"

[Path Planning with A\* and RRT \| Autonomous Navigation\, Part 4 \- YouTube]: (https://www.youtube.com/watch?v=QR3U1dgc5RE) "\(94\) Path Planning with A* and RRT | Autonomous Navigation, Part 4 - YouTube"

[path finding \- rapid exploring random trees \- Stack Overflow]: (https://stackoverflow.com/questions/11933385/rapid-exploring-random-trees) "path finding - rapid exploring random trees - Stack Overflow"

[Understanding SLAM Using Pose Graph Optimization \| Autonomous Navigation\, Part 3 \- YouTube]: (https://www.youtube.com/watch?v=saVZtgPyyJQ&t=161s) "\(94\) Understanding SLAM Using Pose Graph Optimization | Autonomous Navigation, Part 3 - YouTube"

[A brief introduction to GraphSLAM \| by Shiva Chandrachary \| Medium]: (https://shivachandrachary.medium.com/a-brief-introduction-to-graphslam-4204b4fce2f0) "A brief introduction to GraphSLAM | by Shiva Chandrachary | Medium"

[Application of Maximum Likelihood Estimation in GraphSLAM \| by Shiva Chandrachary \| Medium]: (https://shivachandrachary.medium.com/application-of-maximum-likelihood-estimation-in-graphslam-db4897f0083b) "Application of Maximum Likelihood Estimation in GraphSLAM | by Shiva Chandrachary | Medium"

[An application of Numerical Solution to Maximum Likelihood Estimation in GraphSLAM \| by Shiva Chandrachary \| Medium]: https://shivachandrachary.medium.com/an-application-of-numerical-solutions-to-maximum-likelihood-estimation-in-graphslam-31a7284721e3 "An application of Numerical Solution to Maximum Likelihood Estimation in GraphSLAM | by Shiva Chandrachary | Medium"

## 5. Checklist

In general, for all tasks, they should follow the same guidelines:

- [ ] make the code happen in any IDE or simulator
- [ ] simulate it in normal C code in Visual Studio Code, MATLAB, or any other IDE, if possible
  - [ ] if simulating in MATLAB, use its C-code converter
- [ ] optimize the functions once done, if applicable

### Task 1: 12 Dec 2025 (3 weeks)

- [ ] improve motor control and motor tachometer functions
  - [ ] perhaps incorporate the `ICM-20948` (exclusively I^2^C) to supplement the tachometer-only motion model
- [ ] improve GraphSLAM
  - [ ] loop closure detection
    - [ ] develop auto-steering/local planning using the Dynamic Window Approach (DWA) to prevent direct steering
  - [ ] perhaps explore other methods like HECTOR SLAM (will provide links)

### Task 2

- [ ] explore EKF-SLAM

### Ongoing Task

- [ ] meet every Monday to check on our weekly progress

| Monday |                                Task                                 |
|-------:|:--------------------------------------------------------------------|
| 14 Dec | <input type="checkbox"> 1^st^ progress report                       |
| 21 Dec | <input type="checkbox"> 2^nd^ progress report    <br><input type="checkbox"> give time to decide to stick by the project |
| 28 Dec | <input type="checkbox"> 3^rd^ progress report                       |
