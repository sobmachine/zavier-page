---
layout: default
title: Home
nav_order: 1
description: "Overview and documentation for the Zavier Smart Dustbin project built using YOLOv5, Kalman filtering, and AI-powered automation."
---

# Zavier: Autonomous Predictive Waste Management System

**Authors:** Harshit Bery et al.  
**Date:** November 2025

---

## **Abstract**

The increasing inefficiency and lack of hygienic awareness in public and private environments have led to a fundamental need for an automated waste management solution. **Zavier** is a self-navigating, predictive, and user-interactive robotic waste collector designed to reduce manual effort and improve cleanliness in shared spaces such as classrooms, offices, and homes. Built using STM32F411CEU6, ESP8266, and Luckfox Pico RV1103 platforms, the system leverages Kalman-trained motion prediction, YOLO-based trash detection, and real-time PID control to autonomously reach users or intercept thrown waste midair.  
This paper presents the complete design, architecture, mathematical modeling, and software stack of Zavier, alongside analysis, limitations, and future development opportunities.

**Keywords:**  
Autonomous Robotics, Waste Management, PID Control, YOLO, Embedded Systems, ESP8266, RV1103, Kalman Filter, STM32F411CEU6

---

## **1. Introduction**

Human laziness and disregard for hygienic maintenance in communal environments, particularly among students and young adults, present a recurring issue. Despite widespread awareness campaigns, physical labor remains a deterrent to consistent waste disposal. The Zavier system addresses this behavioral gap by automating the act of waste collection itself.

The project’s primary goal is to enable effortless hygiene — allowing individuals to dispose of waste without moving from their workspace. Zavier integrates intelligent navigation, waste detection, and automated response through dual robotic units: a Collector Bot and a Support Bot (equipped with a robotic arm). The synergy between these units and their microcontroller architecture forms the backbone of a seamless waste-handling process.

---

## **2. Problem Statement and Ideation**

### **2.1 Identified Problem**

Conventional waste management systems depend on human involvement for both disposal and cleaning. This dependency introduces irregularity, especially in classrooms or labs where time and effort are prioritized elsewhere. The physical act of reaching a dustbin or maintaining it is considered “low-priority.”

### **2.2 Ideation**

The conceptual stage revolved around a core question:  
**“Can hygiene be automated without needing human initiative?”**  
From that emerged **Zavier** — a robotic waste management ecosystem that comes to the user rather than the other way around. It acts upon motion detection, gesture signals, or vocal call commands to autonomously approach, open its waste compartment, and dispose of the trash.

---

## **3. System Architecture**

The Zavier ecosystem comprises three core components:

### **Collector Bot**
- 3-wheeled base  
- Powered by an ESP8266 and Luckfox Pico RV1103  
- Equipped with a primary RGB camera and ultrasonic distance sensor  

### **Support Bot (Arm Unit)**
- Controlled via ESP8266  
- Robotic arm with 3 axes (base rotation, arm lift, wrist tilt)  
- Assists in waste collection when detection or interception fails  

### **Docking and Management Unit**
- Managed by STM32F411CEU6  
- Responsible for charging, scheduling, and maintaining fleet synchronization  
- Communication between bots is maintained via Wi-Fi (ESP-NOW protocol)  
- Docking synchronization follows PID-based charging regulation  

---

## **4. Hardware Components**

| Component | Function |
|------------|-----------|
| STM32F411CEU6 | Central control for docking and scheduling |
| ESP8266 | Wireless communication and remote control |
| Luckfox Pico RV1103 | Edge AI computations, YOLO inference |
| Ultrasonic Sensor | Obstacle avoidance and distance estimation |
| 720p RGB Camera | Object detection and tracking |
| DC Motors (x2) | Rear wheel actuation |
| Caster Wheel | Front steering balance |
| Li-ion Battery Pack | Power supply |
| Robotic Arm | 3-axis manipulator for trash pickup |

---

## **5. Software Architecture**

The system employs a hybrid stack:

- **Embedded C:** Firmware for STM32 and ESP8266 (control, motor drive, PID)  
- **Arduino C:** Intermediary communication layer  
- **Buildroot Linux:** Running on RV1103  
- **YOLOv5-Lite:** Optimized for TrashNet dataset  
- **Kalman-trained motion estimator:** For predictive waste interception  

All software modules communicate through message queues and low-latency serial protocols, maintaining **<50 ms inter-bot sync.**

---

## **6. Mathematical Modeling**

### **6.1 Kinematic Model**

The *Zavier* platform operates using a **differential drive configuration**, featuring two independently driven rear wheels and one front caster wheel for balance.  
The robot’s translational and rotational motion can be expressed as:

$$
\begin{aligned}
\dot{x} &= \frac{r}{2}(\omega_R + \omega_L)\cos\theta \\
\dot{y} &= \frac{r}{2}(\omega_R + \omega_L)\sin\theta \\
\dot{\theta} &= \frac{r}{L}(\omega_R - \omega_L)
\end{aligned}
$$

Where:  
- \( (x, y) \): Cartesian coordinates of the robot’s center  
- \( \theta \): Robot’s orientation  
- \( r \): Wheel radius  
- \( L \): Distance between wheels  
- \( \omega_R, \omega_L \): Angular velocities of right and left wheels  

Linear and angular velocities:

$$
v = \frac{r}{2}(\omega_R + \omega_L)
$$

$$
\omega = \frac{r}{L}(\omega_R - \omega_L)
$$

Wheel angular velocities (inverse kinematics):

$$
\begin{aligned}
\omega_R &= \frac{v + \frac{L}{2}\omega}{r} \\
\omega_L &= \frac{v - \frac{L}{2}\omega}{r}
\end{aligned}
$$

Pose update equations:

$$
\begin{aligned}
x_{t+\Delta t} &= x_t + v \cos(\theta_t)\Delta t \\
y_{t+\Delta t} &= y_t + v \sin(\theta_t)\Delta t \\
\theta_{t+\Delta t} &= \theta_t + \omega\Delta t
\end{aligned}
$$

---

### **6.2 PID Control**

PID controller regulates Zavier’s velocity and orientation:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau)d\tau + K_d \frac{de(t)}{dt}
$$

where \( e(t) = r(t) - y(t) \) is the error.

Wheel-specific control:

$$
\begin{aligned}
u_R(t) &= K_p e_R(t) + K_i \int e_R(t)dt + K_d \frac{de_R(t)}{dt} \\
u_L(t) &= K_p e_L(t) + K_i \int e_L(t)dt + K_d \frac{de_L(t)}{dt}
\end{aligned}
$$

Position control:

$$
e_p = \sqrt{(x_d - x)^2 + (y_d - y)^2}
$$

$$
e_\theta = \theta_d - \theta
$$

Adaptive gain scheduling modifies \( K_p, K_d \) dynamically to prevent overshoot.

![Alt text](/zavier-page/assets/images/pid.png)

---

### **6.3 Kalman Filter Prediction**

State vector:

$$
x_k =
\begin{bmatrix}
x \\ y \\ \dot{x} \\ \dot{y}
\end{bmatrix}
$$

State model:

$$
x_k = A x_{k-1} + B u_{k-1} + w_{k-1}
$$

$$
A =
\begin{bmatrix}
1 & 0 & \Delta t & 0 \\
0 & 1 & 0 & \Delta t \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix},
\quad
B =
\begin{bmatrix}
0.5\Delta t^2 \\ 0.5\Delta t^2 \\ \Delta t \\ \Delta t
\end{bmatrix}
$$

**Prediction:**

$$
\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + B u_{k-1}
$$
$$
P_{k|k-1} = A P_{k-1|k-1} A^T + Q
$$

**Update:**

$$
K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}
$$
$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H\hat{x}_{k|k-1})
$$
$$
P_{k|k} = (I - K_k H) P_{k|k-1}
$$

---

## **7. YOLO-based Detection (TrashNet)**

### **7.1 Overview**

Zavier employs **YOLOv5-Lite** for waste recognition, optimized for the **LuckFox Pico RV1103** (≈0.5 TFLOPS).  
It identifies waste categories — *plastic*, *paper*, *metal*, *organic* — in real time. Training uses **TrashNet**, extended with **Hugging Face** dataset augmentations.

---

### **7.2 Neural Pipeline**

1. **Image Input:** 720p feed @30FPS  
2. **Preprocessing:** resize → normalize → tensor convert  
3. **Forward Pass:** YOLOv5-Lite inference  
4. **Post-Processing:** confidence filter (>0.75), NMS (IoU < 0.4)  
5. **Tracking:** Kalman-based temporal consistency  

---

### **7.3 Dataset and Augmentation**

| Set | Images | Percentage |
|------|---------|------------|
| Train | 16,000 | 72% |
| Validation | 4,000 | 18% |
| Test | 2,000 | 10% |

Augmentations include Gaussian noise, hue/saturation jitter, perspective distortion, and motion blur.

---

### **7.4 Model Performance**

| Metric | Value | Notes |
|---------|--------|-------|
| mAP@0.5 | 92.3% | High precision |
| Precision | 0.90 | Stable |
| Recall | 0.88 | Reliable |
| FPS | 30 | Real-time |
| Power | <2W | Efficient |

---

### **7.5 Integration with Motion Subsystem**

3D localization:

$$
\begin{aligned}
\begin{bmatrix}
X_c \\ Y_c \\ Z_c
\end{bmatrix}
&= 
K^{-1}
\begin{bmatrix}
u \\ v \\ 1
\end{bmatrix} Z
\end{aligned}
$$

where  
\(K\) = intrinsic camera matrix,  
\((u, v)\) = pixel coordinates,  
and \(Z\) = depth (ultrasonic).

![Alt text](/zavier-page/assets/images/trash.webp)
### **7.6 Detection Loop (Pseudocode)**

```python
while True:
    frame = camera.read()
    pre = preprocess(frame)
    boxes, scores, labels = yolov5_lite(pre)
    filtered = nms_filter(boxes, scores, threshold=0.75)
    world_coords = project_to_world(filtered, depth_map)
    kalman.update(world_coords)
    target = kalman.predict()
    move_collector(target)
```

---

## **8. Algorithm and Pseudocode**

### **Main Zavier Bot Logic**

```python
# Initialize sensors, motors, and camera
while system_active:
    frame = capture_frame()
    detected_objects = YOLOv5(frame)
    
    if trash_detected:
        predicted_position = Kalman(trash_position)
        move_to(predicted_position)
        
        if intercept_failed:
            signal(SUPPORT_BOT)
    
    elif voice_command == "come here":
        move_to(user_location)
```

### **Support Bot Logic**
```python
while idle:
    if signal_from_main:
        navigate_to(location)
        activate_arm()
        collect_trash()
        return_to_dock()
```

---

## **9. Results and Discussion**

| **Metric**                   | **Value**      |
|------------------------------|----------------|
| Object Detection Accuracy    | 92.3 %         |
| Real-time Response Delay     | 45 – 60 ms     |
| Trash Intercept Success Rate | 81 %           |
| Docking Precision            | ± 3 cm         |
| Average Power Consumption    | 18 W           |

Zavier achieved reliable real-time detection and predictive interception despite low-cost hardware.


---

## **10. Limitations and Future Work**

- Hardware torque limitations.

- YOLO-lite struggles with overlapping objects.

- Planned upgrades: LIDAR-based SLAM, improved arm kinematics, and wireless charging.



---

## **11. Conclusion**

Zavier bridges the gap between convenience and cleanliness.
By merging embedded control, lightweight AI, and predictive motion algorithms, it transforms waste disposal into an effortless experience.
Its modular, scalable design shows how automation can solve small but deeply human problems — like neglect and laziness — through engineering.


---

## **12. References**

```
1. Redmon, J., & Farhadi, A. “YOLOv3: An Incremental Improvement.” arXiv:1804.02767


2. Welch, G., & Bishop, G. “An Introduction to the Kalman Filter.” UNC, 2006


3. Texas Instruments, “PID Control Theory and Implementation,” App Note SPRA089


4. Hugging Face Datasets, “TrashNet Waste Dataset,” 2024 Edition


5. STM32F411CEU6 Datasheet, STMicroelectronics, 2023
```
