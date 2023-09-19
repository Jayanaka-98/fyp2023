# FYP_2023
Final Year Project 2023 - Object Classification and Haptic Amplifier based on Surface Texture

**-- INTRODUCTION**

There are five different sensory mechanisms in the grand design of the human being. Namely optical vision, hearing, smell detection, taste detection and touch sensing. In each one of these sensing methods, there are multiple components that makes up the final feeling. As an example, vision sensing consists of color, brightness, resolution, and depth measurement. Likewise, in touch sensing there are multiple components that makes up the sense of touch such as, stickiness, roughness, friction, warmth etc. 

The sense of touch  which us also referred to as haptic sensation is much harder to record and reproduce relative to the auditory and visual senses. The mechanism of feeling the haptic sensation of any object is by making direct contact with any part of the skin. But to feel some surface thoroughly humans tend to apply a force. This force is countered by a reaction force which then deforms the skin. This stimulates mechanoreceptors \citep{serhat2021free} within the skin which is what we feel as haptic sensation. However, this haptic sensation will not be equal for all humans as human skin loses the ability to perceive the sense of touch with age. Therefore, even though the human sense of touch is very advanced, we are sometimes incapable of distinguishing extremely smooth surfaces. However, these surfaces also have minute surface texture irregularities which are beyond the capturing range of a human.

<img src="https://user-images.githubusercontent.com/111507682/268958278-02fb806a-402a-45e4-94f9-3619df62773e.png" alt="GitHub Logo" width="450" height="500">


**-- PROBLEM STATEMENT**

There are requirements to identify minute surface texture irregularities and classify objects based on surface texture data. 


**-- OBJECTIVES**

1. HAPTIC AMPLIFIER:

As the haptic amplifier, a non-identical master replica system that is capable of simultaneous force-position response amplification was developed. Two linear actuators were designed—one to interact with a human hand and the other to interact with the surface. The robust control was realized with the disturbance observer while a sensorless sensing approach was employed to estimate the reaction force experienced by the motors.

<img src="https://user-images.githubusercontent.com/111507682/268957386-f285f55e-6015-413b-a753-3baadba2d105.png" alt="GitHub Logo" width="500" height="250">

2. OBJECT CLASSIFICATION:

This study delves into the utilization of machine learning techniques to classify objects based on the haptic perception of surface textures. The haptic feedback from various surfaces was captured using the replica actuator used in the haptic amplifier to record position and force variation of the surface texture. Notably, the force data was estimated without relying on traditional force sensors, employing the sensorless sensing approach instead. Using the gathered data for five different surface classes, feature extraction was done and feature correlation was identified.

<img src="https://user-images.githubusercontent.com/111507682/268955402-4af6e72a-96f6-4cc1-ac6e-0b8a63c2dd85.png" alt="GitHub Logo" width="500" height="500">



**-- EXPERIMENTS AND RESULTS**

Three different controllers; position controller, force controller, and an acceleration based scaling bilateral controller were tested for the haptic amplifier, and the scaling bilateral controller with mass normalization exhibited the highest capability in reproducing surface texture. This controller was recommended for achieving haptic reproduction, allowing operators to perceive intricate surface details otherwise unnoticeable. The haptic amplifier also revealed minor imperfections in the fabricated test surfaces that were more perceptible through the amplifier than when touched directly by hand.

Using selected features after gathering data, two machine learning models were employed: a Random Forest (RF) classifier and a Convolutional Neural Network (CNN). Training these models revealed that the CNN outperformed the RF model in terms of both accuracy and speed during testing. However, it's worth noting that the CNN model demanded substantial computational resources for training. Despite this computational cost, the CNN algorithm emerged as the most suitable machine learning model for this application.


**-- APPLICATIONS**

The haptic amplifier demonstrates versatility in various applications, including remote haptic perception of fabrics, ensuring secure haptic feedback in hazardous environments, and assisting individuals with impaired tactile senses in augmenting their ability to perceive touch. Also, the results indicated that surfaces could be classified using machine learning algorithms based solely on position and reaction force data. 





