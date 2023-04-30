# ROS-Neuro feedback example (neurowheel)

The packages provides an example of the use of [libneurodraw](https://github.com/neurorobotics-iaslab/neurodraw) to create a wheel feedback for [**ROS-Neuro**](https://github.com/rosneuro) based applications.

[**ROS-Neuro**](https://github.com/rosneuro) and [libneurodraw](https://github.com/neurorobotics-iaslab/neurodraw) must be already installed.

The package represents **JUST** an example: optimization and paramterization can (**MUST**) be performed.

## Usage
The neurowheel accepts a [NeuroOutput](https://github.com/rosneuro_msgs) message and convert the *softprediction* in a rotation angle of the wheel. When the *ESCAPE* key is pressed the feedback closes. The package provides also an additional node to simulate the publication of [NeuroOutput](https://github.com/rosneuro_msgs) messages.

Run the neurowheel node:
```
rosrun rosneuro_feedback_example neurowheel
```

Run the neuroinput node:
```
rosrun rosneuro_feedback_example neuroinput
```
