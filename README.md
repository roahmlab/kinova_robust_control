# kinova_robust_control

This repository hosts C++ implementation of the robust controller introduced in [ARMOUR](https://roahmlab.github.io/armour/) on [Kinova-gen3](https://www.kinovarobotics.com/product/gen3-robots) hardware.

This repository not only integrates the default position control from Kinova, but also implements torque control for **tracking a series of continuous trajectory** robustly under **model uncertainties**.

## Structure
The functions in kinova_robust_control codebase are organized into several folders.
- `dynamics`: Core controller implementations
- `experiments`: Example scripts for testing controllers
- `kortex`: Interface with Kinova robots
- `customized_msgs`: ROS2 message and service definitions
- `system`: System architecture and block-based design
- `trajectories`: Trajectory computation and management
- `utils`: Utility functions and logging

## Getting Started

### Install
We provide a detailed documentation on how to install the code in [installation/README.md](installation/README.md).

### Before using this codebase:
1. Due to limited time and resources, we were not able to make this codebase fully user-friendly. We strongly recommend carefully reviewing all available documentation, especially the ros2 Python scripts in `experiments/`, to fully understand how the controller operates.
2. You should make sure that your Kinova arm is equipped with an **emergency stop button**, so you can immediately shut it down in case of unexpected behavior.
3. You should also make sure that there are no obstacles around the arm when you run the examples in `experiments/`.

### Read the following READMEs in sequence:
1. Read [dynamics/README.md](dynamics/README.md) to learn how to formulate a proper trajectory message. The controller will not execute the trajectory if the trajectory is invalid.
2. Read [trajectories/README.md](trajectories/README.md) to learn how to formulate a proper trajectory message. The controller will not execute the trajectory if the trajectory is invalid.
3. Read [kortex/README.md](kortex/README.md) to learn more about the controller instance and related parameters.
4. Read [experiments/README.md](experiments/README.md) for examples that send trajectory messages to the controller instance and move the robot along the desired trajectories.

### Other stuff for customizing the functions:
1. Read [customized_msgs/README.md](customized_msgs/README.md) for definitions of all ros2 messages if you want to customize your own messages.

## Acknowlgement

The repository relies on [KINOVA® KORTEX™ API Reference](https://github.com/Kinovarobotics/kortex) developed and maintained by [Kinova Robotics](https://www.kinovarobotics.com/). 
We gratefully acknowledge their work and contribution to the open-source robotics community.

## Authors

This work is developed in [ROAHM Lab](https://www.roahmlab.com/). 

[Bohao Zhang](https://cfather.github.io/) (jimzhang@umich.edu): **Current maintainer**, Robust controller implementation in C++.

[Jonathan Michaux](https://jonmichaux.com/) (jmichaux@umich.edu): Robust controller theory developer.

Patrick D. Holmes (pdholmes@umich.edu): Robust controller theory developer.

Che Chen (cctom@umich.edu): Original creator and maintainer of the repository.

Zichang Zhou (zhouzichang1234@gmail.com): Implemented other controller comparisons.

## License

`ARMOUR` is released under a [3-clause BSD license](LICENSE). 
For a closed-source version of `ARMOUR` for commercial purpose, please contact the authors. 

An overview of the theoretical and implementation details has been published in [arxiv](https://arxiv.org/abs/2301.13308). 
If you use `ARMOUR` in an academic work, please cite using the following BibTex entry:
```bibtex
@article{michaux2023can,
  title={Can't Touch This: Real-Time, Safe Motion Planning and Control for Manipulators Under Uncertainty},
  author={Michaux, Jonathan and Holmes, Patrick and Zhang, Bohao and Chen, Che and Wang, Baiyue and Sahgal, Shrey and Zhang, Tiancheng and Dey, Sidhartha and Kousik, Shreyas and Vasudevan, Ram},
  journal={arXiv preprint arXiv:2301.13308},
  year={2023}
}
```

## Rules
If you have any questions or suggestions, please raise them in [Issues](https://github.com/roahmlab/kinova_robust_control/issues).
We will get back to you as soon as possible.