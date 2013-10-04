Bridge between hpp and ros

  Implement a robot state publisher that takes as input
    - a list of joint names,
    - a configuration vector,
  and that publishes a rigid transformation in tf corresponding to the first 6
  variables and a robot joint state corresponding to the remaining variables.

  See doc/script.py for instruction of use with hrp2_14.
