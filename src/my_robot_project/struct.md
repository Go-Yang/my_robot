/my_robot
├── my_robot_msgs
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg
│       └── MyCustomMessage.msg
└── my_robot_project
    ├── CMakeLists.txt
    ├── package.xml
    ├── lib
    │   ├── robot_control_lib
    │   │   ├── CMakeLists.txt
    │   │   ├── package.xml
    │   │   ├── src
    │   │   │   └── control.cpp
    │   │   └── include
    │   │       └── robot_control
    │   │           └── control.h
    │   ├── sensor_interface_lib
    │   │   ├── CMakeLists.txt
    │   │   ├── package.xml
    │   │   ├── src
    │   │   │   └── sensor.cpp
    │   │   └── include
    │   │       └── sensor_interface
    │   │           └── sensor.h
    │   ├── mapping_lib
    │   │   ├── CMakeLists.txt
    │   │   ├── package.xml
    │   │   ├── src
    │   │   │   └── mapping.cpp
    │   │   └── include
    │   │       └── mapping
    │   │           └── mapping.h
    │   └── navigation_lib
    │       ├── CMakeLists.txt
    │       ├── package.xml
    │       ├── src
    │       │   └── navigation.cpp
    │       └── include
    │           └── navigation
    │               └── navigation.h
    └── src
        └── main.cpp
