# Adaptive Cruise Control System (ACC) in the [CARLA Simulator](https://carla.readthedocs.io/en/latest/)
This repository contains the extended files to implement an ACC system in the CARLA Simulator. This system was used in order to experiment with the runtime verification approach proposed in "Runtime Verification of Autonomous Driving Systems in CARLA" published in [RV2020](https://rv20.ait.ac.at/) by E. Zapridou, E. Bartocci, P. Katsaros.

## ACC extension
In the folder "source code files" you can find the files of the CARLA Simulator that were extended to implement the ACC system. If you have already built CARLA and wish to add this extension, please replace the original CARLA files with the ones found in this folder and build again the simulator and the Python API. 
```bash
make launch
make PythonAPI
```

The paths of the files that were extended to implement the ACC system are the following:
```bash
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/WheeledVehicleAIController.h
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/WheeledVehicleAIController.cpp
```

The following files were extended to add the functionality of enabling/disabling the ACC system using the Python API:
```bash
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.h
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp
LibCarla/source/carla/client/detail/Client.h
LibCarla/source/carla/client/detail/Client.cpp
LibCarla/source/carla/client/detail/Simulator.h
```

## Runtime monitoring
The script "acc_runtime_monitoring.py" is used to implement the rtamt library in order to monitor the ACC system in runtime. The requirement to be checked by the monitor is set as a constant inside the script.
