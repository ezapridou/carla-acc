# Adaptive Cruise Control System (ACC) in the [CARLA Simulator](https://carla.readthedocs.io/en/latest/)
This repository contains the files that were extended in the CARLA Simulator in order to implement an ACC system. This system was used to experiment with the runtime verification approach proposed in "Runtime Verification of Autonomous Driving Systems in CARLA" (to be published in [RV2020](https://rv20.ait.ac.at/) by E. Zapridou, E. Bartocci, P. Katsaros).

## ACC extension
In the folder [ACC](https://github.com/ezapridou/carla-acc/tree/master/ACC) you can find the files that were extended to implement the ACC system. The paths of these files are the following:
```bash
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/WheeledVehicleAIController.h
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Vehicle/WheeledVehicleAIController.cpp
```

Additionally, the functionality of enabling/disabling the ACC system using the Python API was added. You can find the files that were extended to add this functionality in the folder [enable-disable ACC](https://github.com/ezapridou/carla-acc/tree/master/enable-disable%20ACC). Their paths are the following:
```bash
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.h
Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Server/CarlaServer.cpp
LibCarla/source/carla/client/detail/Client.h
LibCarla/source/carla/client/detail/Client.cpp
LibCarla/source/carla/client/detail/Simulator.h
```

If you have already built CARLA and wish to add this ACC extension, please replace the original CARLA files with the ones found in these two folders and build again the simulator and the Python API. 
```bash
make launch
make PythonAPI
```

## Runtime monitoring
The script "acc_runtime_monitoring.py" is used to implement the rtamt library in order to monitor the ACC system in runtime. The requirement to be checked by the monitor is set as a constant inside the script.
