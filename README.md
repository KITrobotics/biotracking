# biotracking
Biofeedback with opencv: Schoulder tracking

Just run: 
```
roslaunch biotracking biotracking.launch 
```
Parameters should be adjusted in:
```
config/parameters.yaml
```
Calling service for calculating average image:
```
rosservice call /cv_leg_tracking/calculateAvg
```
