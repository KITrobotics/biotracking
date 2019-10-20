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
rosservice call /biotracking/calculateAvg
```
Shoulder positions are published as BioFeedbackMsg to:
```
/biotracking/biofeedback
```
