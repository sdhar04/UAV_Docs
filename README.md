# UAV_Docs

## Setup
Follow the guide given below, but first:
Note that the version of ardupilot you have to clone into your system has changed. Use this: [https://github.com/sdhar04/Agri_Loiter-Stable](https://github.com/sdhar04/Argi_Loiter-Stable)
[guide](https://docs.google.com/document/d/1ihAxgX1y3yRMqRnX1yWfk9WDaxZt8JmtFyEddi13SWw/edit?usp=sharing)

## Algorithm
This algorithm has been tested on 2D LiDAR data in simulation. The next goal is to make it work with 3D data, which may be coming from LiDAR or RaDAR sensors. 
The main idea is to take heights on distances of obstacles in front of and below the drone. We have to do a few angle corrections as the drone will not always stay perfectly level.
Once we get arrays from both LiDARs, we concatenate them and find heights and distances of all detections. 
The heights are multiplied with a weight that depends on the drone's current forward velocity. Higher velocity means farther obstacles are given higher priority. Initially a 1/v dependence was used, but it was later changed to ln(k.v).
Out of these weighted heights, the most notable one is taken to publish a target height, with an offset of 3m.

The equation used was
+ self.k1*math.log(self.k2*self.f_vel))*math.pow(x-d-3,3))+1)*h_values[i]


In the PID loop used, Kp has been made dependent on velocity and distance:
+ self.Kp = 0.6*math.exp(self.f_vel)/self.fard

Video of its application:


https://github.com/cintlib/UAV_Docs/assets/154340547/cc1087fa-0cc8-40bd-a08e-8b21ba48c460



A potential field function has also been used to limit horizontal velocity in case of very tall obstacle, so that the drone gets enough time to complete its ascent.

## New mode
The custom mode can be invoked inside SITL using `mode 29`.
In this mode, te pilot has zero control over the z-axis of the drone, thrust is computed by the companion board. When no input is given, the drone keeps its current position. 
To run that, refer to [this repo](https://github.com/neeraj12321/Ardupilot_Firmaware)
