
# run

Go2 Dokcing Station上で行う：

terminal 1: (service server)
```
$ source install/setup.bash
$ ros2 run apriltag_service service_server_gst
```

terminal 2: (service client)
```
$ source install/setup.bash
$ ros2 run apriltag_service service_client 
[INFO] [1709197060.057063466] [apriltag_client_simple]: AprilTag detected: ID 303 at [x: 0.082160, y: -0.026208, z: 0.515865, rotation: -0.157693]
```
