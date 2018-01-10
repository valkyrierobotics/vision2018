# Valkyrie Robotics's Computer Vision Code for 2018.
This release will only have auto yaw alignment.
Replace /etc/rc.local with utilities/rc.local to prepare the camera.
This requires OpenCV 2.4.x, Protobuf 2.5.0 and Gnuplot with wxt.

### Active Contributors
- Min Hoo Lee
- Pravin Suranthiran
- Sahas Munamala

### Sample run commands

#### Main vision
` make main `  

#### MJPG Streamer that streams video from main vision if STREAM is enabled
` make mjpg_streamer_instance `  

#### Use gnuplot to show FPS in real time (change ARGS to "-h" for help)
` make gnuplot_fps PLOT_FPS_ARGS="-c 1 -n 60 -m 30"`  

#### Use gnuplot to show main vision data in real time (change ARGS to "-h" for help)
` make gnuplot_vision `  
` make gnuplot_vision PLOT_VISION_ARGS="-c 4 -n 30 -m 60"`  

#### Calibrate camera using OpenCV calibration code
` make camera_calib `  

### Contributors
- Min Hoo Lee
- Michael Wan
- Rahul Amara
- Pravin Suranthiran
- Jeremy Tien
- Sahas Munamala
- Marcus Plutowski
- Lee Mracek
