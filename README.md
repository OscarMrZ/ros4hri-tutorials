# ROS4HRI tutorials

Welcome to the ROS2 humble tutorial for the ROS4HRI [framework](https://wiki.ros.org/hri). This tutorial walks you through the basics for setting up a person identification pipeline. Using this information, you'll be able to perform different human-robot interactions. All the nodes used in the tutorial are [REP-155](https://www.ros.org/reps/rep-0155.html) compliant. 

## HRI packages

* [libhri](https://github.com/ros4hri/libhri)
* [hri_msgs](https://github.com/ros4hri/hri_msgs)
* [hri_rviz](https://github.com/ros4hri/hri_rviz)
* [hri_face_detect](https://github.com/ros4hri/hri_face_detect)

## Prepare the environment

### Initial environment preparation

We'll use docker compose to launch a tutorial docker container based on this image (link). The image is based on ROS 2 humble and comes with all the necessary deps installed! 

```bash
xhost +local:docker
sudo apt install docker-compose
git clone https://github.com/OscarMrZ/ros4hri-tutorials.git
cd ros4hri-tutorials
docker compose up
```

We will work with different terminal windows. To connect to the running docker simply execute the following command in each of them. 

```
docker exec -it ws_container bash
```

## Face detection

### Start the usb_cam node

This node simply reads the input from the webcam and the [camera intrinsics](config/default_webcam_calibration.yml) and publishes the images under `image_raw` and the camera parameters under `camera_info`. 

```bash
ros2 run usb_cam usb_cam_node_exe &
```

As you may notice, this command will run in the background, avoiding the need for opening more terminals in the docker. That's completely optional though. 

### Start the face detection node

The [`hri_face_detect`](https://github.com/ros4hri/hri_face_detect) package performs fast face detection using YuNet face detector and Mediapipe Face Mesh. This node publishes under the `/humans/faces/<faceID>/` topic different info about the detected face, such as the roi.
Also, it will publish the list of tracked faces be published under the `/humans/faces/tracked`

Importantly, this ID is not persistent: once a face is lost (for instance, the person goes out of frame), its ID is not valid nor meaningful anymore. To cater for a broad range of applications (where re-identification might not be always necessary), there is no expectation that the face detector will attempt to recognise the face and re-assign the same face ID if the person reappears.

There is a one-to-one relationship between this face ID and the estimated 6D pose of the head. The node publishes a head pose estimation with a TF frame named `face_<faceID>`. 

```bash
ros2 launch hri_face_detect face_detect.launch.py
```

You should immediately see on the console that some faces are indeed detected. Let's visualise them.

#### Visualise the result

We can check that the faces are detected and published at ROS message by simply typing:

```
ros2 topic echo /humans/faces/tracked
```

We can also use `rviz2` to display the faces with the facial landmarks.
Then, in `rviz2`, set the fixed frame to `head_camera`, and enable the `Humans` and TF plugins:

![rviz human plugin](images/rviz-humans-plugin.png)

Configure the `Humans` plugin to use the `/usb_cam/image_raw` topic. You should see the face being displayed with the landmarks. Also, set up the `TF` plugin in order to see the face position in 3D. 

![rviz displaying faces](images/rviz-faces.png)

We are effectively running the face detector, extracting features and 3D position in a Docker container, no GPU needed! 

## Body detection

First, open yet another terminal connected to the docker. 

### Start the body detection


```bash

```

Start the body detector:

```
roslaunch hri_fullbody hri_fullbody.launch rgb_camera:=usb_cam
```

Re-open the browser tab with `rviz`: you should now see the skeleton being
detected, in addition to the face:

![Body and face, visualised in rviz](images/body-face.png)

## 'Assembling' full persons

Now that we have a face and a body, we can build a 'full' person.

![ROS4HRI IDs](images/ros4hri-ids.png)

Until now, we were running two ROS4HRI perception module: `hri_face_detect` and
`hri_fullbody`.

The face detector is assigning a unique identifier to each face that it
detects (and since it only *detects* faces, but does not *recognise* them, a
new identifier might get assigned to the same actual face if it disappears and
reappears later); the body detector is doing the same thing for bodies.

Next, we are going to run a node dedicated to managing full *persons*. Persons
are also assigned an identifier, but the person identifier is meant to be permanent.

First, to avoid generating too many new people, we are going to only publish the
few same frames from the video. Switch back to your `rosbag` terminal. Stop the
current bag (Ctrl+C), and run:

```
rosbag play --loop --clock -s 3 -u 1 severin-sitting-table.bag
```

Then, open a new terminal and install `hri_person_manager`:

```
cd ws/src
git clone https://github.com/ros4hri/hri_person_manager.git
cd ..
catkin build hri_person_manager
```

Source again `install/setup.bash`, configure some general parameters (needed
because we are using a webcam, not an actual robot, [check the doc](https://github.com/ros4hri/hri_person_manager?tab=readme-ov-file#hri_person_manager) to know more), and start
`hri_person_manager`:

```
source install/setup.bash
rosparam set /humans/reference_frame head_camera
rosparam set /humans/robot_reference_frame head_camera
rosrun hri_person_manager hri_person_manager
```

If the face and body detector are still running, you might see that
`hri_person_manager` is already creating some *anonymous* persons: the node
knows that some persons must exist (since faces and bodies are detected), but it
does not know *who* these persons are (you can ignore the warning regarding TF
frames: they come from the use of bag files instead of real 'live' data).

To get 'real' people, we need a node able to match for instance a *face* to a unique and
stable *person*: a face identification node.

### Display the person feature graph

We can use a small utility tool to display what the person manager understand of
the current situation.

Open a new terminal and run:

```
source /opt/ros/noetic/setup.bash
cd ws/src/hri_person_manager/scripts/
./show_humans_graph.py
```

In a different terminal, run:

```
 evince /tmp/graph.pdf
 ```

You should see a graph similar to:

![ROS4HRI graph](images/ros4hri-graph.png)

### Connecting the person feature graph

First, let's manually tell `hri_person_manager` that the face and body are
indeed parts of the same person. TO do so, we need to publish a *match* between
the two ids (in this example, `rlkas` (the face) and `mnavu` (the body), but
your IDs might be different, as they are randomly chosen)

In a new terminal (with ROS sourced):

```
rostopic pub /humans/candidate_matches hri_msgs/IdsMatch "{id1: 'rlkas', id1_type: 2, id2: 'mnavu', id2_type: 3, confidence: 0.9}"
```

The graph updates to:

![ROS4HRI graph](images/ros4hri-graph-2.png)

> ⚠️  do not forget to change the face and body IDs to match the ones in your system!

> 💡 the values `2` and `3` correspond respectively to a face and a body. See
> [hri_msgs/IdsMatch](https://github.com/ros4hri/hri_msgs/blob/master/msg/IdsMatch.msg)
> for the list of constants.

### Manually identifying the person

To turn our *anonymous* person into a known person, we need to match the face ID
(or the body ID) to a person ID:

For instance:

```
rostopic pub /humans/candidate_matches hri_msgs/IdsMatch "{id1: 'rlkas', id1_type: 2, id2: 'severin', id2_type: 0, confidence: 0.9}"
```

The graph updates to:

![ROS4HRI graph](images/ros4hri-graph-3.png)

Know that the person is 'known' (ie, at least one person 'part' is associated to
a person ID)m the automatically-generated 'anonymous' person is replaced by the
actual person.

We are doing it manually here, but in practice, we want to do it automatically.

### Installing and running automatic face identification

Let's install the `hri_face_identification` node:

```
cd ws/src
git clone https://github.com/ros4hri/hri_face_identification.git
cd ..
```


Then, build it:

```
catkin build hri_face_identification
```

> 💡 again, all the dependencies are already installed in the container. To do
> it manually, run `rosdep install -r -y --from-paths src`.


Start the node:

```
source install/setup.bash
roslaunch hri_face_identification face_identification.launch
```

You can now check in the graph (or directly on the `/humans/candidate_matches`
topic): the face should now be automatically associated to a person.

### Probabilistic feature matching

The algorithm used by `hri_person_manager` exploits the probabilities of *match*
between each and all personal features perceived by the robot to find the most
likely set of *partitions* of features into persons.

For instance, from the following graph, try to guess which are the most likely
'person' associations:

![complex ROS4HRI graph](images/ex1.png)

Response in the paper (along with the exact algorithm!): [the 'Mr Potato' paper](https://academia.skadge.org/publis/lemaignan2024probabilistic.pdf).

## If you want more...!

Here a few additional tasks you might want to try, to further explore ROS4HRI:

- Write a small Python script that list on the console the people around the
  robot ([hint!](https://www.phind.com/search?cache=rhu3n4zmjwshfp0h3vp29b9w)).

- write a node (C++ or Python) to automatically match faces and bodies. One
  approach consists in computing the overlap of the regions of interest of pairs
  of (face, body), and compute a likelihood based on that.

  Check the [`pyhri` API documentation](https://pyhri.readthedocs.io/en/latest/)
  here, and the [C++ `libhri` API
  documentation](http://docs.ros.org/en/noetic/api/hri/html/c++/) here.
