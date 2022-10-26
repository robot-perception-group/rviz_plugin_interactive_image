rviz plugin for interactive images
===========================================================

This repository provides an rviz plugin to display a 2D image or video in RVIZ and interact with it.

Visually, the plugin is virtually identical to the rviz default ImageDispay plugin and supports `sensor_msgs/Image` `sensor_msgs/CompressedImage` and `theora_image_tranbsport/Packet` ROS messages, which are displayed in a 2D display widget.

In addition to the `Image Topic`, an additional `Point Topic` can be configured for each instance. If set, a mouse interaction with the widget will publish a `geometry_msgs/Point` message with the normalized image coordinate in its x and y coordinates. The z coordinate encodes the type of interaction (see below)

Each instance has 4 boolean configuration options.

1. 'React to Mouse Clicks' - this publishes a point coordinate message on every single click when the mouse button is first pressed. Responds to a double click as a single click only. - code `1`

2. 'React to Mouse Release' - this publishes a point coordinate message every time the mouse button is released. A doubleclick produces two of these. - code `2`

3. 'React to Mouse Doubleclick' - this publishes a point coordinate message once, when a doubleclick has been detected. - code `3`

4. 'React to Mouse Move' - this publishes a point coordinate message whenever the mouse is moved over the widget. - code `0`

If one or multiple buttons are pressed, the respective button code (see https://doc.qt.io/qt-5/qt.html#MouseButton-enum) is left shifted by two bits and added to the type, then stored in the z coordinate.

Examples:

- When reacting to mouse clicks, a single left click (QtMouseButton 1) is left shifted 2 bits (=4) and added to type 1, so z coordinate is `5.0`
- When reacting to all events, a drag event with the right mouse button (QtMouseButton 2) is left shifted 2 bits (=8) leading to the following sequence:
```
    z=0 (movement with no presses)
    ...
    z=9 (click)
    z=8 ( movement with button 2 pressed)
    ...
    z=2 ( release, no button pressed)
```


<h2>Getting started with rviz_plugin_interactive_image</h2>

<h3>To build this plugin:</h3>

Checkout the master into a catkin workspace and run catkin_make

<h4>To test this plugin:</h4>

```
roscore
rosrun rviz rviz
```
Add new rviz display of type `InteractiveImage`

Select the `Point Topic` to `/testpoint`
```
rostopic echo /testpoint
```
Click anywhere on the image placeholder


