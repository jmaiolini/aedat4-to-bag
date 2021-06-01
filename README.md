# aedat4-to-bag
A simple script to convert event camera streams from aedat4 format to ROS bag

* Build the custom ROS message to hold triggers
 `catkin build triggers`

* Run the script (not a ROS node)
 `python3 converter.py <aedat4_filename>`
