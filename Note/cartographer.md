####Save map
> rosservice call /write_state ${HOME}/Downloads/map.bag.pbstream

####Covert ros fromat  map
> rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename=${HOME}/Downloads/map.bag.pbstream 
