#!/bin/bash


TAG=neoware-0.1.0
wget https://github.com/tyuownu/cartographer/archive/${TAG}.zip \
    -O cartographer.zip

wget https://github.com/tyuownu/cartographer_ros/archive/${TAG}.zip \
    -O cartographer_ros.zip

wget https://github.com/tyuownu/cartographer_turtlebot/archive/${TAG}.zip \
    -O cartographer_turtlebot.zip

wget https://github.com/tyuownu/ceres-solver/archive/${TAG}.zip \
    -O ceres-solver.zip

wget https://github.com/tyuownu/stage_ros/archive/${TAG}.zip \
    -O stage_ros.zip

#wget https://github.com/tyuownu/navigation/archive/${TAG}.zip \
#    -O navigation.zip

unzip -n cartographer.zip -d ../../../
unzip -n cartographer_ros.zip -d ../../../
unzip -n cartographer_turtlebot.zip -d ../../../
unzip -n ceres-solver.zip -d ../../../
unzip -n stage_ros.zip -d ../../../
# unzip -n navigation.zip -d ../../../

cd ../../../

mv cartographer-neoware-0.1.0 cartographer
mv cartographer_ros-neoware-0.1.0 cartographer_ros
mv cartographer_turtlebot-neoware-0.1.0 cartographer_turtlebot
mv ceres-solver-neoware-0.1.0 ceres-solver
mv stage_ros-neoware-0.1.0 stage_ros
# mv navigation-neoware-0.1.0 navigation
