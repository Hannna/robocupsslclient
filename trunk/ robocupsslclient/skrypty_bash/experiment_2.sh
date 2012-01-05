#!/bin/bash
SSL_REFBOX_PATH = "/home/maciek/Pulpit/magisterka/sslrefbox-2010.2"

ulimit -c unlimited
killall -9 gazebo
sleep 1
killall -9 sslrefbox


sleep 1

echo "start "
	
TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/test_worlds/test_rrt_world.world"
	
sit="empty"
param="experiment_2"

if [ "$1" = "dynamic" ] ; then
	sit="/home/maciek/workspace/magisterka/sytuacje/dynamiczne.txt"
elif [ "$1" = "static" ] ; then
	sit="/home/maciek/workspace/magisterka/sytuacje/worlds.txt"
else
	sit="dupa"	
fi
	

for i in $(seq 1 1 20)
do
	echo $i
	echo "creating world"
	echo $TEST_WORLD
	gazebo -g $TEST_WORLD  | tee  gazebo_logi &
	#gazebo $TEST_WORLD  | tee  gazebo_logi &	
	sleep 5
	./magisterka ${param} $sit | tee run.log
	echo "exit from magisterka"
	mv tmp.txt results/tmp$i.txt
	killall -9 gazebo
	sleep 1
	killall -9 sslrefbox
done
	
	
