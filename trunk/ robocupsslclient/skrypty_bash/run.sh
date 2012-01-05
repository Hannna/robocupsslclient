#!/bin/bash
SSL_REFBOX_PATH = "/home/maciek/Pulpit/magisterka/sslrefbox-2010.2"

ulimit -c unlimited
killall -9 gazebo
sleep 1
killall -9 sslrefbox

if [ "$1" = "--help" ];	then
	printf "enable options: \nprofile \ndebug \ntask \nrrt \nmultirrt \nvelocity \acc"
elif [ "$1" = "--kill" ];	then
	echo "kill gazebo"
	killall -9 gazebo
	sleep 1
	echo "kill magisterka"	
	killall -9 magisterka
elif [ "$1" = "--mem-check" ];	then	
	gazebo /home/maciek/mgr/mgr/plansza/test_task.world > /dev/null &	
	echo "start magisterka"
	valgrind --leak-check=yes --log-file=/home/maciek/codeblocks/magisterka/bin/Debug/valgrind.log /home/maciek/codeblocks/magisterka/bin/Debug/magisterka test ${string:2} 
	
elif [ "$1" = "--profile" ];	then
	PROF_FILE="$(pwd)/mgr.prof"
	PROF_PROG="$(pwd)/mgr"
	#export CPUPROFILE="${PROF_FILE} ${PROF_PROG}"   
	env CPUPROFILE=/home/maciek/workspace/mgr/Debug/mgr mgr.prof
	echo $CPUPROFILE 
	sleep 10
	echo "start magisterka"
	TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/test_task_new_gazebo.world"	
	echo "creating world"
	echo $TEST_WORLD
	gazebo $TEST_WORLD > /dev/null &
	./mgr test multirrt | tee run.log 
	

	#pprof --gv magisterka magisterka.prof
elif [ "$1" = "--debug" ];	then
	echo "start "
	echo $1
		
	TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/dribblerTest.world"	
	echo "creating world"
	echo $TEST_WORLD
	#arg = "play"
	gazebo $TEST_WORLD | tee  gazebo_logi &	
	/home/maciek/sslrefbox-2010.2/sslrefbox -C /home/maciek/sslrefbox-2010.2/referee.conf &
	sleep 5
	#./magisterka test $arg | tee run.log
	#gazebo /home/maciek/mgr/mgr/my_plansza/test_task.world > gazebo_logi &	
	#echo "start magisterka"
	#insight /home/maciek/codeblocks/magisterka/bin/Debug/magisterka test "$2" | tee run.log 
else
	string="$1"
	if [ "$1" = "--task" ] || [ "$1" = "--rrt" ] || [ "$1" = "--multirrt" ];	then	
		echo "start magisterka"
		TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/test_task_new_gazebo.world"	
		echo "creating world"
		echo $TEST_WORLD
		gazebo $TEST_WORLD > /dev/null &
		./magisterka test ${string:2} | tee run.log 
	
	elif [ "$1" = "--velocity" ] || [ "$1" = "--acc" ] || [ "$1" = "--motion" ]  || [ "$1" = "--rotation" ] ; then	
	#	echo "start velocity test"
		TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/test_velocity.world"		
		echo "creating world"
		echo $TEST_WORLD
		gazebo $TEST_WORLD > gazebo_logi &	
		./magisterka test ${string:2} | tee run.log 	
	
	elif [ "$1" = "--dribbler" ] ||  [ "$1" = "--kick" ] || [ "$1" = "--shoot" ] || [ "$1" = "--pass" ] || [ "$1" = "--refereeBox" ] || [ "$1" = "--play" ] ; then	
		
		#killall -9 magisterka
		sleep 1
		
		echo "start "
		echo $1
			
		TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/dribblerTest.world"	
		echo "creating world"
		echo $TEST_WORLD
		gazebo $TEST_WORLD | tee  gazebo_logi &	
		/home/maciek/sslrefbox-2010.2/sslrefbox -C /home/maciek/sslrefbox-2010.2/referee.conf &
		sleep 5
		 ./magisterka test ${string:2} | tee run.log
#		valgrind --leak-check=yes --log-file=/home/maciek/workspace/mgr/Debug/valgrind.log /home/maciek/workspace/mgr/Debug/mgr test ${string:2}  | tee run.log 
	elif [ "$1" = "--experiment_1" ] ; then	
		
		sleep 1
		
		echo "start "
		echo $1
			
		TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/test_worlds/test_world_1.world"	
		echo "creating world"
		echo $TEST_WORLD
		gazebo $TEST_WORLD | tee  gazebo_logi &	
		sleep 5
		 ./magisterka ${string:2} | tee run.log
	elif [ "$1" = "--experiment_2" ] ; then	
		
		sleep 1
		
		echo "start "
		echo $1
			
		TEST_WORLD="/home/maciek/mgr/mgr/my_plansza/test_worlds/test_rrt_world.world"
			
		sit="empty"
	
		if [ "$2" = "dynamic" ] ; then
			sit="/home/maciek/workspace/magisterka/sytuacje/dynamiczne.txt"
		elif [ "$2" = "static" ] ; then
			sit="/home/maciek/workspace/magisterka/sytuacje/worlds.txt"
		else
			sit="dupa"	
		fi
			
		
		echo "creating world"
		echo $TEST_WORLD
		gazebo $TEST_WORLD  | tee  gazebo_logi &	
		sleep 5
		 ./magisterka ${string:2} $sit | tee run.log
	else
	printf " \nusage: --kill \t--mem-check
	\n\t--profile \t--debug
	\n\t--task     \t--rrt 
	\n\t--multirrt \t--velocity
	\n\t--acc \t--dribbler
	\n\t--kick \t--refereeBox		
	\n\t--shoot \t--pass
	\n\t--rotation \t--play
	\n\t--experiment_1
	\n\t--experiment_2\n"
	fi
fi

	
