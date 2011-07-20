/*
 * TestRRT.cpp
 *
 *  Created on: 2010-11-03
 *      Author: maciek
 */

#include "TestRRT.h"
#include "../Test/Test.h"
#include "../Logger/Logger.h"
#include "../RRT/RRTPlanner.h"
#include "../Task/GoToPose.h"


const double maxXvalue = ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.x - Config::getInstance().field.FIELD_MARIGIN );
const double minXvalue = ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.x + Config::getInstance().field.FIELD_MARIGIN );
const double maxYvalue = ( Config::getInstance().field.FIELD_TOP_RIGHT_CORNER.y - Config::getInstance().field.FIELD_MARIGIN );
const double minYvalue = ( Config::getInstance().field.FIELD_BOTTOM_LEFT_CORNER.y + Config::getInstance().field.FIELD_MARIGIN );

Pose getRandomPose(){

	//generator wspolrzednej X
	static boost::mt19937 rngX(static_cast<unsigned> (time(NULL)));
	//generator wspolrzednej Y
	static boost::mt19937 rngY(static_cast<unsigned> (2*time(NULL)));

	//interesujace sa jedynie pozycje rozniace sie co najwyzej o 0.01 [m]
	static boost::uniform_int<int> uni_distX( ceil(minXvalue*10), floor(maxXvalue*10) );

	static boost::variate_generator<boost::mt19937, boost::uniform_int<int> >
		genX(rngX, uni_distX);

	//interesujace sa jedynie pozycje rozniace sie co najwyzej o 0.01 [m]
	static boost::uniform_int<int> uni_distY( ceil(minYvalue*10), floor(maxYvalue*10) );

	static boost::variate_generator<boost::mt19937, boost::uniform_int<int> >
		genY(rngY, uni_distY);

	double x=genX();
	double y=genY();

	Pose randomPose(x/10.0,y/10.0,0);

	return randomPose;
}

TestRRT::TestRRT(Robot* robot,Pose goalPose,bool serialize_ ): serialize(serialize_) {
	arg1.goalPose=goalPose;
	arg1.video=NULL;
	arg1.robot=robot;

	this->start(reinterpret_cast<void *>(&this->arg1));

}

void TestRRT::execute(void *args){

	GameStatePtr gameState(new GameState());
	threadArgPtr threadArg=reinterpret_cast<struct threadArg *>(args);
    Videoserver* video;
    Robot* robot;


    if(threadArg!=NULL){
        video=&Videoserver::getInstance();
        robot=threadArg->robot;
    }
    else
        return;

    log4cxx::LoggerPtr logger = getLoggerPtr( robot->getRobotName().c_str() );
    LOG_INFO(logger, "starting rrt test for "<<robot);

    while(true){
		video->updateGameState(gameState);
		double distanceToTarget;

		Pose goalPose = getRandomPose();

		//Pose goalPose;

		while( ( distanceToTarget = goalPose.distance(gameState->getRobotPos( robot->getRobotID() ) ) ) >
				   ( Config::getInstance().getRobotMainCylinderRadious() + 0.04 ) ){

			GoToPose goToPose( goalPose,robot);
			Task::status status = goToPose.execute(NULL);
			if( status == Task::collision){
				SimControl::getInstance().restart();
			    LOG_INFO(logger, "collision exit from test ");
				return;
			}
			else if ( status == Task::not_completed){
				video->updateGameState(gameState);
			}
			else break;
		};

		//   #ifdef DEBUG
		//       std::cout<<robot->getRobotName()<<
		//       " Mean update delta time=" <<(currSimTime-startSimTime)/step<<" max update delta time "<<max<<std::endl;
		//	#endif
	}

}

void TestRRT::joinThread(){
    this->join();
}
TestRRT::~TestRRT() {
	// TODO Auto-generated destructor stub
}
