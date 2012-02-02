/*
 * SimAnnealing.cpp
 *
 *  Created on: Jan 15, 2012
 *      Author: maciek
 */

#include "SimAnnealing2.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <gsl/gsl_siman.h>
#include "../Config/Config.h"

#include "../Vector2d/Vector2D.h"

/* set up parameters for this simulated annealing run */

/* how many points do we try before stepping */
#define N_TRIES 200

/* how many iterations for each T? */
#define ITERS_FIXED_T 10

/* max step size in random walk */
#define STEP_SIZE 1.0

/* Boltzmann constant */
#define K 1.0

/* initial temperature */
#define T_INITIAL 0.8

/* damping factor for temperature */
#define MU_T 1.05
#define T_MIN 0.06

typedef
struct GsimanParams2_{
	Vector2D answer;
	double angleToShoot;
	const EvaluationModule * evaluation;
	const GameStatePtr* gameState;
	const std::string* robotName;
	Vector2D* obstacles;
	int obs_size;
}GsimanParams2;

/* now some functions to test in one dimension */
double E2(void *xp)
{
	GsimanParams2* gsimanParams = (GsimanParams2*)xp;
	for(int ii =0;ii < gsimanParams->obs_size;ii++){
		if(  pow(gsimanParams->answer.x -gsimanParams->obstacles[ii].x,2) + pow(gsimanParams->answer.y -gsimanParams->obstacles[ii].y,2) <= pow(Config::getInstance().getRRTRobotRadius(),2) ){
			return std::numeric_limits<double>::max() ;
		}
	}


	Vector2D v(0,0);
	Pose pos(gsimanParams->answer,0);
	(*gsimanParams->gameState)->updateRobotData(*gsimanParams->robotName,pos,v,0);
	double angleToShoot;
	double score = 0;
	std::pair<double, double> ang =gsimanParams->evaluation->aimAtGoal( (*gsimanParams->gameState),*gsimanParams->robotName, angleToShoot,score);

	gsimanParams->angleToShoot = angleToShoot;
	return -score;
}

double M2(void *xp, void *yp)
{
	GsimanParams2* gsimanParamsX = (GsimanParams2*)xp;
	GsimanParams2* gsimanParamsY = (GsimanParams2*)yp;

	return gsimanParamsX->answer.distance(gsimanParamsY->answer);
}

void S2(const gsl_rng * r, void *xp, double step_size)
{
	GsimanParams2* gsimanParams = (GsimanParams2*)xp;
	do{

		//double	t=	gsl_rng_uniform_int(r,472);
		//double u = t/100.0;
		//gsimanParams->answer.x = u;//u * 2 * step_size - step_size + gsimanParams->answer.x;
		//t=	gsl_rng_uniform_int(r,672); //gsl_rng_uniform_pos(r);
		//u = t/100.0;
		//gsimanParams->answer.y = u ;

		double u = gsl_rng_uniform_pos(r);;
		gsimanParams->answer.x = u * 2 * step_size - step_size + gsimanParams->answer.x;
		u = gsl_rng_uniform_pos(r);
		gsimanParams->answer.y = u* 2 * step_size - step_size + gsimanParams->answer.y;
	}while( gsimanParams->answer.x<0.675 || gsimanParams->answer.y<0.675 );
}

void P2(void *xp)
{
	GsimanParams2* gsimanParams = (GsimanParams2*)xp;
  printf ("x %f  y %f", gsimanParams->answer.x ,gsimanParams->answer.y);
}

SimAnnealing2::SimAnnealing2(const GameStatePtr& gamestate_,  const std::string robotName_,Robot::robotID rid_):
		gameState(gamestate_), robotName( robotName_ ), evaluation( EvaluationModule::getInstance() ),rid(rid_)  {



}

SimAnnealing2::~SimAnnealing2() {
	// TODO Auto-generated destructor stub
}


std::pair<Vector2D, double> SimAnnealing2::simAnnnealing2(){
	const gsl_rng_type * T;
	gsl_rng * r;

	gsl_siman_params_t params
	  = {N_TRIES, ITERS_FIXED_T, STEP_SIZE,
	     K, T_INITIAL, MU_T, T_MIN};

	T = gsl_rng_default;
	r = gsl_rng_alloc(T);

	//inicjacja generatora
	gsl_rng_env_setup();

	GsimanParams2 initial_state;
	initial_state.angleToShoot = 0;
	initial_state.robotName = &this->robotName;
	initial_state.evaluation = &this->evaluation;
	initial_state.gameState = &this->gameState;
	//initial_state.target = this->goalPose;
	initial_state.answer = this->gameState->getRobotPos(this->rid).getPosition();
	//initial_state.obstacles[0] =  Vector2D(1,1) ;
	std::vector<Pose> obs = this->gameState->getEnemyRobotsPos( this->rid );
	std::vector<Pose>::iterator ii = obs.begin();
	initial_state.obs_size = obs.size();
	initial_state.obstacles = new Vector2D [initial_state.obs_size];

	for(int i=0;ii!=obs.end();ii++, i++){
		initial_state.obstacles[i] = ii->getPosition();
	}


	gsl_siman_solve(r, &initial_state,E2,S2,M2,NULL,//P2
					NULL, NULL, NULL,
					sizeof(GsimanParams2), params);

	gsl_rng_free (r);

	return std::pair<Vector2D, double>(initial_state.answer,initial_state.angleToShoot);
	//std::cout<<" result "<<initial_state.answer<<std::endl;
}
