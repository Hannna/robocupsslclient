/*
 * SimAnnealing.cpp
 *
 *  Created on: Jan 15, 2012
 *      Author: maciek
 */

#include "SimAnnealing.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <gsl/gsl_siman.h>
#include "../Config/Config.h"
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
#define MU_T 1.003
#define T_MIN 0.01


typedef
struct GsimanParams_{
	Vector2D* obstacles;
	int obs_size;
	Vector2D target;
	Vector2D answer;
}GsimanParams;

/* now some functions to test in one dimension */
double E1(void *xp)
{
  GsimanParams* gsimanParams = (GsimanParams*)xp;

  for(int ii =0;ii < gsimanParams->obs_size;ii++){
	if(  pow(gsimanParams->answer.x -gsimanParams->obstacles[ii].x,2) + pow(gsimanParams->answer.y -gsimanParams->obstacles[ii].y,2) <= pow(Config::getInstance().getRRTRobotRadius(),2) ){
			return std::numeric_limits<double>::max() ;
	}
  }

  return gsimanParams->target.distance(gsimanParams->answer);
}

double M1(void *xp, void *yp)
{
	GsimanParams* gsimanParamsX = (GsimanParams*)xp;
	GsimanParams* gsimanParamsY = (GsimanParams*)yp;

	return gsimanParamsX->answer.distance(gsimanParamsY->answer);
}

void S1(const gsl_rng * r, void *xp, double step_size)
{
	GsimanParams* gsimanParams = (GsimanParams*)xp;

	double u = gsl_rng_uniform(r);
	gsimanParams->answer.x = u * 2 * step_size - step_size + gsimanParams->answer.x;
	u = gsl_rng_uniform(r);
	gsimanParams->answer.y = u * 2 * step_size - step_size + gsimanParams->answer.y;

}

void P1(void *xp)
{
	GsimanParams* gsimanParams = (GsimanParams*)xp;
  printf ("x %f  y %f", gsimanParams->answer.x ,gsimanParams->answer.y);
}

SimAnnealing::SimAnnealing(const GameStatePtr& gamestate_, const Vector2D goalPose_,  Robot::robotID robotID__):
		gameState(gamestate_), goalPose(goalPose_), robotID_( robotID__ )  {
	// TODO Auto-generated constructor stub

}

SimAnnealing::~SimAnnealing() {
	// TODO Auto-generated destructor stub
}


Vector2D SimAnnealing::simAnnnealing(){
	const gsl_rng_type * T;
	gsl_rng * r;

	gsl_siman_params_t params
	  = {N_TRIES, ITERS_FIXED_T, STEP_SIZE,
	     K, T_INITIAL, MU_T, T_MIN};

	T = gsl_rng_default;
	r = gsl_rng_alloc(T);

	//inicjacja generatora
	gsl_rng_env_setup();

	GsimanParams initial_state;
	initial_state.target = this->goalPose;
	initial_state.answer = this->goalPose;
	//initial_state.obstacles[0] =  Vector2D(1,1) ;
	std::vector<Pose> obs = this->gameState->getEnemyRobotsPos(robotID_);
	std::vector<Pose>::iterator ii = obs.begin();
	initial_state.obs_size = obs.size();
	initial_state.obstacles = new Vector2D [initial_state.obs_size];

	for(int i=0;ii!=obs.end();ii++, i++){
		initial_state.obstacles[i] = ii->getPosition();
	}


	gsl_siman_solve(r, &initial_state,E1,S1,M1,NULL,
					NULL, NULL, NULL,
					sizeof(GsimanParams), params);

	gsl_rng_free (r);

	return initial_state.answer;
	//std::cout<<" result "<<initial_state.answer<<std::endl;
}
