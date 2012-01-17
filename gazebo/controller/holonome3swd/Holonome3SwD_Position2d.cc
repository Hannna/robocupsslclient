/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Desc: Position2d controller for a Holonome3Sw drive with dribbler bar
 * for robocup.
 * Author: Kamil Muszynski, based on Holonome3SwD_Position2d.cc
 * Date:2-12-2009
 */

#include "XMLConfig.hh"
#include "Model.hh"
#include "Global.hh"
#include "Joint.hh"
#include "World.hh"
#include "Simulator.hh"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "Holonome3SwD_Position2d.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("holonome3swd_position2d", Holonome3SwD_Position2d);

enum {RIGHT, LEFT};

////////////////////////////////////////////////////////////////////////////////
// Constructor
  Holonome3SwD_Position2d::Holonome3SwD_Position2d(Entity *parent )
: Controller(parent)
{
  this->myParent = dynamic_cast<Model*>(this->parent);

  if (!this->myParent)
    gzthrow("Holonome3SwD_Position2d controller requires a Model as its parent");

  ResetData();

  ///init PID params
//  u = 0;
//  t = Simulator::Instance()->GetRealTime().Double();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Holonome3SwD_Position2d::~Holonome3SwD_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void Holonome3SwD_Position2d::LoadChild(XMLConfigNode *node)
{
  this->myIface = dynamic_cast<libgazebo::PositionIface*>(this->GetIface("position"));

  XMLConfigNode * startNode = node;

  // Get wheels child
  node = node->GetChild("wheels");
  if (!node)
    gzthrow("Holonome3SwD_Position2d controller requires a <wheels> node");

  // Get default params
  float distdef = node->GetFloat("distance", 0.0, 0);
  float radiusdef = node->GetFloat("radius", 0.0, 0);
  float maxtorquedef = node->GetFloat("torque", 0.0, 0);
  float betadef = node->GetFloat("beta", 0.0, 0);
  float gammadef = node->GetFloat("gamma", 0.0, 0);

  // Get params for each wheels
  for (size_t i = 0; i < 3; ++i)
  {
    std::ostringstream sw; sw << "swedish" << i;
    XMLConfigNode *lnode = node->GetChild(sw.str());
    if (!lnode)
      gzthrow("The controller couldn't get swedish wheels " + sw.str());

    std::string joint =  lnode->GetString("joint", "", 1);
    this->joint[i] = this->myParent->GetJoint(joint);

    if (!this->joint[i])
      gzthrow("The controller couldn't get hinge joint for " + sw.str());

    this->ALPHA[i] = lnode->GetFloat("alpha", 1000000, 1);
    if (this->ALPHA[i] == 1000000)
      gzthrow("The controller could't get alpha param for " + sw.str());

    this->DIST[i] = lnode->GetFloat("distance", distdef, 0);
    if (!this->DIST[i])
      gzthrow("The controller could't get distance param for " + sw.str());

    this->BETA[i] = lnode->GetFloat("beta", betadef, 0);

    this->GAMMA[i] = lnode->GetFloat("gamma", gammadef, 0);

    this->RADIUS[i] = lnode->GetFloat("radius", radiusdef, 0);
    if (!this->RADIUS[i])
      gzthrow("The controller could't get radius param for " + sw.str());

    this->MAXTORQUE[i] = lnode->GetFloat("torque", maxtorquedef, 0);
    if (!this->MAXTORQUE[i])
      gzthrow("The controller could't get torque param for " + sw.str());

    // Pre-compute some constants
    A[i] = (2*3.14159265358979)*fmodf(ALPHA[i]+BETA[i]+GAMMA[i], 360)/360;
    L[i] = DIST[i]*cos((2*3.14159265358979)*fmodf(BETA[i]+GAMMA[i], 360)/360);
    R[i] = RADIUS[i]*cos((2*3.14159265358979)*fmodf(GAMMA[i], 360)/360);
  }



	//dribbler ---------------------------------------------------------

   // Get dribbler child
  node = startNode->GetChild("dribbler");
  if (!node)
    gzthrow("Holonome3SwD_Position2d controller requires a <dribbler> node");

  
  this->drSpeed = node->GetFloat("speed", 0.0, 0);
  this->drMaxTorque = node->GetFloat("max-torque", 0.0, 0);
  
   if (!this->drSpeed || !this->drMaxTorque)
      gzthrow("The controller couldn't get one of the params: speed, max-torque for dribbler");

  std::string leftJoint = node->GetString("left-joint", "", 1);
  this->dribblerJoint[0] = this->myParent->GetJoint(leftJoint);
  std::string rightJoint = node->GetString("right-joint", "", 1);
  this->dribblerJoint[1] = this->myParent->GetJoint(rightJoint);
  
  if (!this->dribblerJoint[0] || !this->dribblerJoint[1])
      gzthrow("The controller couldn't get hinge joint for dribbler");

  // kicker params        ----------------------------------------------

  // Get kicker child
  node = startNode->GetChild("kicker");
  if (!node)
    gzthrow("Holonome3SwD_Position2d controller requires a <kicker> node");

  
  this->krForce = node->GetFloat("force", 0.0, 0);
   if (!this->krForce)
      gzthrow("The controller couldn't get force param for dribbler");

  std::string kickerJointName = node->GetString("joint", "", 1);
  this->kickerJoint = this->myParent->GetJoint(kickerJointName);

  if (!this->kickerJoint)
      gzthrow("The controller couldn't get slider joint for kicker");
  


#if 0
  std::cout << "Holonomous robot, here are the params:" << std::endl;
  for (size_t i = 0; i < 3; ++i)
  {
    std::cout << " - swedish" << i << std::endl;
    std::cout << "   alpha =      " << ALPHA[i] << std::endl;
    std::cout << "   beta =       " << BETA[i] << std::endl;
    std::cout << "   gamma =      " << GAMMA[i] << std::endl;
    std::cout << "   dist =       " << DIST[i] << std::endl;
    std::cout << "   radius =     " << RADIUS[i] << std::endl;
    std::cout << "   max-torque = " << MAXTORQUE[i] << std::endl;
    std::cout << "   A =          " << A[i] << std::endl;
    std::cout << "   L =          " << L[i] << std::endl;
    std::cout << "   R =          " << R[i] << std::endl;
    std::cout << std::endl;      
  }
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
/*void Holonome3SwD_Position2d::SaveChild(XMLConfigNode *node)
  {
  node = node->GetChild("wheels");
  if (!node)
  gzthrow("No such child node: wheels");
  for (size_t i = 0; i < 3; ++i)
  {
  std::ostringstream sw; sw << "swedish" << i;
  XMLConfigNode *lnode = node->GetChild(sw.str());
  if (!lnode)
  gzthrow("No such child node wheels/" + sw.str());
  lnode->SetValue("alpha", this->ALPHA[i]);
  lnode->SetValue("distance", this->DIST[i]);
  lnode->SetValue("beta", this->BETA[i]);
  lnode->SetValue("gamma", this->GAMMA[i]);
  lnode->SetValue("radius", this->RADIUS[i]);
  lnode->SetValue("max-torque", this->MAXTORQUE[i]);
// "joint" ???
}
}*/

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void Holonome3SwD_Position2d::InitChild()
{
  ResetData();
  
    
  for (size_t i = 0; i < 2; i++)
  {
	  this->dribblerJoint[i]->SetVelocity( 0, this->drSpeed);
      this->dribblerJoint[i]->SetMaxForce( 0, this->drMaxTorque);
  }
 
   kickerJoint->SetForce(0,-10);	//just to hide kicker
   
   kickerState = KICKER_STATE_READY;
}

void Holonome3SwD_Position2d::ResetChild()
{
  ResetData();
  
  for (size_t i = 0; i < 2; i++)
  {
	  this->dribblerJoint[i]->SetVelocity( 0, this->drSpeed);
      this->dribblerJoint[i]->SetMaxForce( 0, this->drMaxTorque);
  }
  
  kickerJoint->SetForce(0,-10);	//just to hide kicker
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void Holonome3SwD_Position2d::UpdateChild()
{
  this->GetPositionCmd();

  //std::cout << "Anchors: {";
  
  //std::cout<<"joint velocities:\n";

  double maxDelta = 0;
  for (size_t i = 0; i < 3; ++i){
	  double delta = fabs(this->PhiP[i] - this->joint[i]->GetVelocity(0));
	  if (delta > maxDelta)
		  maxDelta = delta;
  }

  for (size_t i = 0; i < 3; ++i)
  {
    if (this->enableMotors)
    {
//	  std::cout<<"["<<i<<"] Current: "<<this->joint[i]->GetVelocity(0)<<" | to be set: "<<this->PhiP[i]<<std::endl;
//      this->joint[i]->SetVelocity( 0, this->PhiP[i]);
//      this->joint[i]->SetMaxForce( 0, this->MAXTORQUE[i] );



//    	std::cout<<"["<<i<<"] Current: "<<this->joint[i]->GetVelocity(0)<<" | to be set: "<<this->PhiP[i]<<std::endl;
    	double delta = fabs(this->PhiP[i] - this->joint[i]->GetVelocity(0));
    	this->joint[i]->SetVelocity( 0, this->PhiP[i]);
    	this->joint[i]->SetMaxForce( 0, (delta / maxDelta) * this->MAXTORQUE[i] );

//    	std::cout<<"Delta: "<<delta<<std::endl;
//    	std::cout<<"Max torque: "<<(delta / maxDelta) * this->MAXTORQUE[i]<<std::endl;
    }
    else
    {
      this->joint[i]->SetVelocity( 0, 0 ); 
      this->joint[i]->SetMaxForce( 0, 0 );
    }   
  }
    
  //------ kicker update part: ----------------------------------
  double kickerPosition = kickerJoint->GetAngle(0).GetAsRadian();
  //std::cout<<"Position: "<<kickerPosition<<std::endl;
  //changing states
  if ( kickerPosition > 0.034  && kickerState == KICKER_STATE_KICK ) {
	  kickerState = KICKER_STATE_BACK;
	 // std::cout<<"Changing state to BACK\n";
  }
  if ( kickerPosition < 0.005 && kickerState == KICKER_STATE_BACK ) {
	  kickerState = KICKER_STATE_READY;
	  //signal that kicker is ready
	  this->myIface->Lock(1);
	  this->myIface->data->cmdVelocity.pos.z = -1;
	  this->myIface->Unlock();
	 // std::cout<<"Changed state to READY\n";
  }
  //doing state actions
  switch(kickerState){
	  
	  case KICKER_STATE_BACK  :
	  case KICKER_STATE_READY : 
	  kickerJoint->SetForce(0,-1); //to prevent kicker from going forward
	  break;
	  
	  case KICKER_STATE_KICK:
	  kickerJoint->SetForce(0,krForce); //to prevent kicker from going forward
	  break;
  }
  
  this->PutPositionData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void Holonome3SwD_Position2d::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Reset internal data to 0
void Holonome3SwD_Position2d::ResetData()
{
  for (size_t i = 0; i < 3; ++i)
  {
    Xi[i] = 0;
    XiP[i] = 0;
    PhiP[i] = 0;
  }
  this->enableMotors = true;
}

//////////////////////////////////////////////////////////////////////////////
// Get commands from the external interface
void Holonome3SwD_Position2d::GetPositionCmd()
{
  if (this->myIface->Lock(1))
  {
    double vx = this->myIface->data->cmdVelocity.pos.x;
    double vy = this->myIface->data->cmdVelocity.pos.y;
    double va = this->myIface->data->cmdVelocity.yaw;

    //Vector3 vel = this->myParent->GetLinearVel();

    for (size_t i = 0; i < 3; ++i)
    {
      this->PhiP[i] = -(1/R[i])*(-sin(A[i])*vx +
          cos(A[i])*vy +
          L[i]*va);
    }

    this->enableMotors = this->myIface->data->cmdEnableMotors > 0;
    
    if (this->myIface->data->cmdVelocity.pos.z > 0 && kickerState == KICKER_STATE_READY){
		//std::cout<<"Recived kick order, changing state to KICK\n";
		kickerState = KICKER_STATE_KICK;
	}
    
    this->myIface->Unlock();

#if 0
    std::cout << "Xi=[" << Xi[0] << ", " << Xi[1] << ", " << Xi[2] << "]; ";
    std::cout << "vx=" << vx <<", vy=" << vy << ", va=" << va << "; ";
    std::cout << "PhiP=[" << PhiP[0] << ", " << PhiP[1] << ", " << PhiP[2] << "];\n";
    std::cout << "Enable motors: "<<this->enableMotors<<" "<<this->myIface->data->cmdEnableMotors<<"\n";
#endif
  }
}

//////////////////////////////////////////////////////////////////////////////
// Update the data in the interface
void Holonome3SwD_Position2d::PutPositionData()
{

  if (this->myIface->Lock(1))
  {
    Vector3 vel = this->myParent->GetWorldLinearVel();
    Vector3 rotvel = this->myParent->GetWorldAngularVel();
  //  std::cout<<"Model vel: "<< vel.x <<" "<< vel.y<<std::endl;
   // std::cout<<"vel length: "<<sqrt((vel.x*vel.x)+(vel.y*vel.y))<<std::endl;
  //  std::cout<<"Time: "<<Simulator::Instance()->GetSimTime().Double()<<std::endl;

    // TODO: Data timestamp
    this->myIface->data->head.time = Simulator::Instance()->GetSimTime().Double();

    this->myIface->data->pose.pos.x = this->Xi[0];
    this->myIface->data->pose.pos.y = this->Xi[1];
    this->myIface->data->pose.yaw = NORMALIZE(this->Xi[2]);

    this->myIface->data->velocity.pos.x = vel.x;
    this->myIface->data->velocity.pos.y = vel.y;
    this->myIface->data->velocity.yaw = rotvel.z;

    // TODO
    this->myIface->data->stall = 0;

    this->myIface->Unlock();
  }
}
