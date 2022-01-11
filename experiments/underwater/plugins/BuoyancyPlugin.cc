/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "BuoyancyPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)

/////////////////////////////////////////////////
BuoyancyPlugin::BuoyancyPlugin()
  // Density of liquid water at 1 atm pressure and 15 degrees Celsius.
  : fluidDensity(999.1026), surfaceHeight(0)
{
}

/////////////////////////////////////////////////
void BuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  this->model = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  this->sdf = _sdf;

  if (this->sdf->HasElement("fluid_density"))
  {
    this->fluidDensity = this->sdf->Get<double>("fluid_density");
  }

  if (this->sdf->HasElement("surface_height"))
  {
    this->surfaceHeight = this->sdf->Get<double>("surface_height");
  }

  // Get "center of volume" and "volume" that were inputted in SDF
  // SDF input is recommended for mesh or polylines collision shapes
  if (this->sdf->HasElement("link"))
  {
    for (sdf::ElementPtr linkElem = this->sdf->GetElement("link"); linkElem;
         linkElem = linkElem->GetNextElement("link"))
    {
      int id = -1;
      std::string name = "";
      if (linkElem->HasAttribute("name"))
      {
        name = linkElem->Get<std::string>("name");
        physics::LinkPtr link =
            this->model->GetLink(name);
        if (!link)
        {
          gzwarn << "Specified link [" << name << "] not found." << std::endl;
          continue;
        }
        id = link->GetId();
      }
      else
      {
        gzwarn << "Required attribute name missing from link [" << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        // Exit if we didn't set ID
        continue;
      }

      if (this->volPropsMap.count(id) != 0)
      {
        gzwarn << "Properties for link [" << name << "] already set, skipping "
               << "second property block" << std::endl;
      }

      if (linkElem->HasElement("center_of_volume"))
      {
        ignition::math::Vector3d cov = linkElem->GetElement("center_of_volume")
            ->Get<ignition::math::Vector3d>();
        this->volPropsMap[id].cov = cov;
      }
      else
      {
        gzwarn << "Required element center_of_volume missing from link ["
               << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }

      if (linkElem->HasElement("volume"))
      {
        double volume = linkElem->GetElement("volume")->Get<double>();
        if (volume <= 0)
        {
          gzwarn << "Nonpositive volume specified in BuoyancyPlugin!"
                 << std::endl;
          // Remove the element from the map since it's invalid.
          this->volPropsMap.erase(id);
          continue;
        }
        this->volPropsMap[id].volume = volume;
      }
      else
      {
        gzwarn << "Required element volume missing from element link [" << name
               << "] in BuoyancyPlugin SDF" << std::endl;
        continue;
      }
    }
  }

  // For links the user didn't input, precompute the center of volume and
  // density. This will be accurate for simple shapes.
  for (auto link : this->model->GetLinks())
  {
    int id = link->GetId();
    if (this->volPropsMap.find(id) == this->volPropsMap.end())
    {
      double volumeSum = 0;
      ignition::math::Vector3d weightedPosSum = ignition::math::Vector3d::Zero;

      // The center of volume of the link is a weighted average over the pose
      // of each collision shape, where the weight is the volume of the shape
      for (auto collision : link->GetCollisions())
      {

        // The modules have a different actual volume than their collision box
        // and by checking their name we can determine which module we have and
        // then we can assign the pre-calculated volume to that module to 
        // properly simulate the buoyancy of the module.
        std::string collisionName = collision->GetName();
        double volume = 0;
        if (collisionName.find("CoreComponent") != std::string::npos) // if the module is the core module
        {
          volume = 5.054e-05;
        }
        else if (collisionName.find("ActiveHinge") != std::string::npos) // if the module is an active hinge module
        {
          std::cout << "Active Hinge ";
          if (collisionName.find("frame") != std::string::npos)       // frame
          {
            volume = 3.032e-06;
          }
          else if (collisionName.find("servo") != std::string::npos) // servo 2
          {
            // The servo consists of two parts, I don't know the volume of each part sperately but I do know the volume of both parts together: 1.014e-05 m^3
            // Thus, if I give each part half of that volume (so 5.07e-06 m^3) the total will have the right volume 
            volume = 5.07e-06; 
          }
        }
        else if (collisionName.find("FixedBrick") != std::string::npos) // if the module is a brick module
        {
          volume = 1.070e-05;
        }
        else if (collisionName.find("TouchSensor") != std::string::npos) // if the module is a touch sensor module
        {
          volume = 2.735e-6;
        }

        //double volume = collision->GetShape()->ComputeVolume();
        volumeSum += volume;
        weightedPosSum += volume*collision->WorldPose().Pos();
      }
      // Subtract the center of volume into the link frame.
      this->volPropsMap[id].cov =
          weightedPosSum/volumeSum - link->WorldPose().Pos();
      this->volPropsMap[id].volume = volumeSum;
    }
  }
}

/////////////////////////////////////////////////
void BuoyancyPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void BuoyancyPlugin::OnUpdate()
{
  for (auto link : this->model->GetLinks())
  {
    if(link->WorldCoGPose().Pos().Z() < this->surfaceHeight)
    {
      VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
      double volume = volumeProperties.volume;
      GZ_ASSERT(volume > 0, "Nonpositive volume found in volume properties!");

      // By Archimedes' principle,
      // buoyancy = -(mass*gravity)*fluid_density/object_density
      // object_density = mass/volume, so the mass term cancels.
      // Therefore,
      ignition::math::Vector3d buoyancy =
          -this->fluidDensity * volume * this->model->GetWorld()->Gravity();

      ignition::math::Pose3d linkFrame = link->WorldPose();
      // rotate buoyancy into the link frame before applying the force.
      ignition::math::Vector3d buoyancyLinkFrame =
          linkFrame.Rot().Inverse().RotateVector(buoyancy);

      link->AddLinkForce(buoyancyLinkFrame, volumeProperties.cov);
    }
  }
}
