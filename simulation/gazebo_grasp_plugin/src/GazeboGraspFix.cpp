#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/physics.hh>
#include <stdio.h>

#include <gazebo_grasp_plugin/GazeboGraspFix.h>

using gazebo::GazeboGraspFix;

#define DEFAULT_FORCES_ANGLE_TOLERANCE 120
#define DEFAULT_UPDATE_RATE 5
#define DEFAULT_MAX_GRIP_COUNT 10
#define DEFAULT_RELEASE_TOLERANCE 0.005
#define DEFAULT_DISABLE_COLLISIONS_ON_ATTACH false

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboGraspFix)

GazeboGraspFix::GazeboGraspFix()
{
  InitValues();
}

GazeboGraspFix::GazeboGraspFix(physics::ModelPtr _model)
{
  InitValues();
}

GazeboGraspFix::~GazeboGraspFix()
{
  this->update_connection.reset();
  if (this->node)
    this->node->Fini();
  this->node.reset();
}

void GazeboGraspFix::Init()
{
  this->prevUpdateTime = common::Time::GetWallTime();
}

void GazeboGraspFix::InitValues()
{
#if GAZEBO_MAJOR_VERSION > 2
  gazebo::common::Console::SetQuiet(false);
#endif

  this->prevUpdateTime = common::Time::GetWallTime();

  this->node = transport::NodePtr(new transport::Node());
}

void GazeboGraspFix::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{

  physics::ModelPtr model = _parent;
  this->world = model->GetWorld();

  sdf::ElementPtr disableCollisionsOnAttachElem =
      _sdf->GetElement("disable_collisions_on_attach");
  if (!disableCollisionsOnAttachElem)
  {

    this->disableCollisionsOnAttach = DEFAULT_DISABLE_COLLISIONS_ON_ATTACH;
  }
  else
  {
    std::string str = disableCollisionsOnAttachElem->Get<std::string>();
    bool bVal = false;
    if ((str == "true") || (str == "1"))
      bVal = true;
    this->disableCollisionsOnAttach = bVal;
  }

  sdf::ElementPtr forcesAngleToleranceElem =
      _sdf->GetElement("forces_angle_tolerance");
  if (!forcesAngleToleranceElem)
  {

    this->forcesAngleTolerance = DEFAULT_FORCES_ANGLE_TOLERANCE * M_PI / 180;
  }
  else
  {
    this->forcesAngleTolerance =
        forcesAngleToleranceElem->Get<float>() * M_PI / 180;
  }

  sdf::ElementPtr updateRateElem = _sdf->GetElement("update_rate");
  double _updateSecs;
  if (!updateRateElem)
  {
    _updateSecs = 1.0 / DEFAULT_UPDATE_RATE;
  }
  else
  {
    int _rate = updateRateElem->Get<int>();
    double _updateRate = _rate;
    _updateSecs = 1.0 / _updateRate;
  }
  this->updateRate = common::Time(0, common::Time::SecToNano(_updateSecs));

  sdf::ElementPtr maxGripCountElem = _sdf->GetElement("max_grip_count");
  if (!maxGripCountElem)
  {

    this->maxGripCount = DEFAULT_MAX_GRIP_COUNT;
  }
  else
  {
    this->maxGripCount = maxGripCountElem->Get<int>();
  }

  sdf::ElementPtr gripCountThresholdElem =
      _sdf->GetElement("grip_count_threshold");
  if (!gripCountThresholdElem)
  {
    this->gripCountThreshold = floor(this->maxGripCount / 2.0);
  }
  else
  {
    this->gripCountThreshold = gripCountThresholdElem->Get<int>();
  }

  sdf::ElementPtr releaseToleranceElem = _sdf->GetElement("release_tolerance");
  if (!releaseToleranceElem)
  {
    this->releaseTolerance = DEFAULT_RELEASE_TOLERANCE;
  }
  else
  {
    this->releaseTolerance = releaseToleranceElem->Get<float>();
  }

  std::vector<std::string> collisionNames;

  sdf::ElementPtr armElem = _sdf->GetElement("arm");
  if (!armElem)
  {
    gzerr << "GazeboGraspFix: Cannot load the GazeboGraspFix without any <arm> "
             "declarations"
          << std::endl;
    return;
  }
  for (; armElem != NULL; armElem = armElem->GetNextElement("arm"))
  {
    sdf::ElementPtr armNameElem = armElem->GetElement("arm_name");
    sdf::ElementPtr handLinkElem = armElem->GetElement("palm_link");
    sdf::ElementPtr fingerLinkElem = armElem->GetElement("gripper_link");

    if (!handLinkElem || !fingerLinkElem || !armNameElem)
    {
      gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix arm because "
            << "not all of <arm_name>, <palm_link> and <gripper_link> elements "
               "specified in URDF/SDF. Skipping."
            << std::endl;
      continue;
    }

    std::string armName = armNameElem->Get<std::string>();
    std::string palmName = handLinkElem->Get<std::string>();

    std::vector<std::string> fingerLinkNames;
    for (; fingerLinkElem != NULL;
         fingerLinkElem = fingerLinkElem->GetNextElement("gripper_link"))
    {
      std::string linkName = fingerLinkElem->Get<std::string>();
      fingerLinkNames.push_back(linkName);
    }

    if (grippers.find(armName) != grippers.end())
    {
      gzerr << "GazeboGraspFix: Arm named " << armName
            << " was already added, cannot add it twice." << std::endl;
    }
    GazeboGraspGripper& gripper = grippers[armName];
    std::map<std::string, physics::CollisionPtr> _collisions;
    if (!gripper.Init(_parent, armName, palmName, fingerLinkNames,
                      disableCollisionsOnAttach, _collisions))
    {
      gzerr << "GazeboGraspFix: Could not initialize arm " << armName
            << ". Skipping." << std::endl;
      grippers.erase(armName);
      continue;
    }

    for (std::map<std::string, physics::CollisionPtr>::iterator collIt =
             _collisions.begin();
         collIt != _collisions.end(); ++collIt)
    {
      const std::string& collName = collIt->first;

      std::map<std::string, std::string>::iterator collIter =
          this->collisions.find(collName);
      if (collIter != this->collisions.end())
      {
        gzwarn << "GazeboGraspFix: Adding Gazebo collision link element "
               << collName
               << " multiple times, the grasp plugin may not work properly"
               << std::endl;
        continue;
      }
      this->collisions[collName] = armName;
      collisionNames.push_back(collName);
    }
  }

  if (grippers.empty())
  {
    gzerr << "ERROR: GazeboGraspFix: Cannot use a GazeboGraspFix because "
          << "no arms were configured successfully. Plugin will not work."
          << std::endl;
    return;
  }

  physics::PhysicsEnginePtr physics = this->world->Physics();
  this->node->Init(this->world->Name());
  physics::ContactManager* contactManager = physics->GetContactManager();
  contactManager->PublishContacts();

  std::string topic =
      contactManager->CreateFilter(model->GetScopedName(), collisionNames);
  if (!this->contactSub)
  {
    bool latching = false;
    this->contactSub = this->node->Subscribe(topic, &GazeboGraspFix::OnContact,
                                             this, latching);
  }

  update_connection = event::Events::ConnectWorldUpdateEnd(
      boost::bind(&GazeboGraspFix::OnUpdate, this));
}

class GazeboGraspFix::ObjectContactInfo
{
    public:
  std::vector<ignition::math::Vector3d> appliedForces;

  std::map<std::string, int> grippersInvolved;

  int maxGripperContactCnt;

  std::string maxContactGripper;
};

bool GazeboGraspFix::isGripperLink(const std::string& linkName,
                                   std::string& gripperName) const
{
  for (std::map<std::string, GazeboGraspGripper>::const_iterator it =
           grippers.begin();
       it != grippers.end(); ++it)
  {
    if (it->second.hasLink(linkName))
    {
      gripperName = it->first;
      return true;
    }
  }
  return false;
}

std::map<std::string, std::string> GazeboGraspFix::getAttachedObjects() const
{
  std::map<std::string, std::string> ret;
  for (std::map<std::string, GazeboGraspGripper>::const_iterator it =
           grippers.begin();
       it != grippers.end(); ++it)
  {
    const std::string& gripperName = it->first;
    const GazeboGraspGripper& gripper = it->second;
    if (gripper.isObjectAttached())
    {
      ret[gripper.attachedObject()] = gripperName;
    }
  }
  return ret;
}

bool GazeboGraspFix::objectAttachedToGripper(
    const ObjectContactInfo& objContInfo,
    std::string& attachedToGripper) const
{
  for (std::map<std::string, int>::const_iterator gripInvIt =
           objContInfo.grippersInvolved.begin();
       gripInvIt != objContInfo.grippersInvolved.end(); ++gripInvIt)
  {
    const std::string& gripperName = gripInvIt->first;
    if (objectAttachedToGripper(gripperName, attachedToGripper))
    {
      return true;
    }
  }
  return false;
}

bool GazeboGraspFix::objectAttachedToGripper(
    const std::string& gripperName,
    std::string& attachedToGripper) const
{
  std::map<std::string, GazeboGraspGripper>::const_iterator gIt =
      grippers.find(gripperName);
  if (gIt == grippers.end())
  {
    gzerr << "GazeboGraspFix: Inconsistency, gripper " << gripperName
          << " not found in GazeboGraspFix grippers" << std::endl;
    return false;
  }
  const GazeboGraspGripper& gripper = gIt->second;
  if (gripper.isObjectAttached())
  {
    attachedToGripper = gripperName;
    return true;
  }
  return false;
}

/**
 * Helper class to encapsulate a collision information.
 * One contact has two bodies, and only
 * the ones where one of the bodies is a gripper link are considered.
 * Each contact consists of a *list* of forces with their own origin/position
 * each (e.g. when the object and gripper are colliding at several places). The
 * averages of each contact's force vectors along with their origins are
 * *accumulated* in the given Vector3 \e pos and \eforce objects.
 * The number of additions is stored in \e sum.
 * This is to get the average force application over time between link and
 * object.
 *
 * \author Jennifer Buehler
 */
class GazeboGraspFix::CollidingPoint
{
    public:
  CollidingPoint() : sum(0)
  {
  }
  CollidingPoint(const CollidingPoint& o)
      : gripperName(o.gripperName),
        collLink(o.collLink),
        collObj(o.collObj),
        force(o.force),
        pos(o.pos),
        objPos(o.objPos),
        sum(o.sum)
  {
  }

  // Name of the gripper/arm involved in contact point
  // This is not the specific link, but the name of the
  // whole gripper
  std::string gripperName;

  // the collision
  physics::CollisionPtr collLink, collObj;

  // average force vector of the colliding point
  ignition::math::Vector3d force;

  // position (relative to reference frame of gripper
  // collision surface) where the contact happens on collision surface
  ignition::math::Vector3d pos;

  // position (relative to reference frame of *gripper* collision surface)
  // where the object center is located during collision.
  ignition::math::Vector3d objPos;

  // sum of force and pose (they are actually summed
  // up from several contact points).
  // Divide both \e force and \e pos by this to obtain average
  int sum;
};

void GazeboGraspFix::OnUpdate()
{
  if ((common::Time::GetWallTime() - this->prevUpdateTime) < this->updateRate)
    return;

  this->mutexContacts.lock();
  std::map<std::string, std::map<std::string, CollidingPoint>> contPoints(
      this->contacts);
  this->contacts.clear(); // clear so it can be filled anew by OnContact().
  this->mutexContacts.unlock();

  std::map<std::string, std::map<std::string, CollidingPoint>>::iterator objIt;
  std::map<std::string, ObjectContactInfo> objectContactInfo;

  for (objIt = contPoints.begin(); objIt != contPoints.end(); ++objIt)
  {
    std::string objName = objIt->first;

    ObjectContactInfo& objContInfo = objectContactInfo[objName];

    std::map<std::string, CollidingPoint>::iterator lIt;
    for (lIt = objIt->second.begin(); lIt != objIt->second.end(); ++lIt)
    {
      std::string linkName = lIt->first;
      CollidingPoint& collP = lIt->second;
      ignition::math::Vector3d avgForce = collP.force / collP.sum;

      objContInfo.appliedForces.push_back(avgForce);

      int& gContactCnt = objContInfo.grippersInvolved[collP.gripperName];
      gContactCnt++;
      int& _maxGripperContactCnt = objContInfo.maxGripperContactCnt;
      if (gContactCnt > _maxGripperContactCnt)
      {
        _maxGripperContactCnt = gContactCnt;
        objContInfo.maxContactGripper = collP.gripperName;
      }
    }
  }

  std::set<std::string> grippedObjects;
  for (std::map<std::string, ObjectContactInfo>::iterator ocIt =
           objectContactInfo.begin();
       ocIt != objectContactInfo.end(); ++ocIt)
  {
    const std::string& objName = ocIt->first;
    const ObjectContactInfo& objContInfo = ocIt->second;

    float minAngleDiff = this->forcesAngleTolerance;
    if (!checkGrip(objContInfo.appliedForces, minAngleDiff, 0.3))
      continue;

    grippedObjects.insert(objName);

    int& counts = this->gripCounts[objName];
    if (counts < this->maxGripCount)
      ++counts;

    if (counts <= this->gripCountThreshold)
      continue;

    std::string attachedToGripper;
    bool isAttachedToGripper =
        objectAttachedToGripper(objContInfo, attachedToGripper);
    if (isAttachedToGripper)
    {

      continue;
    }
    const std::string& graspingGripperName = objContInfo.maxContactGripper;
    std::map<std::string, GazeboGraspGripper>::iterator gIt =
        grippers.find(graspingGripperName);
    if (gIt == grippers.end())
    {
      gzerr << "GazeboGraspFix: Inconsistency, gripper '" << graspingGripperName
            << "' not found in GazeboGraspFix grippers, so cannot do "
               "attachment of object "
            << objName << std::endl;
      continue;
    }
    GazeboGraspGripper& graspingGripper = gIt->second;

    if (graspingGripper.isObjectAttached())
    {
      gzerr << "GazeboGraspFix has found that object "
            << graspingGripper.attachedObject()
            << " is already attached to gripper " << graspingGripperName
            << ", so can't grasp '" << objName << "'!" << std::endl;
      continue;
    }

    const std::map<std::string, CollidingPoint>& contPointsTmp =
        contPoints[objName];
    std::map<std::string, CollidingPoint>& attGripConts =
        this->attachGripContacts[objName];
    attGripConts.clear();
    std::map<std::string, CollidingPoint>::const_iterator contPointsIt;
    for (contPointsIt = contPointsTmp.begin();
         contPointsIt != contPointsTmp.end(); ++contPointsIt)
    {
      const std::string& collidingLink = contPointsIt->first;
      const CollidingPoint& collidingPoint = contPointsIt->second;

      if (graspingGripper.hasCollisionLink(collidingLink))
      {

        attGripConts[collidingLink] = collidingPoint;
      }
    }

    if (!graspingGripper.HandleAttach(objName))
    {
      gzerr << "GazeboGraspFix: Could not attach object " << objName
            << " to gripper " << graspingGripperName << std::endl;
    }
    this->OnAttach(objName, graspingGripperName);
  }

  std::map<std::string, std::string> attachedObjects = getAttachedObjects();

  std::map<std::string, int>::iterator gripCntIt;
  for (gripCntIt = this->gripCounts.begin();
       gripCntIt != this->gripCounts.end(); ++gripCntIt)
  {
    const std::string& objName = gripCntIt->first;

    if (grippedObjects.find(objName) != grippedObjects.end())
    {
      continue;
    }

    if (gripCntIt->second > 0)
      --(gripCntIt->second);

    std::map<std::string, std::string>::iterator attIt =
        attachedObjects.find(objName);
    bool isAttached = (attIt != attachedObjects.end());

    if (!isAttached || (gripCntIt->second > this->gripCountThreshold))
      continue;

    const std::string& graspingGripperName = attIt->second;
    std::map<std::string, std::map<std::string, CollidingPoint>>::iterator
        initCollIt = this->attachGripContacts.find(objName);
    if (initCollIt == this->attachGripContacts.end())
    {
      std::cerr << "ERROR: Consistency: Could not find attachGripContacts for "
                << objName << std::endl;
      continue;
    }

    std::map<std::string, CollidingPoint>& initColls = initCollIt->second;
    int cntRelease = 0;

    std::map<std::string, CollidingPoint>::iterator pointIt;
    for (pointIt = initColls.begin(); pointIt != initColls.end(); ++pointIt)
    {
      CollidingPoint& cpInfo = pointIt->second;
      ignition::math::Vector3d relContactPos = cpInfo.pos / cpInfo.sum;
      ignition::math::Vector3d relObjPos = cpInfo.objPos / cpInfo.sum;

      ignition::math::Pose3d currObjWorldPose =
          cpInfo.collObj->GetLink()->WorldPose();

      ignition::math::Pose3d currLinkWorldPose =
          cpInfo.collLink->GetLink()->WorldPose();

      ignition::math::Matrix4d worldToLink(currLinkWorldPose.Rot());
      worldToLink.SetTranslation(currLinkWorldPose.Pos());

      ignition::math::Matrix4d temp;
      ignition::math::Matrix4d linkToContact = temp.Identity;
      linkToContact.SetTranslation(relContactPos);

      ignition::math::Matrix4d _currContactWorldPose =
          worldToLink * linkToContact;
      ignition::math::Vector3d currContactWorldPose =
          _currContactWorldPose.Translation();

      ignition::math::Vector3d oldObjDist = relContactPos - relObjPos;
      ignition::math::Vector3d newObjDist =
          currContactWorldPose - currObjWorldPose.Pos();

      float diff = fabs(oldObjDist.Length() - newObjDist.Length());

      if (diff > releaseTolerance)
      {
        ++cntRelease;
      }
    }

    if (cntRelease > 0)
    {
      std::map<std::string, GazeboGraspGripper>::iterator gggIt =
          grippers.find(graspingGripperName);
      if (gggIt == grippers.end())
      {
        gzerr << "GazeboGraspFix: Inconsistency: Gazebo gripper '"
              << graspingGripperName
              << "' not found when checking for detachment" << std::endl;
        continue;
      }
      GazeboGraspGripper& graspingGripper = gggIt->second;
      graspingGripper.HandleDetach(objName);
      this->OnDetach(objName, graspingGripperName);
      gripCntIt->second = 0;
      this->attachGripContacts.erase(initCollIt);
    }
  }

  this->prevUpdateTime = common::Time::GetWallTime();
}

double angularDistance(const ignition::math::Vector3d& _v1,
                       const ignition::math::Vector3d& _v2)
{
  ignition::math::Vector3d v1 = _v1;
  ignition::math::Vector3d v2 = _v2;
  v1.Normalize();
  v2.Normalize();
  return acos(v1.Dot(v2));
}

bool GazeboGraspFix::checkGrip(
    const std::vector<ignition::math::Vector3d>& forces,
    float minAngleDiff,
    float lengthRatio)
{
  if (((lengthRatio > 1) || (lengthRatio < 0)) &&
      (lengthRatio > 1e-04 && (fabs(lengthRatio - 1) > 1e-04)))
  {
    std::cerr << "ERROR: checkGrip: always specify a lengthRatio of [0..1]"
              << std::endl;
    return false;
  }
  if (minAngleDiff < M_PI_2)
  {
    std::cerr
        << "ERROR: checkGrip: min angle must be at least 90 degrees (PI/2)"
        << std::endl;
    return false;
  }
  std::vector<ignition::math::Vector3d>::const_iterator it1, it2;
  for (it1 = forces.begin(); it1 != forces.end(); ++it1)
  {
    ignition::math::Vector3d v1 = *it1;
    for (it2 = it1 + 1; it2 != forces.end(); ++it2)
    {
      ignition::math::Vector3d v2 = *it2;
      float l1 = v1.Length();
      float l2 = v2.Length();
      if ((l1 < 1e-04) || (l2 < 1e-04))
        continue;

      float angle = angularDistance(v1, v2);

      if (angle > minAngleDiff)
      {
        float ratio;
        if (l1 > l2)
          ratio = l2 / l1;
        else
          ratio = l1 / l2;

        if (ratio >= lengthRatio)
        {
          return true;
        }
      }
    }
  }
  return false;
}

void GazeboGraspFix::OnContact(const ConstContactsPtr& _msg)
{
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    physics::CollisionPtr collision1 =
        boost::dynamic_pointer_cast<physics::Collision>(
            this->world->EntityByName(_msg->contact(i).collision1()));
    physics::CollisionPtr collision2 =
        boost::dynamic_pointer_cast<physics::Collision>(
            this->world->EntityByName(_msg->contact(i).collision2()));

    if ((collision1 && !collision1->IsStatic()) &&
        (collision2 && !collision2->IsStatic()))
    {
      std::string name1 = collision1->GetScopedName();
      std::string name2 = collision2->GetScopedName();

      int count = _msg->contact(i).position_size();
      if ((count != _msg->contact(i).normal_size()) ||
          (count != _msg->contact(i).wrench_size()) ||
          (count != _msg->contact(i).depth_size()))
      {
        gzerr << "GazeboGraspFix: Contact message has invalid array sizes\n"
              << std::endl;
        continue;
      }

      std::string collidingObjName, collidingLink, gripperOfCollidingLink;
      physics::CollisionPtr linkCollision;
      physics::CollisionPtr objCollision;

      physics::Contact contact;
      contact = _msg->contact(i);

      if (contact.count < 1)
      {
        std::cerr
            << "ERROR: GazeboGraspFix: Not enough forces given for contact of ."
            << name1 << " / " << name2 << std::endl;
        continue;
      }

      std::vector<ignition::math::Vector3d> force;

      std::map<std::string, std::string>::const_iterator gripperCollIt =
          this->collisions.find(name2);
      if (gripperCollIt != this->collisions.end())
      {
        collidingObjName = name1;
        collidingLink = name2;
        linkCollision = collision2;
        objCollision = collision1;
        gripperOfCollidingLink = gripperCollIt->second;
        for (int k = 0; k < contact.count; ++k)
          force.push_back(contact.wrench[k].body1Force);
      }
      else if ((gripperCollIt = this->collisions.find(name1)) !=
               this->collisions.end())
      {
        collidingObjName = name2;
        collidingLink = name1;
        linkCollision = collision1;
        objCollision = collision2;
        gripperOfCollidingLink = gripperCollIt->second;
        for (int k = 0; k < contact.count; ++k)
          force.push_back(contact.wrench[k].body2Force);
      }

      ignition::math::Vector3d avgForce;

      for (int k = 0; k < force.size(); ++k)
      {
        avgForce += force[k];
      }
      avgForce /= force.size();

      ignition::math::Vector3d avgPos;

      for (int k = 0; k < contact.count; ++k)
        avgPos += contact.positions[k];
      avgPos /= contact.count;

      ignition::math::Pose3d linkWorldPose =
          linkCollision->GetLink()->WorldPose();

      ignition::math::Matrix4<double> worldToLink(linkWorldPose.Rot());
      worldToLink.SetTranslation(linkWorldPose.Pos());

      ignition::math::Matrix4d temp;
      ignition::math::Matrix4<double> worldToContact = temp.Identity;

      worldToContact.SetTranslation(avgPos);

      ignition::math::Matrix4<double> worldToLinkInv = worldToLink.Inverse();
      ignition::math::Matrix4<double> contactInLocal =
          worldToLinkInv * worldToContact;
      ignition::math::Vector3d contactPosInLocal = contactInLocal.Translation();

      ignition::math::Pose3d objWorldPose =
          objCollision->GetLink()->WorldPose();
      ignition::math::Matrix4<double> worldToObj(objWorldPose.Rot());
      worldToObj.SetTranslation(objWorldPose.Pos());

      ignition::math::Matrix4<double> objInLocal = worldToLinkInv * worldToObj;
      ignition::math::Vector3d objPosInLocal = objInLocal.Translation();

      {
        boost::mutex::scoped_lock lock(this->mutexContacts);
        CollidingPoint& p =
            this->contacts[collidingObjName]
                          [collidingLink]; // inserts new entry if doesn't exist
        p.gripperName = gripperOfCollidingLink;
        p.collLink = linkCollision;
        p.collObj = objCollision;
        p.force += avgForce;
        p.pos += contactPosInLocal;
        p.objPos += objPosInLocal;
        p.sum++;
      }
    }
  }
}
