#include <constraint_planner/kinematics/panda_model_updater.h>

// arm_arm_state__// arm_arm_state__
void FrankaModelUpdater::PandaRBDLModel()
{
  rbdl_model_ = std::make_shared<Model>();
  rbdl_model_->gravity = Vector3d(0., 0, -9.81);

  double mass[7];
  mass[0] = 4.971e+00;
  mass[1] = 6.469e-01;
  mass[2] = 3.229e+00;
  mass[3] = 3.588e+00;
  mass[4] = 1.226e+00;
  mass[5] = 1.667e+00;
  mass[6] = 7.355e-01;

  Vector3d axis[7];
  axis[0] = Vector3d::UnitZ();
  axis[1] = Vector3d::UnitY();
  axis[2] = Vector3d::UnitZ();
  axis[3] = -1.0 * Vector3d::UnitY();
  axis[4] = Vector3d::UnitZ();
  axis[5] = -1.0 * Vector3d::UnitY();
  axis[6] = -1.0 * Vector3d::UnitZ();

  Vector3d global_joint_position[7];

  /* new vrep */
  global_joint_position[0] = Vector3d(0.0413, 0.0, 0.0);
  global_joint_position[1] = Vector3d(0.0413, 0.0, 0.3330);
  global_joint_position[2] = Vector3d(0.0413, 0.0, 0.3330);
  global_joint_position[3] = Vector3d(0.1238, 0.0, 0.6490);
  global_joint_position[4] = Vector3d(0.0413, 0.0, 1.0330);
  global_joint_position[5] = Vector3d(0.0413, 0.0, 1.0330);
  global_joint_position[6] = Vector3d(0.1293, 0.0, 1.0330);

  joint_position_[0] = global_joint_position[0];
  for (int i = 1; i < 7; i++)
    joint_position_[i] = global_joint_position[i] - global_joint_position[i - 1];

  com_position_[0] = Vector3d(+4.488e-02, +2.081e-03, +3.330e-01);
  com_position_[1] = Vector3d(+3.786e-02, +3.495e-03, +3.617e-01);
  com_position_[2] = Vector3d(+6.852e-02, +3.925e-02, +5.825e-01);
  com_position_[3] = Vector3d(+7.033e-02, -2.745e-02, +7.534e-01);
  com_position_[4] = Vector3d(+2.905e-02, +4.106e-02, +9.946e-01);
  com_position_[5] = Vector3d(+1.011e-01, +1.052e-02, +1.019e+00);
  com_position_[6] = Vector3d(+1.395e-01, +4.252e-03, +9.714e-01);

  for (int i = 0; i < 7; i++)
    com_position_[i] -= global_joint_position[i];

  inertia_[0] << +1.415e-01, -3.604e-05, +1.362e-03, -3.604e-05, +1.422e-01, +3.856e-03, +1.362e-03, +3.856e-03, +1.853e-03;
  inertia_[1] << +1.314e-02, +1.586e-02, +6.157e-03, +1.586e-02, +4.102e-02, -1.189e-03, +6.157e-03, -1.189e-03, +4.347e-02;
  inertia_[2] << +1.750e-02, -2.555e-03, -1.700e-03, -2.555e-03, +1.638e-02, -1.356e-03, -1.700e-03, -1.356e-03, +5.652e-03;
  inertia_[3] << +1.886e-02, -1.089e-03, +7.725e-03, -1.089e-03, +2.162e-02, +4.583e-04, +7.725e-03, +4.583e-04, +9.030e-03;
  inertia_[4] << +3.216e-02, -1.236e-03, -3.752e-03, -1.236e-03, +2.566e-02, +1.765e-03, -3.752e-03, +1.765e-03, +8.866e-03;
  inertia_[5] << +1.488e-03, +6.226e-05, +9.145e-04, +6.226e-05, +7.077e-03, -5.614e-05, +9.145e-04, -5.614e-05, +6.341e-03;
  inertia_[6] << +2.083e-02, +5.372e-04, +2.274e-03, +5.372e-04, +1.754e-02, -7.455e-04, +2.274e-03, -7.455e-04, +6.675e-03;

  for (int i = 0; i < 7; i++)
  {
    body_[i] = Body(mass[i], com_position_[i], inertia_[i]);
    joint_[i] = Joint(JointTypeRevolute, axis[i]);
    if (i == 0)
      body_id_[i] = rbdl_model_->AddBody(0, Math::Xtrans(joint_position_[i]), joint_[i], body_[i]);
    else
      body_id_[i] = rbdl_model_->AddBody(body_id_[i - 1], Math::Xtrans(joint_position_[i]), joint_[i], body_[i]);
  }
}

void FrankaModelUpdater::updatemodel()
{
  transform_ = getTransform(q_);

  mass_matrix_ = getMassMatrix(q_);
  gravity_ = getGravity(q_);
  jacobian_ = getJacobian(q_);
  position_ = transform_.translation();
  rotation_ = transform_.linear();
}

Affine3d FrankaModelUpdater::getTransform(const Vector7d &q)
{
  VectorXd q_temp_ = q;
  VectorXd qdot_temp_;
  qdot_temp_.setZero(7);

  UpdateKinematicsCustom(*rbdl_model_, &q_temp_, &qdot_temp_, NULL);
  auto x = CalcBodyToBaseCoordinates(*rbdl_model_, q, body_id_[7 - 1], com_position_[7 - 1], true);
  auto rotation = CalcBodyWorldOrientation(*rbdl_model_, q, body_id_[7 - 1], true).transpose();

  Matrix3d body_to_ee_rotation;
  body_to_ee_rotation.setIdentity();
  body_to_ee_rotation(1, 1) = -1;
  body_to_ee_rotation(2, 2) = -1;

  rotation = rotation * body_to_ee_rotation;

  Affine3d transform;
  transform.linear() = rotation;
  transform.translation() = x;

  return transform;
}

Matrix<double, 6, 7> FrankaModelUpdater::getJacobian(const Vector7d &q)
{
  MatrixXd j_temp;
  j_temp.resize(6, 7);
  CalcPointJacobian6D(*rbdl_model_, q, body_id_[7 - 1], com_position_[7 - 1], j_temp, true);

  Matrix<double, 6, 7> j;
  for (int i = 0; i < 2; i++)
    j.block<3, 7>(i * 3, 0) = j_temp.block<3, 7>(3 - i * 3, 0);

  return j;
}

Matrix<double, 7, 7> FrankaModelUpdater::getMassMatrix(const Vector7d &q)
{
  MatrixXd m_temp;
  m_temp.resize(7, 7);
  Matrix<double, 7, 7> mass;
  CompositeRigidBodyAlgorithm(*rbdl_model_, q, m_temp, true);

  return m_temp;
}

Matrix<double, 7, 1> FrankaModelUpdater::getGravity(const Vector7d &q)
{
  VectorXd g_temp;
  g_temp.resize(7);
  NonlinearEffects(*rbdl_model_, q, Matrix<double, 7, 1>::Zero(), g_temp);
  return g_temp;
}

void FrankaModelUpdater::setInitialTransform()
{
  initial_transform_ = transform_;
}

void FrankaModelUpdater::setRobotState(franka::RobotState state)
{
  state_ = state;
}

panda_ik::panda_ik() : tracik_solver(chain_start, chain_end, "/single_robot_description")
{
  KDL::JntArray ll, ul; //lower joint limits, upper joint limits

  bool valid = tracik_solver.getKDLChain(chain);
  if (!valid)
  {
    std::cout << "error~~~~~`" << std::endl;
    return;
  }

  valid = tracik_solver.getKDLLimits(ll, ul);
  if (!valid)
  {
    std::cout << "error~~~~~`" << std::endl;
    return;
  }

  assert(chain.getNrOfJoints() == ll.data.size());
  assert(chain.getNrOfJoints() == ul.data.size());

  lb_ = ll.data;
  ub_ = ul.data;
  length = (ub_ - lb_) /2.;

  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j = 0; j < nominal.data.size(); j++)
  {
    nominal(j) = (ll(j) + ul(j)) / 2.0;
  }

}

Eigen::VectorXd panda_ik::getRandomConfig()
{
  return length.asDiagonal() * Eigen::VectorXd::Random(7) + length + lb_;
}

bool panda_ik::solve(VectorXd start, Affine3d target, Eigen::Ref<Eigen::VectorXd> solution)
{
  KDL::JntArray nominal(chain.getNrOfJoints());
  for (uint j = 0; j < nominal.data.size(); j++)
    nominal(j) = start[j];

  KDL::JntArray result;
  KDL::Frame end_effector_pose;
  for (size_t i = 0; i < 3; i++)
    end_effector_pose.p(i) = target.translation()[i];

  Matrix3d Rot_d = target.linear();
  KDL::Rotation A;
  A.data[0] = Rot_d(0, 0);
  A.data[1] = Rot_d(0, 1);
  A.data[2] = Rot_d(0, 2);
  A.data[3] = Rot_d(1, 0);
  A.data[4] = Rot_d(1, 1);
  A.data[5] = Rot_d(1, 2);
  A.data[6] = Rot_d(2, 0);
  A.data[7] = Rot_d(2, 1);
  A.data[8] = Rot_d(2, 2);
  end_effector_pose.M = A;

  if (tracik_solver.CartToJnt(nominal, end_effector_pose, result) >= 0)
  {
    solution = result.data;
    // std::cout << "solve" << std::endl;
    return true;
  }
  return false;
}

bool panda_ik::randomSolve(Affine3d target, Eigen::Ref<Eigen::VectorXd> solution)
{
  return solve(getRandomConfig(), target, solution);
}