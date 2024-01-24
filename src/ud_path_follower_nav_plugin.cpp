#include <ud_path_follower/ud_path_follower_nav_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <project11_navigation/robot_capabilities.h>

PLUGINLIB_EXPORT_CLASS(ud_path_follower::PathFollowerPlugin, project11_navigation::TaskToTwistWorkflow)

namespace ud_path_follower
{

void PathFollowerPlugin::configure(std::string name, project11_navigation::Context::Ptr context)
{
  context_ = context;
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);
  PathFollower::initialize(nh, private_nh, context_->tfBuffer());
  vis_display_.lines.clear();
  sendDisplay();
}

void PathFollowerPlugin::setGoal(const project11_navigation::TaskPtr& input)
{
  if(current_task_ != input)
    task_update_time_ = ros::Time();
  current_task_ = input;
  updateTask();
}

void PathFollowerPlugin::updateTask()
{
  if(current_task_)
  {
    if(current_task_->lastUpdateTime() != task_update_time_)
    {
      if(!current_task_->done())
      {
        double speed = 0.0;
        const auto& poses = current_task_->message().poses;
        if(poses.empty() || poses.front().header.stamp.isZero() || poses.back().header.stamp <= poses.front().header.stamp)
        {
          ros::NodeHandle nh("~");
          speed = project11_navigation::RobotCapabilities(nh).default_velocity.linear.x;
        }

        PathFollower::setGoal(current_task_->message().poses, speed);
      }
      else
      {
        PathFollower::setGoal(std::vector<geometry_msgs::PoseStamped>());
      }
      updateDisplay();
      task_update_time_ = current_task_->lastUpdateTime();
    }

  }
  else
  {
    PathFollower::setGoal(std::vector<geometry_msgs::PoseStamped>());
    updateDisplay();
  }
}

bool PathFollowerPlugin::running()
{
  updateTask();
  if(current_task_ && !current_task_->done())
    if(goalReached())
      current_task_->setDone();
    else
      return true;
  return false;
}

bool PathFollowerPlugin::getResult(geometry_msgs::TwistStamped& output)
{
  m_base_frame = output.header.frame_id;
  auto ret = generateCommands(output.twist);
  sendDisplay();
  return ret;
}

}  // namespace path_follower
