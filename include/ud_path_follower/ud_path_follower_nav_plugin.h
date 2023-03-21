#ifndef UD_PATH_FOLLOWER_PLUGIN_H
#define UD_PATH_FOLLOWER_PLUGIN_H

#include "ud_path_follower.h"
#include <project11_navigation/interfaces/task_to_twist_workflow.h>

namespace ud_path_follower
{

class PathFollowerPlugin: public project11_navigation::TaskToTwistWorkflow, ud_path_follower::PathFollower
{
public:
  void configure(std::string name, project11_navigation::Context::Ptr context) override;
  void setGoal(const std::shared_ptr<project11_navigation::Task>& input) override;
  bool running() override;
  bool getResult(geometry_msgs::TwistStamped& output) override;
private:
  void updateTask();
  project11_navigation::Context::Ptr context_;
  std::shared_ptr<project11_navigation::Task> current_task_;
  ros::Time task_update_time_;
  std::string map_frame_;

};

} // namespace path_follower

#endif
