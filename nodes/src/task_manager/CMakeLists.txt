cmake_minimum_required(VERSION 3.8)
project(task_manager)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskInstanceDetail.msg"
  "msg/TaskInstanceSummary.msg"
  "msg/TaskStatusEvent.msg"
  "msg/TaskTemplateDetail.msg"
  "msg/TaskTemplateSummary.msg"
  "msg/TaskTypeInfo.msg"
  "srv/GetTaskInstanceDetail.srv"
  "srv/GetTaskTemplateDetail.srv"
  "srv/ListActiveTaskInstances.srv"
  "srv/ListHistoryTaskInstances.srv"
  "srv/ListTaskTemplates.srv"
  "srv/ListTaskTypes.srv"
  "srv/PauseResumeTask.srv"
  "srv/StartTask.srv"
  "srv/SubscribeTaskStatus.srv"
  "srv/TerminateTask.srv"
  "srv/UnsubscribeTaskStatus.srv"
  "srv/ValidateTask.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
