cmake_minimum_required(VERSION 3.8)
project(task_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TaskTemplateSummary.msg"
  "msg/TaskTemplateDetail.msg"
  "msg/TaskInstanceSummary.msg"
  "msg/TaskInstanceDetail.msg"
  "msg/TaskTypeInfo.msg"
  "msg/TaskStatusEvent.msg"
  "srv/ListTaskTemplates.srv"
  "srv/GetTaskTemplateDetail.srv"
  "srv/ListActiveTaskInstances.srv"
  "srv/ListHistoryTaskInstances.srv"
  "srv/GetTaskInstanceDetail.srv"
  "srv/ValidateTask.srv"
  "srv/StartTask.srv"
  "srv/TerminateTask.srv"
  "srv/PauseResumeTask.srv"
  "srv/ListTaskTypes.srv"
  "srv/SubscribeTaskStatus.srv"
  "srv/UnsubscribeTaskStatus.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
