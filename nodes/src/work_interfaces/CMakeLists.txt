cmake_minimum_required(VERSION 3.8)
project(work_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WorkTaskTypeInfo.msg"
  "msg/WorkSummary.msg"
  "msg/WorkTaskInfo.msg"
  "msg/WorkDetail.msg"
  "msg/WorkExecutorInfo.msg"
  "msg/WorkInstanceSummary.msg"
  "msg/WorkInstanceDetail.msg"
  "msg/WorkTaskInstanceSummary.msg"
  "msg/WorkTaskInstanceDetail.msg"
  "msg/WorkPrecheckItem.msg"
  "msg/WorkStatusEvent.msg"
  "srv/WorkListTaskTypes.srv"
  "srv/WorkListWorks.srv"
  "srv/WorkGetWork.srv"
  "srv/WorkListRunningWorks.srv"
  "srv/WorkListHistoryWorks.srv"
  "srv/WorkGetWorkInstance.srv"
  "srv/WorkGetTaskInstance.srv"
  "srv/WorkPrecheck.srv"
  "srv/WorkStartWork.srv"
  "srv/WorkStopWork.srv"
  "srv/WorkPauseWork.srv"
  "srv/WorkResumeWork.srv"
)

ament_package()
