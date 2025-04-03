@echo off

REM 定义要删除的目录列表，使用空格分隔
set "folders=./debug ./record"

REM 循环处理每个目录
for %%f in (%folders%) do (
  REM 删除目录下所有文件，不提示确认
  del /q /s "%%f\*"

  REM 删除空目录，不提示确认
  REM rmdir /q "%%f"
)

echo 已删除指定目录及其所有文件。
pause

:exit_case
pause