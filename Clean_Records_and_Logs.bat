@echo off

REM ����Ҫɾ����Ŀ¼�б�ʹ�ÿո�ָ�
set "folders=./debug ./record"

REM ѭ������ÿ��Ŀ¼
for %%f in (%folders%) do (
  REM ɾ��Ŀ¼�������ļ�������ʾȷ��
  del /q /s "%%f\*"

  REM ɾ����Ŀ¼������ʾȷ��
  REM rmdir /q "%%f"
)

echo ��ɾ��ָ��Ŀ¼���������ļ���
pause

:exit_case
pause