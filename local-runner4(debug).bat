start javaw -jar "local-runner.jar" local-runner4.properties
ping 127.0.0.1 -n 3 > nul
cd "C:\Users\ORLEON\Desktop\cpp-cgdk\Debug\"
start /min cpp-cgdk_vs12.exe 127.0.0.1 31002 0000000000000000
start /min cpp-cgdk_vs12.exe 127.0.0.1 31003 0000000000000000
start /min cpp-cgdk_vs12.exe 127.0.0.1 31004 0000000000000000
