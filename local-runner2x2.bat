start javaw -jar "local-runner.jar" local-runner2x2.properties
ping 127.0.0.1 -n 3 > nul
cd "D:\AICup\cpp-cgdk\Debug\"
start /min cpp-cgdk_vs12.exe 127.0.0.1 31001 0000000000000000
start /min cpp-cgdk_vs12.exe 127.0.0.1 31002 0000000000000000