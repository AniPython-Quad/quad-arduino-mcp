# Quad 四足机器人 arduino 代码, 小智MCP语音控制

开发板:
 - arduino安装开发板: ESP32 (by Espressif Systems 3.0.7)  
 - arduino选择开发板: ESP32 Dev Module  
  
库:
 - ESP32Servo (by Kevin Harrington 3.0.8)
 - WebSockets (by Markus Sattler 2.6.1)
 - ArduinoJson (by Benoit Blanchon 7.4.2)

需要修改的地方:  
```C++
const char* WIFI_SSID = "您的wifi名称";  
const char* WIFI_PASS = "您的wifi密码";  
const char* MCP_ENDPOINT = "wss://api.xiaozhi.me/mcp/?token=<您的小智MCP接入点token>";  
```
