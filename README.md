# Quad 四足机器人 arduino 代码, 小智MCP语音控制

#### 淘宝链接:  
小智Ai对话机器人: https://item.taobao.com/item.htm?ft=t&id=973456274597  
四足机器人: https://item.taobao.com/item.htm?ft=t&id=888872820389

#### 说明:   
安装开发板和依赖库版本的时候, 直接安装最新版就可以, 不一定按下面这些版本  
如果安装最新版有问题, 再尝试回退到以下提供的版本  
以下提供的版本是测试过可用的  

#### ESP32开发板 (不带C, 不带S):
 - arduino安装开发板: ESP32 (by Espressif Systems 3.3.0)  
 - arduino选择开发板: ESP32 Dev Module  

#### 依赖库:
 - ESP32Servo (by Kevin Harrington 3.0.9)
 - WebSockets (by Markus Sattler 2.7.0)
 - ArduinoJson (by Benoit Blanchon 7.4.2)
 - xiaozhi-mcp (by toddpan 1.0.0)

#### 需要修改的地方:  
```C++
// main.cpp
const char* WIFI_SSID = "您的wifi名称";  
const char* WIFI_PASS = "您的wifi密码";  
const char* MCP_ENDPOINT = "wss://api.xiaozhi.me/mcp/?token=<您的小智MCP接入点token>";  
```

#### 补充:  
小智AI官网: https://xiaozhi.me/    
小智AI烧录固件, 设备配网, 绑定设备等操作可以参考小智AI`DIY教程`    
