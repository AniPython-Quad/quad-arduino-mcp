/**
 * arduino安装开发板: ESP32 (by Espressif Systems 3.0.7)
 * arduino选择开发板: ESP32 Dev Module
 *
 * 库:
 *  - ESP32Servo (by Kevin Harrington 3.0.8)
 *  - WebSockets (by Markus Sattler 2.6.1)
 *  - ArduinoJson (by Benoit Blanchon 7.4.2)
 *
 * 舵机与 esp32 引脚接线图, 数据口方向为后 (tail)
 * 可以在 minikame.cpp 文中的 MiniKame::init() 方法中修改
 *
 *     前 (head)
 *         -----               -----
 *         |  2  |             |  3  |
 *         |pin25|             |Pin18|
 *         ----- -----   ----- -----
 *             |  0  | |  1  |
 *             |Pin12| |Pin16|
 *              -----   -----
 *             |  4  | |  5  |
 *             |Pin13| |Pin17|
 *         ----- -----   ----- -----
 *         |  6  |             |  7  |
 *         |Pin26|             |Pin19|
 *         -----               -----
 *     后 (tail)
 */

#include <Arduino.h>
#include <WiFi.h>
#include "WebSocketMCP.h"
#include "minikame.h"

/********** 配置项 ***********/
// WiFi设置
const char* WIFI_SSID = "小亦站";
const char* WIFI_PASS = "88889999";

// WebSocket MCP服务器地址 访问ai小智官网获取 https://xiaozhi.me/
const char* MCP_ENDPOINT = "ws://api.xiaozhi.me/mcp/?token=eyJhbGciOiJFUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VySWQiOjIwNDEzNywiYWdlbnRJZCI6NDc0MTA2LCJlbmRwb2ludElkIjoiYWdlbnRfNDc0MTA2IiwicHVycG9zZSI6Im1jcC1lbmRwb2ludCIsImlhdCI6MTc1MjgwODE4Mn0.1bCPzso9QVj7Jp9QExYIDrPDbYIOLOxzJcxE13jD95erJUd66BcYstvsNaQ9WGdYLJ06uAu-fnfXCiNQLNdduw924";

// 调试信息
#define DEBUG_SERIAL Serial
#define DEBUG_BAUD_RATE 115200

// LED控制引脚定义（用于状态指示和工具控制）
#define LED_PIN 2  // ESP32板载LED

/********** 全局变量 ***********/
WebSocketMCP mcpClient;

// 机器人实例
MiniKame robot;
// test_action_mode 为 true 时，机器人将执行测试动作
bool testActionMode = false;

const int periodDefault[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // 动作周期，单位：毫秒
constexpr int amplitudeDefault[8] = {}; // 动作幅度，范围：0-180
const int offsetDefault[8] = {90, 90, 90, 90, 90, 90, 90, 90}; // 动作偏移，90表示不偏移, 范围：0-180
constexpr int phaseDefault[8] = {}; // 动作相位，范围：0-360

int period[8] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000}; // 周期，单位：毫秒
int amplitude[8] = {}; // 振幅，范围：0-180
int offset[8] = {90, 90, 90, 90, 90, 90, 90, 90}; // 偏移，90表示不偏移, 范围：0-180
int phase[8] = {}; // 相位，范围：0-360

// 缓冲区管理
#define MAX_INPUT_LENGTH 1024
char inputBuffer[MAX_INPUT_LENGTH];
int inputBufferIndex = 0;
bool newCommandAvailable = false;

// 连接状态
bool wifiConnected = false;
bool mcpConnected = false;

/********** 函数声明 ***********/
void setupWifi();
void onMcpOutput(const String& message);
void onMcpError(const String& error);
void onMcpConnectionChange(bool connected);
void processSerialCommands();
void blinkLed(int times, int delayMs);
void registerMcpTools();
void printHelp();
void printStatus();
void printOscillator();

void setup()
{
    // 初始化串口
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    DEBUG_SERIAL.println("\n\n[ESP32 MCP客户端] 初始化...");

    // 初始化LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);


    // 连接WiFi
    setupWifi();

    // 初始化MCP客户端
    if (mcpClient.begin(MCP_ENDPOINT, onMcpConnectionChange))
    {
        DEBUG_SERIAL.println("[ESP32 MCP客户端] 初始化成功，尝试连接到MCP服务器...");
    }
    else
    {
        DEBUG_SERIAL.println("[ESP32 MCP客户端] 初始化失败!");
    }

    // 显示帮助信息
    DEBUG_SERIAL.println("\n使用说明:");
    DEBUG_SERIAL.println("- 通过串口控制台输入命令并回车发送");
    DEBUG_SERIAL.println("- 从MCP服务器接收的消息将显示在串口控制台上");
    DEBUG_SERIAL.println("- 输入\"help\"查看可用命令");
    DEBUG_SERIAL.println();

    // 初始化机器人
    delay(10);
    robot.init();
    robot.home();
    DEBUG_SERIAL.println("初始化机器人完成");
}

void loop()
{
    // 处理MCP客户端
    mcpClient.loop();

    // 处理来自串口的命令
    processSerialCommands();

    // 状态LED显示
    if (!wifiConnected)
    {
        // WiFi未连接: 快速闪烁
        blinkLed(1, 100);
    }
    else if (!mcpConnected)
    {
        // WiFi已连接但MCP未连接: 慢闪
        blinkLed(1, 500);
    }
    else
    {
        // 全部连接成功: LED亮起
        digitalWrite(LED_PIN, HIGH);
    }
}

/**
 * 设置WiFi连接
 */
void setupWifi()
{
    DEBUG_SERIAL.print("[WiFi] 连接到 ");
    DEBUG_SERIAL.println(WIFI_SSID);

    // 开始连接
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // 等待连接
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20)
    {
        delay(500);
        DEBUG_SERIAL.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        wifiConnected = true;
        DEBUG_SERIAL.println();
        DEBUG_SERIAL.println("[WiFi] 连接成功!");
        DEBUG_SERIAL.print("[WiFi] IP地址: ");
        DEBUG_SERIAL.println(WiFi.localIP());
    }
    else
    {
        wifiConnected = false;
        DEBUG_SERIAL.println();
        DEBUG_SERIAL.println("[WiFi] 连接失败! 将继续尝试...");
    }
}

/**
 * MCP输出回调函数(stdout替代)
 */
void onMcpOutput(const String& message)
{
    DEBUG_SERIAL.print("[MCP输出] ");
    DEBUG_SERIAL.println(message);
}

/**
 * MCP错误回调函数(stderr替代)
 */
void onMcpError(const String& error)
{
    DEBUG_SERIAL.print("[MCP错误] ");
    DEBUG_SERIAL.println(error);
}


/**
 * MCP连接状态变化回调函数
 */
void onMcpConnectionChange(bool connected)
{
    mcpConnected = connected;
    if (connected)
    {
        DEBUG_SERIAL.println("[MCP] 已连接到MCP服务器");
        // 连接成功后注册工具
        registerMcpTools();
    }
    else
    {
        DEBUG_SERIAL.println("[MCP] 与MCP服务器断开连接");
    }
}

/**
 * 处理来自串口的命令
 */
void processSerialCommands()
{
    // 检查是否有串口数据
    while (DEBUG_SERIAL.available() > 0)
    {
        char inChar = (char)DEBUG_SERIAL.read();

        // 处理回车或换行
        if (inChar == '\n' || inChar == '\r')
        {
            if (inputBufferIndex > 0)
            {
                // 添加字符串结束符
                inputBuffer[inputBufferIndex] = '\0';

                // 处理命令
                String command = String(inputBuffer);
                command.trim();

                if (command.length() > 0)
                {
                    if (command == "help")
                    {
                        printHelp();
                    }
                    else if (command == "status")
                    {
                        printStatus();
                    }
                    else if (command == "reconnect")
                    {
                        DEBUG_SERIAL.println("正在重新连接...");
                        mcpClient.disconnect();
                    }
                    else if (command == "tools")
                    {
                        // 显示已注册工具
                        DEBUG_SERIAL.println("已注册工具数量: " + String(mcpClient.getToolCount()));
                    }
                    else
                    {
                        // 将命令发送到MCP服务器(stdin替代)
                        if (mcpClient.isConnected())
                        {
                            mcpClient.sendMessage(command);
                            DEBUG_SERIAL.println("[发送] " + command);
                        }
                        else
                        {
                            DEBUG_SERIAL.println("未连接到MCP服务器，无法发送命令");
                        }
                    }
                }

                // 重置缓冲区
                inputBufferIndex = 0;
            }
        }
        // 处理退格键
        else if (inChar == '\b' || inChar == 127)
        {
            if (inputBufferIndex > 0)
            {
                inputBufferIndex--;
                DEBUG_SERIAL.print("\b \b"); // 退格、空格、再退格
            }
        }
        // 处理普通字符
        else if (inputBufferIndex < MAX_INPUT_LENGTH - 1)
        {
            inputBuffer[inputBufferIndex++] = inChar;
            DEBUG_SERIAL.print(inChar); // Echo
        }
    }
}


/**
 * 打印当前状态
 */
void printStatus()
{
    DEBUG_SERIAL.println("当前状态:");
    DEBUG_SERIAL.print("  WiFi: ");
    DEBUG_SERIAL.println(wifiConnected ? "已连接" : "未连接");
    if (wifiConnected)
    {
        DEBUG_SERIAL.print("  IP地址: ");
        DEBUG_SERIAL.println(WiFi.localIP());
        DEBUG_SERIAL.print("  信号强度: ");
        DEBUG_SERIAL.println(WiFi.RSSI());
    }
    DEBUG_SERIAL.print("  MCP服务器: ");
    DEBUG_SERIAL.println(mcpConnected ? "已连接" : "未连接");
}

/**
 * LED闪烁函数
 */
void blinkLed(int times, int delayMs)
{
    static int blinkCount = 0;
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;
    static int lastTimes = 0;

    if (times == 0)
    {
        digitalWrite(LED_PIN, LOW);
        blinkCount = 0;
        lastTimes = 0;
        return;
    }
    if (lastTimes != times)
    {
        blinkCount = 0;
        lastTimes = times;
        ledState = false;
        lastBlinkTime = millis();
    }
    unsigned long now = millis();
    if (blinkCount < times * 2)
    {
        if (now - lastBlinkTime > delayMs)
        {
            lastBlinkTime = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState ? HIGH : LOW);
            blinkCount++;
        }
    }
    else
    {
        digitalWrite(LED_PIN, LOW);
        blinkCount = 0;
        lastTimes = 0;
    }
}


/**
 * 打印帮助信息
 */
void printHelp()
{
    DEBUG_SERIAL.println("可用命令:");
    DEBUG_SERIAL.println("  help     - 显示此帮助信息");
    DEBUG_SERIAL.println("  status   - 显示当前连接状态");
    DEBUG_SERIAL.println("  reconnect - 重新连接到MCP服务器");
    DEBUG_SERIAL.println("  tools    - 查看已注册工具");
    DEBUG_SERIAL.println("  其他任何文本将直接发送到MCP服务器");
}

void printOscillator()
{
    for (int i = 0; i < 8; i++)
    {
        DEBUG_SERIAL.print("index " + String(i) + ": ");
        DEBUG_SERIAL.print("period(周期): ");
        DEBUG_SERIAL.print(period[i]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(" amplitude(振幅): ");
        DEBUG_SERIAL.print(amplitude[i]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(" offset(偏移): ");
        DEBUG_SERIAL.print(offset[i]);
        DEBUG_SERIAL.print("\t");
        DEBUG_SERIAL.print(" phase(相位): ");
        DEBUG_SERIAL.println(phase[i]);
    }
}

/**
 * 注册MCP工具
 * 在连接成功后注册工具
 */
void registerMcpTools()
{
    DEBUG_SERIAL.println("[MCP] 注册工具...");

    // 注册系统信息工具
    mcpClient.registerTool(
        "system-info",
        "获取ESP32系统信息",
        "{\"properties\":{},\"title\":\"systemInfoArguments\",\"type\":\"object\"}",
        [](const String& args)
        {
            DEBUG_SERIAL.println("[获取ESP32系统信息: " + args);
            // 收集系统信息
            String chipModel = ESP.getChipModel();
            uint32_t chipId = ESP.getEfuseMac() & 0xFFFFFFFF;
            uint32_t flashSize = ESP.getFlashChipSize() / 1024;
            uint32_t freeHeap = ESP.getFreeHeap() / 1024;

            // 构造JSON响应
            String resultJson = "{\"success\":true,\"model\":\"" + chipModel + "\",\"chipId\":\"" + String(chipId, HEX)
                +
                "\",\"flashSize\":" + String(flashSize) + ",\"freeHeap\":" + String(freeHeap) +
                ",\"wifiStatus\":\"" + (WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") +
                "\",\"ipAddress\":\"" + WiFi.localIP().toString() + "\"}";

            return WebSocketMCP::ToolResponse(resultJson);
        }
    );
    DEBUG_SERIAL.println("[MCP] 系统信息工具已注册");

    // 注册四足机器人工具
    mcpClient.registerTool(
        "robot",
        "遥控机器人指令",
        "{\"properties\":{\"command\":{\"title\":\"机器人指令\",\"type\":\"string\",\"enum\":[\"hello\",\"forward\",\"backward\",\"turn_left\",\"turn_right\",\"moonwalk\",\"push_up\",\"stand_at_attention\",\"execute_test_oscillator\",\"reset_oscillator\"]},\"steps\":{\"title\":\"步数\",\"type\":\"number\",\"minimum\":0}},\"required\":[\"command\"],\"title\":\"robotControlArguments\",\"type\":\"object\"}",
        [](const String& args)
        {
            String resultJson = "{}";
            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, args);

            if (error)
            {
                // 返回错误响应
                resultJson = "{\"success\":false,\"error\":\"无效的参数格式\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }

            String command = doc["command"].as<String>();
            float steps = doc["steps"];
            if (steps == 0.0) { steps = 2.0; };
            if (command == "hello")
            {
                robot.hello();
                resultJson = "{\"success\":true,\"command\":\"hello\",\"steps\":\"0\"}";
            }
            else if (command == "forward")
            {
                robot.forward(steps, 1000);
                resultJson = "{\"success\":true,\"command\":\"forward\",\"steps\":\"" + String(steps) + "\"}";
            }
            else if (command == "backward")
            {
                robot.backward(steps, 1000);
                resultJson = "{\"success\":true,\"command\":\"backward\",\"steps\":\"" + String(steps) + "\"}";
            }
            else if (command == "turn_left")
            {
                robot.turnL(steps, 1000);
                resultJson = "{\"success\":true,\"command\":\"turnL\",\"steps\":\"" + String(steps) + "\"}";
            }
            else if (command == "turn_right")
            {
                robot.turnR(steps, 1000);
                resultJson = "{\"success\":true,\"command\":\"turnR\",\"steps\":\"" + String(steps) + "\"}";
            }
            else if (command == "moonwalk")
            {
                robot.moonwalkL(4, 2000);
                resultJson = "{\"success\":true,\"command\":\"moonwalkL\",\"steps\":\"" + String(steps) + "\"}";
            }
            else if (command == "push_up")
            {
                robot.pushUp(steps, 1600);
                resultJson = "{\"success\":true,\"command\":\"pushUp\",\"steps\":\"" + String(steps) + "\"}";
            }
            else if (command == "stand_at_attention")
            {
                robot.home();
                resultJson = "{\"success\":true,\"command\":\"home\",\"steps\":\"" + String(steps) + "\"}";
            }
            else if (command == "execute_test_oscillator")
            {
                robot.execute(steps, period, amplitude, offset, phase);
                resultJson = "{\"success\":true,\"command\":\"execute_test_oscillator\",\"steps\":\"" + String(steps) +
                    "\"}";
            }
            else if (command == "reset_oscillator")
            {
                for (int i = 0; i < 8; i++)
                {
                    period[i] = periodDefault[i];
                    amplitude[i] = amplitudeDefault[i];
                    offset[i] = offsetDefault[i];
                    phase[i] = phaseDefault[i];
                }
                printOscillator();
                resultJson = "{\"success\":true,\"command\":\"reset_oscillator\"}";
            }
            else
            {
                resultJson = "{\"success\":false,\"error\":\"无效的指令\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }

            return WebSocketMCP::ToolResponse(resultJson);
        }
    );

    // 注册舵机控制工具
    mcpClient.registerTool(
        "robot.servo_control", // 工具名称
        "使用语音单独控制每个舵机的角度。舵机索引对应关系：0 - 左前大腿，1 - 右前大腿，2 - 左前小腿，3 - 右前小腿，4 - 左后大腿，5 - 右后大腿，6 - 左后小腿，7 - 右后小腿", // 工具描述
        "{\"properties\":{\"servo_index\":{\"title\":\"舵机索引（0 - 左前大腿，1 - 右前大腿，2 - 左前小腿，3 - 右前小腿，4 - 左后大腿，5 - 右后大腿，6 - 左后小腿，7 - 右后小腿）\",\"type\":\"integer\",\"minimum\":0,\"maximum\":7},\"angle\":{\"title\":\"舵机角度\",\"type\":\"integer\",\"minimum\":0,\"maximum\":180}},\"required\":[\"servo_index\",\"angle\"],\"title\":\"servoControlArguments\",\"type\":\"object\"}",
        [](const String& args)
        {
            DEBUG_SERIAL.println("[工具] 舵机控制: " + args);
            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, args);

            if (error)
            {
                // 返回错误响应
                String resultJson = "{\"success\":false,\"error\":\"无效的参数格式\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }

            int servoIndex = doc["servo_index"].as<int>();
            int angle = doc["angle"].as<int>();
            if (servoIndex >= 0 && servoIndex < 8 && angle >= 0 && angle <= 180)
            {
                robot.setServo(servoIndex, angle);
                String resultJson = "{\"success\":true,\"servo_index\":" + String(servoIndex) + ",\"angle\":" +
                    String(angle) + "}";
                return WebSocketMCP::ToolResponse(resultJson);
            }
            else
            {
                String resultJson = "{\"success\":false,\"error\":\"舵机索引或角度超出范围\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }
        }
    );
    DEBUG_SERIAL.println("[MCP] 舵机控制工具已注册");

    // 注册振荡器参数设置工具
    mcpClient.registerTool(
        "robot.oscillator_settings", // 工具名称
        "设置振荡器参数", // 工具描述
        "{\"properties\":{\"servo_index\":{\"title\":\"舵机索引（0 - 左前大腿，1 - 右前大腿，2 - 左前小腿，3 - 右前小腿，4 - 左后大腿，5 - 右后大腿，6 - 左后小腿，7 - 右后小腿）\",\"type\":\"integer\",\"minimum\":0,\"maximum\":7},\"arg_name\":{\"title\":\"舵机参数名称\",\"enum\":[\"period\",\"amplitude\",\"offset\",\"phase\"],\"type\":\"string\"},\"value\":{\"title\":\"值\",\"type\":\"integer\"}},\"required\":[\"servo_index\",\"arg_name\",\"value\"],\"title\":\"servoOscillatorArguments\",\"type\":\"object\"}",
        // 输入schema
        [](const String& args)
        {
            DEBUG_SERIAL.println("[工具] 设置振荡器参数: " + args);
            DynamicJsonDocument doc(1024); // 增大文档大小以容纳数组
            DeserializationError error = deserializeJson(doc, args);

            if (error)
            {
                // 返回错误响应
                String resultJson = "{\"success\":false,\"error\":\"无效的参数格式\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }

            // 提取参数并更新全局变量
            int servo_index = doc["servo_index"].as<int>();
            String arg_name = doc["arg_name"].as<String>();
            int value = doc["value"].as<int>();

            if (arg_name == "period") { period[servo_index] = value; }
            else if (arg_name == "amplitude") { amplitude[servo_index] = value; }
            else if (arg_name == "offset") { offset[servo_index] = value; }
            else if (arg_name == "phase") { phase[servo_index] = value; }

            // 返回成功响应
            String resultJson = "{\"success\":true,\"message\":\"振荡器参数设置成功\"}";

            printOscillator();
            return WebSocketMCP::ToolResponse(resultJson);
        }
    );
    DEBUG_SERIAL.println("[MCP] 振荡器参数设置工具已注册");

    // 在registerMcpTools()函数中添加以下代码
    // 注册舵机校准补偿设置工具
    mcpClient.registerTool(
        "robot.servo_trim", // 工具名称
        "设置舵机校准补偿值，支持绝对值设置和相对调整", // 工具描述
        "{\"properties\":{\"servo_index\":{\"title\":\"舵机索引(0-7),(0 - 左前大腿，1 - 右前大腿，2 - 左前小腿，3 - 右前小腿，4 - 左后大腿，5 - 右后大腿，6 - 左后小腿，7 - 右后小腿)\",\"type\":\"integer\",\"minimum\":0,\"maximum\":7},\"mode\":{\"title\":\"设置模式\",\"type\":\"string\",\"enum\":[\"set\",\"adjust\"]},\"value\":{\"title\":\"数值（绝对值或调整量）\",\"type\":\"integer\",\"minimum\":-90,\"maximum\":90}},\"required\":[\"servo_index\",\"mode\",\"value\"],\"title\":\"servoTrimArguments\",\"type\":\"object\"}",
        [](const String& args)
        {
            DEBUG_SERIAL.println("[工具] 舵机校准补偿设置: " + args);
            DynamicJsonDocument doc(256);
            DeserializationError error = deserializeJson(doc, args);

            if (error)
            {
                // 返回错误响应
                String resultJson = "{\"success\":false,\"error\":\"无效的参数格式\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }

            int servoIndex = doc["servo_index"].as<int>();
            String mode = doc["mode"].as<String>();
            int value = doc["value"].as<int>();

            if (servoIndex < 0 || servoIndex >= 8)
            {
                String resultJson = "{\"success\":false,\"error\":\"舵机索引超出范围（0-7）\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }

            int oldValue = robot.getTrim(servoIndex);
            int newValue = oldValue;

            if (mode == "set")
            {
                // 直接设置值
                newValue = value;
                if (newValue < -90 || newValue > 90)
                {
                    String resultJson = "{\"success\":false,\"error\":\"校准值超出范围（-90到90）\"}";
                    return WebSocketMCP::ToolResponse(resultJson, true);
                }
            }
            else if (mode == "adjust")
            {
                // 相对调整值
                newValue = oldValue + value;
                newValue = constrain(newValue, -90, 90); // 限制在有效范围内
            }
            else
            {
                String resultJson = "{\"success\":false,\"error\":\"无效的模式，应为'set'或'adjust'\"}";
                return WebSocketMCP::ToolResponse(resultJson, true);
            }

            // 应用校准补偿到舵机
            robot.setTrim(servoIndex, newValue);
            robot.home();

            String resultJson = "{\"success\":true,\"servo_index\":" + String(servoIndex) +
                ",\"old_value\":" + String(oldValue) +
                ",\"new_value\":" + String(newValue) +
                ",\"mode\":\"" + mode + "\"" +
                ",\"message\":\"舵机校准补偿设置成功\"}";
            return WebSocketMCP::ToolResponse(resultJson);
        }
    );
    DEBUG_SERIAL.println("[MCP] 舵机校准补偿工具已注册");

    // 注册查看舵机校准补偿工具
    mcpClient.registerTool(
        "robot.get_servo_trim", // 工具名称
        "获取当前所有舵机的校准补偿值", // 工具描述
        "{\"properties\":{},\"title\":\"getServoTrimArguments\",\"type\":\"object\"}",
        [](const String& args)
        {
            DEBUG_SERIAL.println("[工具] 获取舵机校准补偿值");

            // 构造包含所有校准值的JSON响应
            String trimArray = "[";
            for (int i = 0; i < 8; i++)
            {
                if (i > 0) trimArray += ",";
                trimArray += String(robot.getTrim(i));
            }
            trimArray += "]";

            String resultJson = "{\"success\":true,\"trim_values\":" + trimArray + "}";
            return WebSocketMCP::ToolResponse(resultJson);
        }
    );
    DEBUG_SERIAL.println("[MCP] 获取舵机校准补偿工具已注册");


    DEBUG_SERIAL.println("[MCP] 工具注册完成，共" + String(mcpClient.getToolCount()) + "个工具");
}
