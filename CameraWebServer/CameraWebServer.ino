#include "esp_camera.h"
#include <WiFi.h>
#include "board_config.h"

// 配置信息
const char* ssid = "REDMI K Pad";
const char* password = "95595555";

// ==================== 固定IP配置 ====================
// STA模式（连接路由器）固定IP配置
IPAddress sta_local_IP(192, 168, 50, 50);     // 固定IP地址
IPAddress sta_gateway(192, 168, 50, 1);       // 网关地址（根据你的路由器修改）
IPAddress sta_subnet(255, 255, 255, 0);       // 子网掩码
IPAddress sta_primaryDNS(8, 8, 8, 8);         // 首选DNS
IPAddress sta_secondaryDNS(8, 8, 4, 4);       // 备用DNS

// AP模式（热点）固定IP配置
IPAddress ap_local_IP(192, 168, 50, 50);      // 热点固定IP地址
IPAddress ap_gateway(192, 168, 50, 1);        // 网关地址
IPAddress ap_subnet(255, 255, 255, 0);        // 子网掩码

// 函数声明
void startCameraServer();
void initCamera();
void initWiFi();
void printCameraInfo();

void setup() {
    // 初始化串口
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    delay(100);
    Serial.println("\n\n==================================");
    Serial.println("ESP32-CAM Web Server Starting");
    Serial.println("==================================");
    
    // 初始化摄像头
    initCamera();
    
    // 打印摄像头信息
    printCameraInfo();
    
    // 配置LED闪光灯（app_httpd.cpp中已有此函数，不需要再定义）
    #if defined(LED_GPIO_NUM)

    Serial.println("✓ LED flash configured");
    #endif
    
    // 连接WiFi
    initWiFi();
    
    // 启动Web服务器
    Serial.println("Starting web server...");
    startCameraServer();
    
    // 启动完成信息
    Serial.println("\n==================================");
    Serial.println("✓ Camera Server Ready!");
    
    // 根据连接模式显示不同的访问地址
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("► Web Interface (STA): http://");
        Serial.println(WiFi.localIP());
        Serial.print("► Stream URL (STA): http://");
        Serial.print(WiFi.localIP());
        Serial.println("/stream");
    } else {
        Serial.print("► Web Interface (AP): http://");
        Serial.println(WiFi.softAPIP());
        Serial.print("► Stream URL (AP): http://");
        Serial.print(WiFi.softAPIP());
        Serial.println("/stream");
    }
    Serial.println("==================================\n");
}

void loop() {
    // 定期检查WiFi连接状态
    static unsigned long lastCheck = 0;
    unsigned long now = millis();
    
    if (now - lastCheck > 30000) {  // 每30秒检查一次
        lastCheck = now;
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected! Reconnecting...");
            WiFi.reconnect();
        } else {
            // 可选：打印内存使用情况
            Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
            if (psramFound()) {
                Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());
            }
        }
    }
    
    delay(1000);
}

// 初始化摄像头
void initCamera() {
    Serial.println("\n--- Camera Initialization ---");
    
    // 创建摄像头配置结构体
    camera_config_t config;
    
    // 引脚配置
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    
    // XCLK频率
    config.xclk_freq_hz = 20000000;
    
    // 图像格式设置
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_QVGA;
    
    // JPEG质量
    config.jpeg_quality = 12;
    
    // 帧缓冲区数量
    config.fb_count = 1;
    
    // 抓取模式
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    
    // 帧缓冲区位置
    config.fb_location = CAMERA_FB_IN_PSRAM;
    
    // 检测PSRAM并优化设置
    if (psramFound()) {
        Serial.println("✓ PSRAM detected");
        config.jpeg_quality = 10;
        config.fb_count = 2;
        config.grab_mode = CAMERA_GRAB_LATEST;
        config.frame_size = FRAMESIZE_VGA;
    } else {
        Serial.println("⚠ No PSRAM detected, using limited settings");
        config.frame_size = FRAMESIZE_QVGA;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.fb_count = 1;
    }
    
    // 初始化摄像头
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("✗ Camera init failed with error 0x%x\n", err);
        
        // 尝试使用保守设置重新初始化
        Serial.println("Retrying with conservative settings...");
        
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        
        err = esp_camera_init(&config);
        if (err != ESP_OK) {
            Serial.printf("✗ Camera init failed again: 0x%x\n", err);
            Serial.println("System halted!");
            while (1) {
                delay(1000);
            }
        }
    }
    
    Serial.println("✓ Camera initialized successfully");
    
    // 测试摄像头捕获
    Serial.print("Testing camera capture... ");
    camera_fb_t* test_fb = esp_camera_fb_get();
    if (test_fb) {
        Serial.printf("✓ OK (%u bytes)\n", test_fb->len);
        esp_camera_fb_return(test_fb);
    } else {
        Serial.println("✗ Failed! Check camera connection");
    }
}

// 初始化WiFi连接
void initWiFi() {
    Serial.println("\n--- WiFi Connection ---");
    Serial.printf("Connecting to: %s\n", ssid);
    
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    
    // 配置STA模式固定IP为192.168.50.50
    Serial.println("Configuring static IP: 192.168.50.50");
    if (WiFi.config(sta_local_IP, sta_gateway, sta_subnet, sta_primaryDNS, sta_secondaryDNS)) {
        Serial.println("✓ Static IP configured successfully");
        Serial.print("  IP Address: ");
        Serial.println(sta_local_IP);
        Serial.print("  Gateway: ");
        Serial.println(sta_gateway);
        Serial.print("  Subnet Mask: ");
        Serial.println(sta_subnet);
    } else {
        Serial.println("⚠ Failed to configure static IP, using DHCP");
    }
    
    WiFi.begin(ssid, password);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
        
        if (attempts % 10 == 0) {
            Serial.printf("\nStill trying... (%d/30)\n", attempts);
        }
    }
    
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("✓ WiFi connected successfully!");
        Serial.print("  IP Address: ");
        Serial.println(WiFi.localIP());
        
        // 验证IP地址是否设置成功
        if (WiFi.localIP() == sta_local_IP) {
            Serial.println("  ✓ Static IP verified: 192.168.50.50");
        } else {
            Serial.print("  ⚠ Warning: Expected IP ");
            Serial.print(sta_local_IP);
            Serial.print(" but got ");
            Serial.println(WiFi.localIP());
        }
        
        Serial.print("  Gateway: ");
        Serial.println(WiFi.gatewayIP());
        Serial.print("  Subnet Mask: ");
        Serial.println(WiFi.subnetMask());
        Serial.print("  DNS Server: ");
        Serial.println(WiFi.dnsIP());
        Serial.print("  Signal strength: ");
        Serial.print(WiFi.RSSI());
        Serial.println(" dBm");
        Serial.print("  MAC Address: ");
        Serial.println(WiFi.macAddress());
    } else {
        Serial.println("✗ WiFi connection failed!");
        Serial.println("  Creating access point instead...");
        
        // 切换到AP模式并设置固定IP为192.168.50.50
        WiFi.mode(WIFI_AP);
        
        // 配置AP模式的固定IP地址
        if (WiFi.softAPConfig(ap_local_IP, ap_gateway, ap_subnet)) {
            Serial.println("✓ Static IP configured for AP mode: 192.168.50.50");
        } else {
            Serial.println("⚠ Failed to configure static IP for AP mode");
        }
        
        // 创建热点
        WiFi.softAP("ESP32-CAM", "12345678");
        Serial.print("  AP SSID: ESP32-CAM");
        Serial.println("  AP Password: 12345678");
        Serial.print("  AP IP Address: ");
        Serial.println(WiFi.softAPIP());
        
        // 验证AP IP地址
        if (WiFi.softAPIP() == ap_local_IP) {
            Serial.println("  ✓ AP static IP verified: 192.168.50.50");
        } else {
            Serial.print("  ⚠ Warning: Expected AP IP ");
            Serial.print(ap_local_IP);
            Serial.print(" but got ");
            Serial.println(WiFi.softAPIP());
        }
    }
}

// 打印摄像头信息
void printCameraInfo() {
    Serial.println("\n--- Camera Information ---");
    sensor_t* s = esp_camera_sensor_get();
    
    if (s) {
        // 打印传感器ID
        Serial.printf("Sensor Model: 0x%x (PID:0x%x)\n", s->id.PID, s->id.PID);
        
        // 根据PID识别传感器型号
        const char* sensor_name = "Unknown";
        if (s->id.PID == OV2640_PID) sensor_name = "OV2640";
        else if (s->id.PID == OV3660_PID) sensor_name = "OV3660";
        else if (s->id.PID == OV5640_PID) sensor_name = "OV5640";
        else if (s->id.PID == OV7725_PID) sensor_name = "OV7725";
        
        Serial.printf("Sensor Type: %s\n", sensor_name);
        
        // 打印当前配置
        Serial.printf("Pixel Format: %s\n", 
            s->pixformat == PIXFORMAT_JPEG ? "JPEG" : 
            s->pixformat == PIXFORMAT_GRAYSCALE ? "GRAYSCALE" : "RGB");
        Serial.printf("Frame Size: %d\n", s->status.framesize);
        Serial.printf("JPEG Quality: %d\n", s->status.quality);
        
        // 应用传感器特定设置
        if (s->id.PID == OV3660_PID) {
            Serial.println("Applying OV3660 specific settings...");
            s->set_vflip(s, 1);
            s->set_brightness(s, 1);
            s->set_saturation(s, -2);
        }
        
        #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
        s->set_vflip(s, 1);
        s->set_hmirror(s, 1);
        #endif
        
        #if defined(CAMERA_MODEL_ESP32S3_EYE)
        s->set_vflip(s, 1);
        #endif
        
        // 设置合适的帧大小
        s->set_framesize(s, FRAMESIZE_VGA);
        
        Serial.println("✓ Camera settings applied");
    } else {
        Serial.println("✗ Failed to get sensor info");
    }
}

// 注意：setupLedFlash() 函数在 app_httpd.cpp 中已经定义，这里不需要再定义
// 如果你需要自定义LED闪光灯行为，请修改 app_httpd.cpp 中的 setupLedFlash() 函数