// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 引入必要的头文件
#include "esp_http_server.h"    // HTTP服务器功能
#include "esp_timer.h"          // 定时器功能
#include "esp_camera.h"         // 摄像头驱动
#include "img_converters.h"     // 图像格式转换
#include "fb_gfx.h"             // 帧缓冲区图形处理
#include "esp32-hal-ledc.h"     // LED PWM控制
#include "sdkconfig.h"          // SDK配置
#include "camera_index.h"       // 摄像头网页索引文件
#include "board_config.h"       // 开发板配置

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"      // 日志功能
#endif

// ==================== LED闪光灯配置 ====================
#if defined(LED_GPIO_NUM)        // 如果定义了LED引脚
#define CONFIG_LED_MAX_INTENSITY 255  // LED最大亮度

int led_duty = 0;                // LED占空比（亮度）
bool isStreaming = true;        // 是否正在视频流传输

#endif

// ==================== JPEG分块传输结构体 ====================
typedef struct {
  httpd_req_t *req;              // HTTP请求对象
  size_t len;                    // 数据长度
} jpg_chunking_t;

// ==================== 视频流边界定义 ====================
#define PART_BOUNDARY "123456789000000000000987654321"  // 多部分边界字符串
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;  // 流内容类型
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";  // 流边界
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";  // 流部分格式

// HTTP服务器句柄
httpd_handle_t stream_httpd = NULL;   // 视频流服务器句柄
httpd_handle_t camera_httpd = NULL;   // 摄像头控制服务器句柄

// ==================== 运行平均滤波器结构体 ====================
typedef struct {
  size_t size;     // 滤波使用的数值数量
  size_t index;    // 当前数值索引
  size_t count;    // 数值计数
  int sum;         // 总和
  int *values;     // 存储数值的数组
} ra_filter_t;

static ra_filter_t ra_filter;  // 运行平均滤波器实例

// 初始化运行平均滤波器
static ra_filter_t *ra_filter_init(ra_filter_t *filter, size_t sample_size) {
  memset(filter, 0, sizeof(ra_filter_t));  // 清空结构体

  filter->values = (int *)malloc(sample_size * sizeof(int));  // 分配内存
  if (!filter->values) {
    return NULL;  // 内存分配失败
  }
  memset(filter->values, 0, sample_size * sizeof(int));  // 初始化数组

  filter->size = sample_size;  // 设置采样大小
  return filter;
}

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
// 运行平均滤波器处理函数
static int ra_filter_run(ra_filter_t *filter, int value) {
  if (!filter->values) {
    return value;  // 如果数组未初始化，直接返回值
  }
  filter->sum -= filter->values[filter->index];           // 减去旧值
  filter->values[filter->index] = value;                  // 存储新值
  filter->sum += filter->values[filter->index];           // 加上新值
  filter->index++;                                         // 移动索引
  filter->index = filter->index % filter->size;           // 循环索引
  if (filter->count < filter->size) {
    filter->count++;                                       // 增加计数直到达到采样大小
  }
  return filter->sum / filter->count;                      // 返回平均值
}
#endif

#if defined(LED_GPIO_NUM)
// 启用/禁用LED闪光灯
void enable_led(bool en) {
  int duty = en ? led_duty : 0;  // 根据状态设置占空比
  if (en && isStreaming && (led_duty > CONFIG_LED_MAX_INTENSITY)) {
    duty = CONFIG_LED_MAX_INTENSITY;  // 限制最大亮度
  }
  ledcWrite(LED_GPIO_NUM, duty);  // 设置LED PWM占空比
  log_i("Set LED intensity to %d", duty);  // 日志输出
}
#endif

// ==================== BMP图像处理函数 ====================
static esp_err_t bmp_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_start = esp_timer_get_time();  // 记录开始时间
#endif
  fb = esp_camera_fb_get();  // 获取摄像头帧
  if (!fb) {
    log_e("Camera capture failed");  // 摄像头捕获失败
    httpd_resp_send_500(req);        // 发送500错误
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/x-windows-bmp");  // 设置响应类型为BMP
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");  // 设置文件名
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);  // 格式化时间戳
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);  // 设置时间戳头

  uint8_t *buf = NULL;
  size_t buf_len = 0;
  bool converted = frame2bmp(fb, &buf, &buf_len);  // 将帧转换为BMP格式
  esp_camera_fb_return(fb);  // 归还帧缓冲区
  if (!converted) {
    log_e("BMP Conversion failed");  // BMP转换失败
    httpd_resp_send_500(req);        // 发送500错误
    return ESP_FAIL;
  }
  res = httpd_resp_send(req, (const char *)buf, buf_len);  // 发送BMP数据
  free(buf);  // 释放缓冲区
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  uint64_t fr_end = esp_timer_get_time();  // 记录结束时间
#endif
  log_i("BMP: %llums, %uB", (uint64_t)((fr_end - fr_start) / 1000), buf_len);  // 日志输出
  return res;
}

// ==================== JPEG编码流回调函数 ====================
static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;  // 重置长度
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;  // 发送失败
  }
  j->len += len;  // 累加长度
  return len;
}

// ==================== 拍照处理函数 ====================
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_start = esp_timer_get_time();  // 记录开始时间
#endif

#if defined(LED_GPIO_NUM)
  enable_led(true);  // 开启LED
  vTaskDelay(150 / portTICK_PERIOD_MS);  // 等待150ms让LED亮起
  fb = esp_camera_fb_get();              // 获取摄像头帧
  enable_led(false);  // 关闭LED
#else
  fb = esp_camera_fb_get();  // 直接获取摄像头帧
#endif

  if (!fb) {
    log_e("Camera capture failed");  // 摄像头捕获失败
    httpd_resp_send_500(req);        // 发送500错误
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");  // 设置响应类型为JPEG
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");  // 设置文件名
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问

  char ts[32];
  snprintf(ts, 32, "%lld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);  // 格式化时间戳
  httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);  // 设置时间戳头

#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  size_t fb_len = 0;
#endif
  if (fb->format == PIXFORMAT_JPEG) {
    // 如果已经是JPEG格式，直接发送
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = fb->len;
#endif
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    // 如果不是JPEG格式，需要先转换
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);  // 发送结束块
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    fb_len = jchunk.len;
#endif
  }
  esp_camera_fb_return(fb);  // 归还帧缓冲区
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
  int64_t fr_end = esp_timer_get_time();  // 记录结束时间
#endif
  log_i("JPG: %uB %ums", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));  // 日志输出
  return res;
}

// ==================== 视频流处理函数 ====================
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char *part_buf[128];

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();  // 初始化上一帧时间
  }

  // 设置响应类型为视频流
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  httpd_resp_set_hdr(req, "X-Framerate", "60");  // 设置帧率

#if defined(LED_GPIO_NUM)
  isStreaming = true;  // 标记正在流传输
  enable_led(true);    // 开启LED
#endif

  // 主循环，持续发送视频帧
  while (true) {
    fb = esp_camera_fb_get();  // 获取摄像头帧
    if (!fb) {
      log_e("Camera capture failed");  // 摄像头捕获失败
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        // 如果不是JPEG格式，转换为JPEG
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");  // JPEG压缩失败
          res = ESP_FAIL;
        }
      } else {
        // 已经是JPEG格式，直接使用
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }
    
    // 发送边界标记
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    // 发送帧头信息（内容类型、长度、时间戳）
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    // 发送JPEG数据
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    
    // 清理资源
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    
    if (res != ESP_OK) {
      log_e("Send frame failed");  // 发送帧失败
      break;
    }
    
    // 计算帧率统计
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
#if ARDUHAL_LOG_LEVEL >= ARDUHAL_LOG_LEVEL_INFO
    uint32_t avg_frame_time = ra_filter_run(&ra_filter, frame_time);
#endif
    log_i(
      "MJPG: %uB %ums (%.1ffps), AVG: %ums (%.1ffps)", (uint32_t)(_jpg_buf_len), (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time, avg_frame_time,
      1000.0 / avg_frame_time
    );
  }

#if defined(LED_GPIO_NUM)
  isStreaming = false;  // 标记流传输结束
  enable_led(false);    // 关闭LED
#endif

  return res;
}

// ==================== 解析GET请求参数 ====================
static esp_err_t parse_get(httpd_req_t *req, char **obuf) {
  char *buf = NULL;
  size_t buf_len = 0;

  buf_len = httpd_req_get_url_query_len(req) + 1;  // 获取查询字符串长度
  if (buf_len > 1) {
    buf = (char *)malloc(buf_len);  // 分配内存
    if (!buf) {
      httpd_resp_send_500(req);  // 内存分配失败
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      *obuf = buf;  // 返回查询字符串
      return ESP_OK;
    }
    free(buf);  // 释放内存
  }
  httpd_resp_send_404(req);  // 发送404错误
  return ESP_FAIL;
}

// ==================== 摄像头控制命令处理函数 ====================
static esp_err_t cmd_handler(httpd_req_t *req) {
  char *buf = NULL;
  char variable[32];
  char value[32];

  if (parse_get(req, &buf) != ESP_OK) {  // 解析GET参数
    return ESP_FAIL;
  }
  // 提取变量名和值
  if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK || 
      httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int val = atoi(value);  // 转换值为整数
  log_i("%s = %d", variable, val);  // 日志输出
  sensor_t *s = esp_camera_sensor_get();  // 获取传感器对象
  int res = 0;

  // 根据不同的命令设置摄像头参数
  if (!strcmp(variable, "framesize")) {
    if (s->pixformat == PIXFORMAT_JPEG) {
      res = s->set_framesize(s, (framesize_t)val);  // 设置帧大小
    }
  } else if (!strcmp(variable, "quality")) {
    res = s->set_quality(s, val);  // 设置JPEG质量
  } else if (!strcmp(variable, "contrast")) {
    res = s->set_contrast(s, val);  // 设置对比度
  } else if (!strcmp(variable, "brightness")) {
    res = s->set_brightness(s, val);  // 设置亮度
  } else if (!strcmp(variable, "saturation")) {
    res = s->set_saturation(s, val);  // 设置饱和度
  } else if (!strcmp(variable, "gainceiling")) {
    res = s->set_gainceiling(s, (gainceiling_t)val);  // 设置增益上限
  } else if (!strcmp(variable, "colorbar")) {
    res = s->set_colorbar(s, val);  // 设置彩条测试模式
  } else if (!strcmp(variable, "awb")) {
    res = s->set_whitebal(s, val);  // 设置自动白平衡
  } else if (!strcmp(variable, "agc")) {
    res = s->set_gain_ctrl(s, val);  // 设置自动增益控制
  } else if (!strcmp(variable, "aec")) {
    res = s->set_exposure_ctrl(s, val);  // 设置自动曝光控制
  } else if (!strcmp(variable, "hmirror")) {
    res = s->set_hmirror(s, val);  // 设置水平镜像
  } else if (!strcmp(variable, "vflip")) {
    res = s->set_vflip(s, val);  // 设置垂直翻转
  } else if (!strcmp(variable, "awb_gain")) {
    res = s->set_awb_gain(s, val);  // 设置自动白平衡增益
  } else if (!strcmp(variable, "agc_gain")) {
    res = s->set_agc_gain(s, val);  // 设置自动增益控制增益
  } else if (!strcmp(variable, "aec_value")) {
    res = s->set_aec_value(s, val);  // 设置自动曝光值
  } else if (!strcmp(variable, "aec2")) {
    res = s->set_aec2(s, val);  // 设置自动曝光2
  } else if (!strcmp(variable, "dcw")) {
    res = s->set_dcw(s, val);  // 设置DCW
  } else if (!strcmp(variable, "bpc")) {
    res = s->set_bpc(s, val);  // 设置坏点校正
  } else if (!strcmp(variable, "wpc")) {
    res = s->set_wpc(s, val);  // 设置白点校正
  } else if (!strcmp(variable, "raw_gma")) {
    res = s->set_raw_gma(s, val);  // 设置原始Gamma
  } else if (!strcmp(variable, "lenc")) {
    res = s->set_lenc(s, val);  // 设置镜头校正
  } else if (!strcmp(variable, "special_effect")) {
    res = s->set_special_effect(s, val);  // 设置特殊效果
  } else if (!strcmp(variable, "wb_mode")) {
    res = s->set_wb_mode(s, val);  // 设置白平衡模式
  } else if (!strcmp(variable, "ae_level")) {
    res = s->set_ae_level(s, val);  // 设置自动曝光级别
  }
#if defined(LED_GPIO_NUM)
  else if (!strcmp(variable, "led_intensity")) {
    led_duty = val;  // 设置LED亮度
    if (isStreaming) {
      enable_led(true);  // 如果正在流传输，立即应用
    }
  }
#endif
  else {
    log_i("Unknown command: %s", variable);  // 未知命令
    res = -1;
  }

  if (res < 0) {
    return httpd_resp_send_500(req);  // 命令执行失败
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  return httpd_resp_send(req, NULL, 0);  // 发送响应
}

// ==================== 打印寄存器值 ====================
static int print_reg(char *p, sensor_t *s, uint16_t reg, uint32_t mask) {
  return sprintf(p, "\"0x%x\":%u,", reg, s->get_reg(s, reg, mask));  // 格式化寄存器值
}

// ==================== 状态信息处理函数 ====================
static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];  // JSON响应缓冲区

  sensor_t *s = esp_camera_sensor_get();  // 获取传感器对象
  char *p = json_response;
  *p++ = '{';

  // 根据不同传感器型号输出寄存器信息
  if (s->id.PID == OV5640_PID || s->id.PID == OV3660_PID) {
    for (int reg = 0x3400; reg < 0x3406; reg += 2) {
      p += print_reg(p, s, reg, 0xFFF);  // 12位寄存器
    }
    p += print_reg(p, s, 0x3406, 0xFF);

    p += print_reg(p, s, 0x3500, 0xFFFF0);  // 16位寄存器
    p += print_reg(p, s, 0x3503, 0xFF);
    p += print_reg(p, s, 0x350a, 0x3FF);   // 10位寄存器
    p += print_reg(p, s, 0x350c, 0xFFFF);  // 16位寄存器

    for (int reg = 0x5480; reg <= 0x5490; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5380; reg <= 0x538b; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }

    for (int reg = 0x5580; reg < 0x558a; reg++) {
      p += print_reg(p, s, reg, 0xFF);
    }
    p += print_reg(p, s, 0x558a, 0x1FF);  // 9位寄存器
  } else if (s->id.PID == OV2640_PID) {
    p += print_reg(p, s, 0xd3, 0xFF);
    p += print_reg(p, s, 0x111, 0xFF);
    p += print_reg(p, s, 0x132, 0xFF);
  }

  // 输出当前摄像头状态
  p += sprintf(p, "\"xclk\":%u,", s->xclk_freq_hz / 1000000);  // XCLK频率
  p += sprintf(p, "\"pixformat\":%u,", s->pixformat);  // 像素格式
  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);  // 帧大小
  p += sprintf(p, "\"quality\":%u,", s->status.quality);  // JPEG质量
  p += sprintf(p, "\"brightness\":%d,", s->status.brightness);  // 亮度
  p += sprintf(p, "\"contrast\":%d,", s->status.contrast);  // 对比度
  p += sprintf(p, "\"saturation\":%d,", s->status.saturation);  // 饱和度
  p += sprintf(p, "\"sharpness\":%d,", s->status.sharpness);  // 锐度
  p += sprintf(p, "\"special_effect\":%u,", s->status.special_effect);  // 特殊效果
  p += sprintf(p, "\"wb_mode\":%u,", s->status.wb_mode);  // 白平衡模式
  p += sprintf(p, "\"awb\":%u,", s->status.awb);  // 自动白平衡
  p += sprintf(p, "\"awb_gain\":%u,", s->status.awb_gain);  // AWB增益
  p += sprintf(p, "\"aec\":%u,", s->status.aec);  // 自动曝光
  p += sprintf(p, "\"aec2\":%u,", s->status.aec2);  // 自动曝光2
  p += sprintf(p, "\"ae_level\":%d,", s->status.ae_level);  // AE级别
  p += sprintf(p, "\"aec_value\":%u,", s->status.aec_value);  // AE值
  p += sprintf(p, "\"agc\":%u,", s->status.agc);  // 自动增益
  p += sprintf(p, "\"agc_gain\":%u,", s->status.agc_gain);  // AGC增益
  p += sprintf(p, "\"gainceiling\":%u,", s->status.gainceiling);  // 增益上限
  p += sprintf(p, "\"bpc\":%u,", s->status.bpc);  // 坏点校正
  p += sprintf(p, "\"wpc\":%u,", s->status.wpc);  // 白点校正
  p += sprintf(p, "\"raw_gma\":%u,", s->status.raw_gma);  // 原始Gamma
  p += sprintf(p, "\"lenc\":%u,", s->status.lenc);  // 镜头校正
  p += sprintf(p, "\"hmirror\":%u,", s->status.hmirror);  // 水平镜像
  p += sprintf(p, "\"vflip\":%u,", s->status.vflip);  // 垂直翻转
  p += sprintf(p, "\"dcw\":%u,", s->status.dcw);  // DCW
  p += sprintf(p, "\"colorbar\":%u", s->status.colorbar);  // 彩条
#if defined(LED_GPIO_NUM)
  p += sprintf(p, ",\"led_intensity\":%u", led_duty);  // LED亮度
#else
  p += sprintf(p, ",\"led_intensity\":%d", -1);
#endif
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");  // 设置响应类型为JSON
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  return httpd_resp_send(req, json_response, strlen(json_response));  // 发送JSON数据
}

// ==================== XCLK时钟设置处理函数 ====================
static esp_err_t xclk_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _xclk[32];

  if (parse_get(req, &buf) != ESP_OK) {  // 解析GET参数
    return ESP_FAIL;
  }
  if (httpd_query_key_value(buf, "xclk", _xclk, sizeof(_xclk)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int xclk = atoi(_xclk);  // 转换频率值
  log_i("Set XCLK: %d MHz", xclk);  // 日志输出

  sensor_t *s = esp_camera_sensor_get();  // 获取传感器对象
  int res = s->set_xclk(s, LEDC_TIMER_0, xclk);  // 设置XCLK频率
  if (res) {
    return httpd_resp_send_500(req);  // 设置失败
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  return httpd_resp_send(req, NULL, 0);  // 发送响应
}

// ==================== 寄存器写操作处理函数 ====================
static esp_err_t reg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];
  char _val[32];

  if (parse_get(req, &buf) != ESP_OK) {  // 解析GET参数
    return ESP_FAIL;
  }
  // 提取寄存器、掩码和值
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || 
      httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK ||
      httpd_query_key_value(buf, "val", _val, sizeof(_val)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);  // 寄存器地址
  int mask = atoi(_mask);  // 掩码
  int val = atoi(_val);  // 值
  log_i("Set Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, val);

  sensor_t *s = esp_camera_sensor_get();  // 获取传感器对象
  int res = s->set_reg(s, reg, mask, val);  // 设置寄存器
  if (res) {
    return httpd_resp_send_500(req);  // 设置失败
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  return httpd_resp_send(req, NULL, 0);  // 发送响应
}

// ==================== 寄存器读操作处理函数 ====================
static esp_err_t greg_handler(httpd_req_t *req) {
  char *buf = NULL;
  char _reg[32];
  char _mask[32];

  if (parse_get(req, &buf) != ESP_OK) {  // 解析GET参数
    return ESP_FAIL;
  }
  // 提取寄存器和掩码
  if (httpd_query_key_value(buf, "reg", _reg, sizeof(_reg)) != ESP_OK || 
      httpd_query_key_value(buf, "mask", _mask, sizeof(_mask)) != ESP_OK) {
    free(buf);
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  free(buf);

  int reg = atoi(_reg);  // 寄存器地址
  int mask = atoi(_mask);  // 掩码
  sensor_t *s = esp_camera_sensor_get();  // 获取传感器对象
  int res = s->get_reg(s, reg, mask);  // 读取寄存器
  if (res < 0) {
    return httpd_resp_send_500(req);  // 读取失败
  }
  log_i("Get Register: reg: 0x%02x, mask: 0x%02x, value: 0x%02x", reg, mask, res);

  char buffer[20];
  const char *val = itoa(res, buffer, 10);  // 转换为字符串
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  return httpd_resp_send(req, val, strlen(val));  // 发送寄存器值
}

// ==================== 解析GET参数变量 ====================
static int parse_get_var(char *buf, const char *key, int def) {
  char _int[16];
  if (httpd_query_key_value(buf, key, _int, sizeof(_int)) != ESP_OK) {
    return def;  // 返回默认值
  }
  return atoi(_int);  // 返回解析的值
}

// ==================== PLL设置处理函数 ====================
static esp_err_t pll_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {  // 解析GET参数
    return ESP_FAIL;
  }

  // 提取PLL参数
  int bypass = parse_get_var(buf, "bypass", 0);
  int mul = parse_get_var(buf, "mul", 0);
  int sys = parse_get_var(buf, "sys", 0);
  int root = parse_get_var(buf, "root", 0);
  int pre = parse_get_var(buf, "pre", 0);
  int seld5 = parse_get_var(buf, "seld5", 0);
  int pclken = parse_get_var(buf, "pclken", 0);
  int pclk = parse_get_var(buf, "pclk", 0);
  free(buf);

  log_i("Set Pll: bypass: %d, mul: %d, sys: %d, root: %d, pre: %d, seld5: %d, pclken: %d, pclk: %d", 
        bypass, mul, sys, root, pre, seld5, pclken, pclk);
  sensor_t *s = esp_camera_sensor_get();  // 获取传感器对象
  int res = s->set_pll(s, bypass, mul, sys, root, pre, seld5, pclken, pclk);  // 设置PLL
  if (res) {
    return httpd_resp_send_500(req);  // 设置失败
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  return httpd_resp_send(req, NULL, 0);  // 发送响应
}

// ==================== 窗口分辨率设置处理函数 ====================
static esp_err_t win_handler(httpd_req_t *req) {
  char *buf = NULL;

  if (parse_get(req, &buf) != ESP_OK) {  // 解析GET参数
    return ESP_FAIL;
  }

  // 提取窗口参数
  int startX = parse_get_var(buf, "sx", 0);
  int startY = parse_get_var(buf, "sy", 0);
  int endX = parse_get_var(buf, "ex", 0);
  int endY = parse_get_var(buf, "ey", 0);
  int offsetX = parse_get_var(buf, "offx", 0);
  int offsetY = parse_get_var(buf, "offy", 0);
  int totalX = parse_get_var(buf, "tx", 0);
  int totalY = parse_get_var(buf, "ty", 0);
  int outputX = parse_get_var(buf, "ox", 0);
  int outputY = parse_get_var(buf, "oy", 0);
  bool scale = parse_get_var(buf, "scale", 0) == 1;
  bool binning = parse_get_var(buf, "binning", 0) == 1;
  free(buf);

  log_i(
    "Set Window: Start: %d %d, End: %d %d, Offset: %d %d, Total: %d %d, Output: %d %d, Scale: %u, Binning: %u", 
    startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning
  );
  sensor_t *s = esp_camera_sensor_get();  // 获取传感器对象
  int res = s->set_res_raw(s, startX, startY, endX, endY, offsetX, offsetY, totalX, totalY, outputX, outputY, scale, binning);  // 设置原始分辨率
  if (res) {
    return httpd_resp_send_500(req);  // 设置失败
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  // 允许跨域访问
  return httpd_resp_send(req, NULL, 0);  // 发送响应
}

// ==================== 主页处理函数 ====================
static esp_err_t index_handler(httpd_req_t *req) {
    // 创建一个HTML页面，默认显示视频流，带有可展开的设置菜单
    const char* stream_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
    <title>ESP32 Camera Stream</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }
        
        body {
            background: #000;
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
        }
        
        .container {
            width: 100%;
            height: 100vh;
            display: flex;
            flex-direction: column;
            position: relative;
        }
        
        .stream-wrapper {
            flex: 1;
            background: #000;
            position: relative;
            overflow: hidden;
        }
        
        #stream {
            width: 100%;
            height: 100%;
            object-fit: contain;
        }
        
        .controls {
            position: fixed;
            bottom: 20px;
            right: 20px;
            z-index: 1000;
        }
        
        .settings-btn {
            background: rgba(0,0,0,0.7);
            backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.2);
            color: #fff;
            width: 50px;
            height: 50px;
            border-radius: 50%;
            font-size: 24px;
            cursor: pointer;
            transition: all 0.3s ease;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        
        .settings-btn:hover {
            background: rgba(0,0,0,0.9);
            transform: scale(1.05);
        }
        
        .settings-panel {
            position: fixed;
            bottom: 80px;
            right: 20px;
            background: rgba(0,0,0,0.9);
            backdrop-filter: blur(10px);
            border-radius: 10px;
            padding: 15px;
            width: 280px;
            display: none;
            border: 1px solid rgba(255,255,255,0.2);
            z-index: 1000;
        }
        
        .settings-panel.show {
            display: block;
        }
        
        .settings-panel h3 {
            color: #fff;
            margin-bottom: 10px;
            font-size: 14px;
        }
        
        .settings-panel select,
        .settings-panel input {
            width: 100%;
            padding: 8px;
            margin-bottom: 10px;
            background: rgba(255,255,255,0.1);
            border: 1px solid rgba(255,255,255,0.2);
            color: #fff;
            border-radius: 5px;
        }
        
        .status-bar {
            position: fixed;
            bottom: 20px;
            left: 20px;
            background: rgba(0,0,0,0.7);
            backdrop-filter: blur(10px);
            padding: 8px 15px;
            border-radius: 8px;
            display: flex;
            gap: 15px;
            color: #fff;
            font-size: 11px;
            font-family: monospace;
            z-index: 1000;
        }
        
        .led {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background-color: #0f0;
            display: inline-block;
            animation: pulse 2s infinite;
        }
        
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
        
        @media (max-width: 768px) {
            .settings-panel {
                width: 250px;
                bottom: 70px;
            }
            .status-bar {
                font-size: 9px;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="stream-wrapper">
            <img id="stream" src="/stream" alt="Camera Stream">
        </div>
    </div>
    
    <div class="controls">
        <div class="settings-btn" onclick="toggleSettings()">⚙️</div>
        <div class="settings-panel" id="settingsPanel">
            <h3>📷 Camera Settings</h3>
            <select id="resolution" onchange="setResolution(this.value)">
                <option value="10">UXGA (1600x1200)</option>
                <option value="9">SXGA (1280x1024)</option>
                <option value="8">XGA (1024x768)</option>
                <option value="7">SVGA (800x600)</option>
                <option value="6">VGA (640x480)</option>
                <option value="5">CIF (400x296)</option>
                <option value="4">QVGA (320x240)</option>
                <option value="3">HQVGA (240x176)</option>
            </select>
            
            <h3>✨ Quality</h3>
            <input type="range" id="quality" min="10" max="63" value="12" onchange="setQuality(this.value)">
            
            <h3>🔆 Brightness</h3>
            <input type="range" id="brightness" min="-2" max="2" step="1" value="0" onchange="setBrightness(this.value)">
            
            <h3>🎨 Contrast</h3>
            <input type="range" id="contrast" min="-2" max="2" step="1" value="0" onchange="setContrast(this.value)">
            
            <h3>🌈 Saturation</h3>
            <input type="range" id="saturation" min="-2" max="2" step="1" value="0" onchange="setSaturation(this.value)">
        </div>
    </div>
    
    <div class="status-bar">
        <span><span class="led"></span> ESP32-CAM</span>
        <span>Live Stream</span>
        <span id="fps">FPS: --</span>
    </div>
    
    <script>
        let frameCount = 0;
        let lastTime = performance.now();
        
        function toggleSettings() {
            const panel = document.getElementById('settingsPanel');
            panel.classList.toggle('show');
        }
        
        function updateFPS() {
            frameCount++;
            const now = performance.now();
            const delta = now - lastTime;
            if (delta >= 1000) {
                const fps = Math.round((frameCount * 1000) / delta);
                document.getElementById('fps').textContent = `FPS: ${fps}`;
                frameCount = 0;
                lastTime = now;
            }
            requestAnimationFrame(updateFPS);
        }
        
        function sendCommand(varName, value) {
            fetch(`/control?var=${varName}&val=${value}`)
                .then(response => console.log(`${varName} set to ${value}`))
                .catch(err => console.error('Error:', err));
        }
        
        function setResolution(value) {
            sendCommand('framesize', value);
        }
        
        function setQuality(value) {
            document.getElementById('quality').value = value;
            sendCommand('quality', value);
        }
        
        function setBrightness(value) {
            sendCommand('brightness', value);
        }
        
        function setContrast(value) {
            sendCommand('contrast', value);
        }
        
        function setSaturation(value) {
            sendCommand('saturation', value);
        }
        
        const img = document.getElementById('stream');
        img.onload = () => frameCount++;
        img.onerror = () => setTimeout(() => img.src = '/stream?' + new Date().getTime(), 2000);
        
        updateFPS();
    </script>
</body>
</html>
)rawliteral";
    
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, stream_html, strlen(stream_html));
    return ESP_OK;
}
// ==================== 启动摄像头服务器 ====================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();  // 获取默认HTTP配置
  config.max_uri_handlers = 16;  // 设置最大URI处理程序数量

  // 定义主页URI
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义状态URI
  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义控制URI
  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义拍照URI
  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义视频流URI
  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义BMP格式URI
  httpd_uri_t bmp_uri = {
    .uri = "/bmp",
    .method = HTTP_GET,
    .handler = bmp_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义XCLK设置URI
  httpd_uri_t xclk_uri = {
    .uri = "/xclk",
    .method = HTTP_GET,
    .handler = xclk_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义寄存器写操作URI
  httpd_uri_t reg_uri = {
    .uri = "/reg",
    .method = HTTP_GET,
    .handler = reg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义寄存器读操作URI
  httpd_uri_t greg_uri = {
    .uri = "/greg",
    .method = HTTP_GET,
    .handler = greg_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义PLL设置URI
  httpd_uri_t pll_uri = {
    .uri = "/pll",
    .method = HTTP_GET,
    .handler = pll_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  // 定义分辨率设置URI
  httpd_uri_t win_uri = {
    .uri = "/resolution",
    .method = HTTP_GET,
    .handler = win_handler,
    .user_ctx = NULL
#ifdef CONFIG_HTTPD_WS_SUPPORT
    ,
    .is_websocket = true,
    .handle_ws_control_frames = false,
    .supported_subprotocol = NULL
#endif
  };

  ra_filter_init(&ra_filter, 20);  // 初始化运行平均滤波器，采样大小为20

  log_i("Starting web server on port: '%d'", config.server_port);  // 日志：启动Web服务器
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {  // 启动HTTP服务器
    // 注册所有URI处理程序
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &bmp_uri);

    httpd_register_uri_handler(camera_httpd, &xclk_uri);
    httpd_register_uri_handler(camera_httpd, &reg_uri);
    httpd_register_uri_handler(camera_httpd, &greg_uri);
    httpd_register_uri_handler(camera_httpd, &pll_uri);
    httpd_register_uri_handler(camera_httpd, &win_uri);
  }

  // 启动视频流服务器（使用下一个端口）
  config.server_port += 1;
  config.ctrl_port += 1;
  log_i("Starting stream server on port: '%d'", config.server_port);  // 日志：启动流服务器
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {  // 启动流服务器
    httpd_register_uri_handler(stream_httpd, &stream_uri);  // 注册视频流URI
  }
}

// ==================== 设置LED闪光灯 ====================
void setupLedFlash() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, 5000, 8);  // 连接LED引脚，频率5kHz，8位分辨率
#else
  log_i("LED flash is disabled -> LED_GPIO_NUM undefined");  // LED闪光灯未启用
#endif
}
