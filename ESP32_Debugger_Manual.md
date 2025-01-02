# ESP32无线调试器操作手册

## 一、产品介绍

这是一款基于ESP32S3的无线调试器，由发送机和接收机两部分组成。接收机默认烧录双机模式固件，可与发送机配对使用。支持以下主要功能：

- SWD和JTAG协议调试
- 虚拟串口通信
- 有线/无线模式切换
- 可选供电方式(5V/3.3V)
- LED状态指示
- 高速下载(>10KB/s)
- Arduino串口烧录支持

## 二、使用准备

### 1. 有线调试模式
1. 直接通过USB将接收机连接到电脑
2. 无需使用发送机
3. 系统将识别出U盘和虚拟串口设备

### 2. 无线调试模式(双机)
1. 将发送机插入电脑USB端口
2. 等待系统识别出虚拟串口设备
3. 发送机会自动创建WiFi热点:
   - 热点名称: MGodmonkey
   - 密码: 1234567890
4. 接收机会自动连接到发送机的WiFi热点

### 3. 无线调试模式(单机)
> 单机模式需要先烧录单机固件,并确保电脑和接收机连接到同一个WiFi网络

#### WiFi配置说明
接收机内置了3个WiFi配置:
1. 配置一:
   - SSID: ESP32_DAP
   - Password: 12345678
2. 配置二:
   - SSID: CMSIS-DAP
   - Password: 12345678
3. 配置三:
   - SSID: DAP
   - Password: 12345678

请将电脑连接到以上任一WiFi,或修改源码中的WiFi配置信息后重新烧录。

#### 使用elaphureLink (仅支持SWD)
1. 解压elaphureLink压缩包，运行elaphureLink.Wpf.exe
2. 首次使用需要安装驱动:
   - 打开elaphureLink
   - 点击"安装"
   - 按提示完成安装
3. 点击开始透传按钮，等待连接成功
4. 连接成功后在Keil中配置:
   - 打开项目设置(Options for Target)
   - 选择Debug选项卡
   - Use栏选择"CMSIS-DAP Debugger"
   - 点击Settings确认设备已识别
   - 如使用JTAG模式，将Max Clock先设为5MHz或低于5MHz，显示识别到设备后，则可设置回10MHz
5. 开始调试下载

#### 使用USBIP (支持SWD/JTAG)
1. 下载USBIP工具包并解压
2. 通过串口查看接收机IP地址:
   - 连接接收机的CH340串口
   - 波特率115200
   - 查看输出的IP地址信息
3. 在CMD中运行:
   ```
   cd [USBIP工具目录]
   usbip.exe attach -r [接收机IP] -b 1-1
   ```
4. 等待系统识别虚拟USB设备
5. 在Keil中按有线模式相同方式配置
6. 开始调试下载

## WIFI配置说明

### 默认WIFI配置
接收机的WIFI配置位于 `main/include/wifi_configuration.h` 文件中，默认预置了3个WIFI信息：

1. 发送机AP热点 (默认优先级最高)
   - SSID: `MGodmonkey`
   - 密码: `1234567890`

2. 备用热点
   - SSID: `DAP`
   - 密码: `12345678`

3. 实验室网络
   - SSID: `INSS-Lab_2.4G`
   - 密码: `G-STAR1211`

### 使用说明
- 接收机会按顺序尝试连接配置文件中的WIFI
- 单机模式使用时，需要确保能连接到其中一个WIFI
- 可以通过以下方式添加自己的WIFI：
  - 修改`main/include/wifi_configuration.h`文件中的WIFI信息
  - 重新烧录接收机固件  

## 二、软件准备

### 所需软件包
1. CH554.zip - CH554程序源码
2. ESP12F.zip - ESP12F程序源码
3. firmware.zip - 所有直接烧录的固件(包含单机模式和双机模式)
4. WCHISPTool Setup.zip - CH554烧录工具
5. flash download tool 3.9.7.zip - ESP芯片烧录工具
6. elaphureLink Windows x64 release.zip - 单机模式无线调试上位机
7. usbip-win-0.3.6-dev.zip - 单机模式无线调试工具

### 3D打印文件
1. 发送机底部.STL
2. 发送机顶盖.STL
3. 接收机底部.STL
4. 接收机顶盖.STL

## 三、固件烧录

### 发送机烧录
1. CH554烧录:
   - 使用WCHISPTool工具
   - 新的CH554默认上电进入Boot模式
   - 已烧录过的CH554需短接U2旁触点后上电
   - 选择firmware/ch554.hex进行烧录

2. ESP12F烧录:
   - 使用flash_download_tool工具
   - 选择ESP8266芯片
   - 长按BOOT按钮，短按RST按钮，松开BOOT进入下载模式
   - 选择firmware/ESP12F.ion.bin进行烧录
   - 烧录完成后重新上电，将自动创建WiFi热点

### 接收机烧录
1. 使用flash_download_tool工具:
   - 选择ESP32S3芯片
   - LoadMode选择UART
   - 选择CH340K串口设备
   
2. 分区表设置:
   | 文件 | 地址 |
   |------|------|
   | bootloader.bin | 0x0 |
   | ESP32_DAPLink.bin | 0x20000 |
   | partition-table.bin | 0x8000 |
   | ota_data_initial.bin | 0xf000 |

## 四、LED指示灯状态说明

| LED状态 | 含义 |
|---------|------|
| 橙色呼吸 | 设备启动中 |
| 橙色闪烁2次 | 正在切换模式 |
| 紫色常亮 | 有线模式 |
| 红色常亮 | 等待连接WIFI或启动错误 |
| 绿色常亮 | WIFI连接成功且信号良好 |
| 绿色呼吸 | WIFI连接成功且信号一般 |
| 红色闪烁 | WIFI连接成功但信号较差 |

## 五、接线说明

### SWD模式接线
- 3V3 → 3V3
- GND → GND  
- SWDIO → DIO
- SWCLK → CLK

### JTAG模式接线
- 3V3 → 3V3
- GND → GND
- TMS → DIO
- TCK → CLK
- TDI → TDI
- TDO → TDO
- nRESET → RST (可选)

### UART接线
- TXD → TXD
- RXD → RXD
> 串口接线对应的是TTL烧录器上的符号，如果在Arduino设备烧录时出现问题，可以将RXD和TXD互换后重试

## 六、调试操作步骤

> 默认烧录双机模式固件，如需单机使用需重新烧录单机固件

### 1. 有线调试模式
1. 通过USB连接接收机到电脑
2. 按照接线说明连接目标板
3. 在Keil MDK中配置:
   - 打开项目设置(Options for Target)
   - 选择Debug选项卡
   - Use栏选择"CMSIS-DAP Debugger"
   - 点击Settings确认设备已识别
   - 如使用JTAG模式，将Max Clock先设为5MHz或低于5MHz，显示识别到设备后，则可设置回10MHz
4. 点击Download按钮下载程序
5. 点击Start/Stop Debug Session进入调试

### 2. 无线调试模式(双机)
1. 插入发送机，等待系统识别
2. 确认接收机已连接到发送机WiFi(查看LED状态)
3. 连接目标板到接收机
4. 在Keil MDK中配置:
   - 打开项目设置(Options for Target)
   - 选择Debug选项卡
   - Use栏选择"CMSIS-DAP Debugger"
   - 点击Settings确认设备已识别
   - 如使用JTAG模式，将Max Clock设为5MHz
5. 开始调试下载

### 3. Arduino设备烧录
1. 连接Arduino设备到接收机的UART接口
2. 打开Arduino IDE
3. 选择对应生成的虚拟串口
4. 点击上传按钮开始烧录
> 如遇烧录失败，尝试将RXD和TXD互换

### 4. 串口通信
1. 有线模式:
   - 连接虚拟串口设备
   - 波特率可设置超过1M
   - 支持串口收发数据
   - 支持与调试同时使用
   
2. 无线模式:
   - 双机模式下直接使用发送机的虚拟串口
   - 单机模式可使用内置WebSerial功能
   - 支持与调试同时使用

#### WebSerial使用说明
1. 设备连接成功后，在浏览器中访问 http://[接收机IP]
2. 网页界面提供以下功能：
   - 实时串口数据显示
   - 可调节波特率（最高115200）
3. 特点：
   - 无需安装额外软件
   - 支持多客户端同时访问
   - 跨平台兼容（支持手机访问）

### 3. 无线调试模式(单机)
#### 使用elaphureLink
1. 打开elaphureLink软件
2. 点击开始透传按钮，等待连接成功
3. 连接成功后在Keil中配置:
   - 按有线模式相同方式设置
   - 确保elaphureLink保持运行
4. 开始调试下载

#### 使用USBIP
1. 获取接收机IP地址
2. 在CMD中运行:
   ```
   cd [USBIP工具目录]
   usbip.exe attach -r [接收机IP] -b 1-1
   ```
3. 等待系统识别虚拟USB设备
4. 在Keil中按有线模式相同方式配置
5. 开始调试下载

## 七、注意事项

1. Keil MDK版本要求5.29及以上
2. JTAG模式可能需要调整时钟频率以解决通信问题
3. 使用无线模式时，确保接收机与发送机距离适中
4. 建议在开阔环境下使用无线功能
5. 默认烧录双机模式固件，如需单机使用需重新烧录单机固件
6. 单机模式下使用USBIP时，确保电脑和接收机在同一局域网内
7. 使用elaphureLink时需保持软件运行，关闭后将断开连接

## 八、已知的BUG

1. JTAG模式下，JTAG时钟频率为10MHz时，无法正常通信，需要先设置为5MHz或低于5MHz的频率后才能正常通信，正常通信后可设置回10MHz
2. SWD模式下，时钟频率为10MHz以下时，无法正常通信
> 以上bug暂时确定为软件问题，后续会修复

## 九、故障排除

1. 无法识别设备
   - 检查USB连接
   - 重新插拔设备
   - 确认驱动安装正确

2. 无线连接不稳定
   - 检查信号强度(观察LED状态)
   - 调整设备间距离
   - 减少环境中的干扰源
   - 必要时切换到有线模式

3. 下载调试失败
   - 检查接线是否正确
   - 确认目标板供电正常
   - JTAG模式下尝试降低时钟频率
   - 确认Keil配置正确 
