## 项目简介

1. 接收机基于立创ESP32S3R8N8开发板制作，可独立运行，也可搭配发送机运行，接收机可以搭配发送机实现即插即用，无需上位机和WIFI，即可远程调试和开发
2. 接收机支持支持SWD和JTAG协议，支持 OpenOCD/pyOCD，支持虚拟串口，可同时进行仿真调试和串口收发
3. 接收机支持有线调试和无线调试的切换，通过MODE按钮长按进行模式切换
4. 接收机内置Web服务，可通过访问接收机IP来实现简单的WebSerial调试
5. 接收机支持向目标板供电（5V、3.3V），以及从目标板取电（5V、3.3V）两种方式进行工作
6. 接收机带状态指示灯，可提示当前调试模式，WIFI信号强度等
7. 烧录速度大于10KB/s，串口通信波特率可设置1M及以上

## 实物展示
| 实物正面图                                                   | 实物背面图                                                   |
| ------------------------------------------------------------ | ------------------------------------------------------------ |
| ![image.png](//image.lceda.cn/oshwhub/23eec65737b04035a22ce59e413d4e35.png) | ![image.png](//image.lceda.cn/oshwhub/1c5177f235e64bbb85e17d9ffd822018.png) |


![image.png](//image.lceda.cn/oshwhub/d6cbd0a665a8490aa1ea96a615d865ed.png)

![image.png](//image.lceda.cn/oshwhub/71ebc79606384779a0456ca78a9ce50e.png)

## 原理解析（硬件说明）
![image.png](//image.lceda.cn/oshwhub/a41efce056da42c7bac39c2de0ca15bc.png)

​	本项目的发送端硬件由**CH554G单片机**和**ESP12F无线模块**组成。CH554G 是一款兼容 MCS51 指令集并支持 USB 2.0 的高性能单片机，主要负责与PC端进行USB 通信。通过USB连接PC，生成 **CMSIS DAP调试设备**和**CDC 虚拟串口设备**，提供调试和串口通信功能。同时，CH554G通过UART与ESP12F无线模块连接。ESP12F 负责**创建无线热点（AP）**并**搭建 TCP 服务器**，等待接收端设备连接，从而实现数据的**无线透传**功能，**CHECK信号线**用于判断是否与**接收机建立连接**。**SY6280AAC**用于**防止意外过压**或**过流**导致的电路损坏，保障系统的安全性和稳定性，限流大小为**I(A)=6800/R2=1A**。

![image.png](//image.lceda.cn/oshwhub/f252d0f7d704405f9a3006186ccc4d65.png)

​	接收端硬件主要由立创**ESP32S3R8N8 开发板**构成，通过移植 **DAPLink** 代码实现 **CMSIS-DAP** 功能，用于支持 **SWD** 和 **JTAG** 协议的调试功能。在有线调试模式下，基于**TinyUSB框架**实现USB通信功能，其中 **CMSIS-DAP** 设备和 **CDC 虚拟串口**设备通过TinyUSB的**HID**和**CDC**模块完成初始化和数据传输，支持在PC端的 Keil 环境中进行调试和串口通信。
​	同时，接收机还实现了无线调试功能，利用 ESP32S3内置的**Wi-Fi模块**，连接到发送端创建的AP热点，并与**TCP服务器**建立连接，实现**无线数据传输**，从而支持无线下载调试和串口透传。
​	此外，接收端还支持**独立运行**，通过连接到与PC端相同的Wi-Fi网络，在局域网内与PC端建立连接，结合**USBIP技术栈**（通过网络远程共享和访问USB设备的一种技术)，可以实现**远程调试**和**串口透传**功能，也能结合**elaphureLink上位机**实现快速的连接和keil的远程下载调试。

## 软件烧录
### 发送机烧录
​	1.发送端需要烧录ESP12F和CH554G的固件，CH554G需要使用`WCHISPTool_Setup`烧录工具进行烧录，如果是全新的CH554G，默认插上电脑的USB，上电就会进入Boot模式，此时可以通过工具进行识别和烧录，如果是已经烧录过的CH554G则需要短接U2旁边的触点，然后插上电脑上电，才能通过工具进行烧录，烧录固件为`firmware/ch554.hex`

![image.png](//image.lceda.cn/oshwhub/aef66dd19c12471eb1a3330e53d20c63.png)

![ch554烧录.png](//image.lceda.cn/oshwhub/a7a4096cce4b4425a3008bc5134f4f4a.png)

​	ESP12F烧录通过引出来的串口接口进行烧录，需要用一个USB转TTL下载工具，使用乐鑫官方下载工具flash_download_tool进行烧录，长按BOOT按钮，然后短按RST按钮，松开BOOT按钮即可进入ESP12F的下载模式

![image.png](//image.lceda.cn/oshwhub/1139f03db3f4419da49d39d64a24b587.png)

​	`flash_download_tool`选择ESP8266, 然后选择固件`firmware/ESP12F.ion.bin`进行烧录，如果点击开始后没反应，先判断一下串口是否连接对了，再判断一下ESP12F是否进入的下载模式

![image.png](//image.lceda.cn/oshwhub/e34f7b8e3eb7403499066eebf29bde68.png)

![image.png](//image.lceda.cn/oshwhub/70b20ec877b849dd8a5686663987812c.png)

​	烧录完成后，重新接入电脑，发送机就可以正常使用了，此时ESP12F会生成一个AP热点，热点名称：`MGodmonkey`,密码：`1234567890`（后续会考虑给ESP12F添加OTA更新的功能，让其烧录一次代码就能无限远程更新）

### 接收机烧录
​	接收机的固件有两种，一个是搭配USBIP技术栈独立使用的单机模式固件，一个是搭配发送机使用的双机模式固件，两种固件下载方式都是一样的。
`flash_download_tool`选择ESP32S3, LoadMode选择UART,ESP32S3立创开发板连接电脑后会生成两个串口，一个是USB-OTG的USB串行设备，一个是连接CH340K的串口设备，选择第二个串口进行烧录即可

![image.png](//image.lceda.cn/oshwhub/5ce15e3a21ab4a85b44a2d98c1c8d307.png)

![image.png](//image.lceda.cn/oshwhub/733644ed4fae4bada1844f29783f8967.png)

​	`flash_download_tool`的分区设置要严格按照图中的进行设置，否则可能会出错，烧录的固件在`firmware/Single_Mode`或`firmware/Double_Mode`中

|         文件         |  索引   |
| :------------------: | :-----: |
|    bootloader.bin    |   0x0   |
|  ESP32_DAPLink.bin   | 0x20000 |
| partition-table.bin  | 0x8000  |
| ota_data_initial.bin | 0xf000  |

![image.png](//image.lceda.cn/oshwhub/00c666c9831b4e6ebec64f9bca81e07e.png)

## 外壳说明
发送机的外壳可以在淘宝上买到公版的电流计亚克力外壳来使用：
https://item.taobao.com/item.htm?_u=l2otn78942e7&id=536932505082&spm=a1z09.2.0.0.6b752e8dQAsOIG

## 接线方式
根据接收机上的引脚进行接线

SW协议接线：

| 符号  | 对应引脚 |
| :---: | :------: |
|  3V3  |   3V3    |
|  GND  |   GND    |
| SWDIO |   DIO    |
| SWCLK |   CLK    |

JTAG协议接线：

|  符号  |   对应引脚    |
| :----: | :-----------: |
|  3V3   |      3V3      |
|  GND   |      GND      |
|  TMS   |      DIO      |
|  TCK   |      CLK      |
|  TDI   |      TDI      |
|  TDO   |      TDO      |
| nRESET | RST（可不接） |

UART接线：

| 符号 | 对应引脚 |
| :--: | :------: |
| TXD  |   TXD    |
| RXD  |   RXD    |

## 指示灯说明

|   指示灯    |          状态           |
| :---------: | :---------------------: |
|  橙色呼吸   |         启动中          |
| 橙色闪烁2次 |       模式切换中        |
|  紫色常亮   |        有线模式         |
|  红色常亮   | 等待连接WIFI / 启动错误 |
|  绿色常亮   | 连接WIFI成功并信号很好  |
|  绿色呼吸   | 连接WIFI成功并信号一般  |
|  红色闪烁   | 连接WIFI成功并信号较差  |

## 使用步骤

> **Keil MDK-ARM**版本需要**5.29**及以上版本，其他软件暂未测试

### 单接收机使用

1. 有线模式（SWD/JTAG）

   有线模式下直接连接usb到电脑上即可，另一边通过SWD/JATG连接待调试的设备，启动成功后会在电脑上生成一个U盘，这是DAPLink的特性（但离线文件下载功能还没实现，，，所以暂时还不能通过传固件文件到u盘上来实现自动烧录），同时会生成一个虚拟串口
   
   ![image-20241226102833534](https://s2.loli.net/2024/12/26/Uo6TWyah1gb9IjK.png)

​	此时打开keil工程，点击魔法棒->debug->Use选择CMSIS-DAP Debugger->点击Settings后即可看到设备信息，值得说明的是，JTAG协议下有个小bug，就是有时会显示**SWD/JTAG Communication Failure**，此时**Max Clock**设置为**5MHz或低于10MHz下的任意频率**即可，然后就能正常显示设备了，此时也能正常切换回10MHz下的频率进行使用了（应该是软件代码问题，暂未找到bug原因）

![image-20241226105423465](https://s2.loli.net/2024/12/26/ys2CLgI5wqN6ATO.png)

![image-20241226105525512](https://s2.loli.net/2024/12/26/TwOAsYEFjR5Vdea.png)

2. 有线模式（UART）

   串口支持接收发，支持在SWD/JTAG调试下一起使用，串口频率可以任意设置，可设置到超过1M波特率

   ![xx](https://s2.loli.net/2024/12/26/DakEGTxO7YnszwA.gif)

3.无线模式(SWD/JTAG)

 	单机无线模式下keil需要搭配开源上位机`elaphureLink`（`elaphureLink`不支持JTAG协议）或者USBIP（支持SWD/JTAG协议）来使用。双击`elaphureLink.Wpf.exe`打开软件，第一次使用需要安装keil驱动，由于代码已经设置好了mDNS，因此无需知道设备IP也可直接连接

![image-20241226112001788](https://s2.loli.net/2024/12/26/nx961E3BStyPe5u.png)

​	打开keil工程，点击魔法棒->debug->Use选择CMSIS-DAP Debugger->点击Settings后即可看到设备信息

![image-20241226112242339](https://s2.loli.net/2024/12/26/4Z9JnO3wAm6TxQI.png)

​	USBIP使用需要知道提前ESP32S3的IP地址，并且与ESP32S3同一网络下，有线连接开发板，串口助手连接CH340K开头的设备串口即可看到ESP32S3的串口调试信息，成功连接WIFI后会显示IP地址，一般情况下这个IP不会改变，如果家里设备情况多的话，路由器可能会给其重新分配一个IP，此时就需要重新获取了

![image-20241226112746854](https://s2.loli.net/2024/12/26/mqrjMYTUdL1lK2g.png)

​	**cmd终端**中输入`.\usbip.exe attach -r 设备IP -b 1-1`即可连接设备并且在设备管理器中即可看到生成的**无线CMSIS-DAP**，打开keil工程，点击魔法棒->debug->Use选择CMSIS-DAP Debugger->点击Settings后即可看到设备信息

![xxx](https://s2.loli.net/2024/12/26/IsiVUwolGePtOSq.gif)

![image-20241226113735096](https://s2.loli.net/2024/12/26/CWq7j81aJxs9zuL.png)

4.无线模式(UART)

无线串口传输跟有线串口传输功能是一样的，不过接收机内置了一个Web服务，可以通过访问ESP32S3的IP地址来使用**WebSerial**功能，在网页上发送数据到接收机的串口中

![image.png](//image.lceda.cn/oshwhub/2a9e50d867be4b28908b0e5f6890810e.png)

### 搭配发送机使用

> 发送机能实现单接收机的所有功能，只不过接收机自带WIFI，无需上位机和局域网，通过ESP12F与ESP32S3的WIFI模块进行通信，实现远程下载调试和串口传输

## 注意事项

* 如果仅需要单机使用，直接打样接收机的板子即可
* 单机模式需要搭配`elaphureLink`或`usbip`来使用
* 接收机3D外壳底座的按键部分打印出来需要轻轻的向外掰，注意是轻轻的哦

## 总结

> 该项目是匆匆忙忙赶工期造出来的，因为本人处于研究生开题阶段，也在忙着找创新点，经历过的应该都知道这个阶段的苦哈，项目基本功能已经实现，但还有很多bug和不足，后续会慢慢补充和完善
> 本项目借鉴了以下优秀的开源项目：
>
> - [https://github.com/windowsair/wireless-esp8266-dap](https://github.com/windowsair/wireless-esp8266-dap)
> - [ttps://oshwhub.com/q837877663/duo-gong-neng-wu-xian-xia-zai-qi](https://oshwhub.com/q837877663/duo-gong-neng-wu-xian-xia-zai-qi)
