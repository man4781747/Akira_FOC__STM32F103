# STM32F103 無刷馬達 FOC 驅動

自製 PCB 上的三相無刷馬達 FOC 控制韌體，是 [STM32F446RE_TEST](../STM32F446RE_TEST)（後代）的前一代產品。以 STM32CubeMX 產生專案，CMake + GCC 建置。指令、高速擷取與 SWD 介面和後代相同，後代 repo 的電腦端工具可以直接使用。

| 項目 | 內容 |
| --- | --- |
| MCU | STM32F103C8T6，72 MHz，沒有 FPU（64 KB Flash / 20 KB RAM） |
| 馬達驅動 | DRV8317H（3x PWM），PWM 20 kHz |
| 位置感測 | AS5600 磁編碼器，I2C 400 kHz + DMA，12-bit（× 16 換成 65536 count/rev） |
| 馬達 | 12 槽 14 極（7 極對），與後代同一顆 |
| 控制 | 電流環 20 kHz（定點數）；速度環、位置環 5 kHz。速度模式、位置模式（串級或 MIT） |
| 操作介面 | USART1 115200 8N1 文字指令、ST-Link (SWD) 指令信箱；CAN 1 Mbit/s（舊的指令格式） |

## 文件

[doc/設計文件.html](doc/設計文件.html)：硬體、腳位、周邊、FOC 架構、參數、時序、已知問題、與後代的差異（第 17 節是指令的差異）。指令本身的說明見後代的 `doc/serial指令.html`。

## 建置與燒錄

需要 STM32CubeCLT（`arm-none-eabi-gcc`、`cmake`、`ninja`、`STM32_Programmer_CLI` 在 PATH 內）。

```sh
cmake --preset debug          # 第一次，產生 build/debug
cmake --build build/debug
STM32_Programmer_CLI --connect port=swd --download build/debug/STM32_FOC.elf -hardRst -rst --start
```

Debug 建置是 `-Os -g3`：`-O0` 時程式放不進 64 KB Flash。修改周邊設定時改 `STM32_FOC.ioc` 後以 CubeMX 重新產生程式碼，使用者程式都在 USER CODE 區塊內。

## 電腦端工具

在後代 repo 執行，連線時讀 MCU 型號自動判斷是哪一代的板子（`motor_ui/board.py`），elf 預設用這裡的 `build/Debug/STM32_FOC.elf`。

```sh
cd ../STM32F446RE_TEST
uv run -m motor_ui                    # SWD：調參、即時圖、擷取
uv run swd_cmd.py ALIGN               # SWD：重新對齊電氣零點
uv run log_capture.py 40 --port COMxx # UART：高速擷取存成 CSV
```

這一代還沒有的功能（EEPROM 參數保存、MT6835 自校正、棘輪 / 限位 / 無摩擦模式、軌跡規劃、ITEST）在 motor_ui 內會停用。

## 上電後的行為

開機約 1.1 秒後進入閉迴路，以速度模式、0.1 圈/秒 運轉（從軸端看逆時針），並每 5 ms 從 UART 輸出一行數據。開機過程會把轉子拉到電氣零點做對齊（約 0.5 秒）。常用指令：

```
SET SPEED 5          轉速改成 5 圈/秒
GET VEL              讀速度環參數
SET MODE 1           進入位置模式，鎖在目前位置
SET ANGLE 90         轉到 90 度
SET POSTYPE 1        位置模式改用 MIT（彈簧 + 阻尼）
STOP / RUN           關閉 / 恢復輸出
```
