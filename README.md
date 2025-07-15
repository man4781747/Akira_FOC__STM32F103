# STM32 FOC（磁場導向控制）專案

本專案為基於 STM32F103 系列微控制器的磁場導向控制（FOC, Field Oriented Control）實作範例，適用於馬達控制、嵌入式開發學習與研究。

---

## 功能特色

- 支援 SVPWM（空間向量脈寬調變）演算法
- I2C 感測器資料讀取
- CAN 通訊介面
- UART 訊息輸出
- 多通道 PWM 控制
- 可擴充 PID 控制、低通濾波等功能

---

## 硬體需求

- STM32F103 系列開發板（或相容 STM32F1）
- 馬達驅動板
- I2C 感測器（如磁編碼器）
- 相關連接線材

---

## 開發環境

- Visual Studio Code
  - STM32Cube for Visual Studio Code
  - twxs.cmake
- STM32CubeMX
- STM32CubeCLT
  - vscode 需設定此程式資料夾路徑

---

## 快速開始

1. 以 STM32CubeMX 開啟 `STM32_FOC.ioc`，根據硬體配置產生程式碼
2. 使用 VSCode 開啟本專案資料夾
3. 編譯並燒錄至開發板
4. 透過 UART 監控訊息輸出，或以示波器觀察 PWM 波形

---

## 目錄結構

- `Core/Inc/`：主要標頭檔
- `Core/Src/`：主要程式碼
- `Drivers/`：HAL 及 CMSIS 驅動
- `STM32_FOC.ioc`：CubeMX 專案檔
- `README.md`：專案說明文件

---

## 常見問題

- **PWM 觸發不同步？**  
  請參考 main.c 內同步啟動 PWM/OC 的範例程式碼。
- **CubeCLT 設定？**  
  請於 VSCode 設定 STM32CubeCLT 路徑。

---

## 參考教學

- [YouTube 教學](https://www.youtube.com/watch?v=fhuFuQU7gyo)
- [STM32 中文社區討論串](https://shequ.stmicroelectronics.cn/thread-643922-1-1.html)

---