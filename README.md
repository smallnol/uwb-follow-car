# UWB + 視覺跟隨小車

樹莓派當大腦：**相機判斷方向、UWB 判斷距離、UNO 只執行馬達命令**。

對應的 Pi 程式在：[`PixelBit_Example/13_Vision_UWB_Follow`](https://github.com/smallnol/PixelBit_Example/tree/cursor/vision-uwb-follow-1509/13_Vision_UWB_Follow)

## 資料夾

| 資料夾 | 板子 | 角色 |
|--------|------|------|
| `uwb_tag_initiator/` | 人身上 ESP32-UWB | DS-TWR Initiator |
| `uwb_car_responder/` | 車上 ESP32-UWB | 算距離，輸出 `D:1.85`（USB / Serial2） |
| `uno_car/` | UNO R3 + BMP73T104 | **Pi-slave**：聽 `F/L/R/Z/S/X` 驅動馬達 |
| `uno_car_legacy/` | 同上 | 舊版「只靠 UWB 直線跟隨」（備份） |

## 新架構（建議）

```
身上 Tag  --UWB DS-TWR-->  車上 Responder
                              │ USB 115200（D:xx.xx）
                              ▼
                         樹莓派 5 + Camera
                         YOLO：人在左／中／右
                         UWB：遠近
                              │ USB-TTL 9600
                              ▼
                         UNO D10 ← 命令 F/L/R/Z/S/X
```

### 跟隨行為（由 Pi 決定）

- 找不到人 → `Z` 自轉搜尋
- 人在畫面左 → `L` 左轉置中
- 人在畫面右 → `R` 右轉置中
- 置中且 ≥ 2.0 m → `F` 前進
- 置中且 < 1.8 m → `S` 停止
- Pi 約 0.8 s 無命令 → UNO 緊急煞停

### UNO 接線（Pi-slave）

| 來源 | UNO |
|------|-----|
| Pi 側 USB-TTL **TX** | D10 |
| GND | GND |
| 不要接 | D11 / 互相 5V |

### 車上 ESP32 → Pi（距離）

- 最省事：車上 ESP32 **USB 插樹莓派**，鮑率 **115200**
- 輸出示例：
  ```
  D:1.85
  [CAR] 1.85 m
  ```
- Pi 解析 `D:` 開頭（也相容舊的 `[CAR]`）

### 天線 / 頻道

- `TX_ANT_DLY` / `RX_ANT_DLY` = **16400**
- Channel：**5**（Tag / Responder 必須相同）

## Arduino IDE

- UNO：`Arduino Uno`，需 `BMP73T104`
- ESP32-UWB：`ESP32 Dev Module`，需 `dw3000`

## 舊版直線跟隨

若暫時不用相機，可燒 `uno_car_legacy/`，並把車上 ESP32 `Serial2 TX (GPIO17)` 接回 UNO D10（舊接線）。新架構下請改燒 `uno_car/`。
