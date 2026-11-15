# UWB 直線跟隨小車

三份程式對應角色（最新架構：距離在車上算，不再傳 REPORT）：

| 資料夾 | 板子 | 角色 |
|--------|------|------|
| `uno_car/` | Arduino UNO R3 + BMP73T104 | 讀距離、控制馬達 |
| `uwb_car_responder/` | 車上 ESP32-UWB | DS-TWR Responder，算距離 → Serial2 |
| `uwb_tag_initiator/` | 人身上 ESP32-UWB | DS-TWR Initiator，發 POLL／FINAL |

## 行為

- 距離 ≥ 2.0 m → 前進
- 距離 < 1.8 m → 停止
- 1.8～2.0 m → 維持上一狀態
- UWB 超過約 1 秒無資料 → 緊急煞停

## 接線

| 車上 ESP32-UWB | UNO R3 |
|----------------|--------|
| GPIO17 (TX2) | D10 |
| GND | GND |
| 不要接 | D11 / 5V（各板各自供電） |

- USB 監看鮑率：UNO 與兩顆 ESP32 皆 **115200**
- ESP32 → UNO 資料鮑率：**9600**
- 天線延遲：`TX_ANT_DLY` / `RX_ANT_DLY` = **16400**
- Channel：目前兩邊皆為 **5**（必須相同）

## Arduino IDE

- UNO：開發板選 `Arduino Uno`
- ESP32-UWB：`ESP32 Dev Module`，需安裝 `dw3000` 函式庫
- UNO 需安裝 `BMP73T104` 函式庫
