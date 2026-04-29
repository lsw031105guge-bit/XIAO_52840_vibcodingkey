#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <bluefruit.h>
#include <PDM.h>

static constexpr uint32_t kBaud = 115200;

static constexpr uint8_t kButtonPin1 = D1;
// EN: If your button module outputs LOW when pressed, set this to true.
// 中文：如果你的按键模块“按下输出低电平(LOW)”，这里设为 true。
static constexpr bool kButtonActiveLow = false;
static constexpr uint32_t kDebounceMs = 25;
static constexpr uint32_t kRecMaxMs = 30000;
static constexpr uint8_t kRecEndBlinkTimes = 3;
static constexpr uint16_t kRecEndBlinkOnMs = 80;
static constexpr uint16_t kRecEndBlinkOffMs = 80;

// EN: Key #1 NeoPixel
// 中文：按键 #1 的 NeoPixel
static constexpr uint8_t kNeoPixelPin1 = D2;
static constexpr uint16_t kNeoPixelCount1 = 1;

// EN: Key #2 (new Grove keycap): S1->D8 (button), S2->D9 (NeoPixel)
// 中文：按键 #2（新加 Grove 键帽）：S1->D8（按键输出），S2->D9（灯控信号）
static constexpr uint8_t kButtonPin2 = D8;
static constexpr uint8_t kNeoPixelPin2 = D9;
static constexpr uint16_t kNeoPixelCount2 = 1;

// EN: Key #3 (record button): S1->D5 (button), S2->D4 (NeoPixel)
// 中文：按键 #3（录音键）：S1->D5（按键输出），S2->D4（灯控信号）
static constexpr uint8_t kButtonPin3 = D5;
static constexpr uint8_t kNeoPixelPin3 = D4;
static constexpr uint16_t kNeoPixelCount3 = 1;

// EN: On macOS, "Keyboard Setup Assistant" asks for a specific physical key.
//     A one-key macropad usually can't satisfy that prompt. We send 'z' once
//     to help macOS finish the recognition.
// 中文：macOS 的“键盘设置助理”会要求按下特定物理按键。
//     单键设备通常无法完成该步骤。这里先发送一次 'z' 帮助识别完成。
static constexpr char kMacSetupKey = 'z';

// EN: macOS shortcuts (Command + key)
// 中文：macOS 快捷键（Command + 按键）
// EN: HID keycodes: A=0x04, B=0x05, C=0x06 ... V=0x19
// 中文：HID 键码：A=0x04, B=0x05, C=0x06 ... V=0x19
static constexpr uint8_t kHidKeycodeC = 0x06; // EN: 'c' | 中文：字母 c
static constexpr uint8_t kHidKeycodeV = 0x19; // EN: 'v' | 中文：字母 v

// EN: BLE HID keyboard (macro pad)
// 中文：BLE HID 键盘（宏键盘）
BLEDis bledis;
BLEHidAdafruit blehid;

// EN: BLE UART (NUS) for audio transfer
// 中文：BLE UART（NUS）用于音频数据传输
BLEUart bleuart;

static constexpr uint16_t kAudioSampleRateHz = 16000;
static constexpr uint8_t kAudioChannels = 1;
static constexpr uint8_t kAudioBytesPerSample = 2;

static constexpr uint16_t kBleAudioPayloadBytes = 200;
static constexpr uint16_t kBleAudioAckTimeoutMs = 300;
static constexpr uint8_t kBleAudioRetryMax = 3;

Adafruit_NeoPixel pixels1(kNeoPixelCount1, kNeoPixelPin1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels2(kNeoPixelCount2, kNeoPixelPin2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels3(kNeoPixelCount3, kNeoPixelPin3, NEO_GRB + NEO_KHZ800);

// EN: Recording state
// 中文：录音状态
enum class RecState : uint8_t { Idle = 0, Recording = 1 };
static volatile RecState gRecState = RecState::Idle;
static uint32_t gRecStartMs = 0;
static bool gRecWaitRelease = false;

// EN: PDM -> PCM buffer
// 中文：PDM -> PCM 缓冲
static int16_t gPdmSampleBuf[256];
static volatile int gPdmSamplesRead = 0;

// EN: Ring buffer to store recent audio while recording
// 中文：录音期间写入的环形缓冲（用于先验证采样链路）
static constexpr int kAudioRingSamples = 16000; // EN: ~1s @16kHz | 中文：16kHz 约 1 秒
static int16_t gAudioRing[kAudioRingSamples];
static int gAudioWriteIdx = 0;
static int gAudioCount = 0;

static void setPixels(uint32_t c1, uint32_t c2, uint32_t c3) {
  pixels1.setPixelColor(0, c1);
  pixels1.show();
  pixels2.setPixelColor(0, c2);
  pixels2.show();
  pixels3.setPixelColor(0, c3);
  pixels3.show();
}

static void setAllPixels(uint32_t c) {
  setPixels(c, c, c);
}

static void setConnPixels(bool connected) {
  const uint32_t c = connected ? pixels1.Color(30, 255, 60) : pixels1.Color(255, 40, 40);
  const uint32_t c3 = (gRecState == RecState::Recording) ? pixels1.Color(190, 60, 255) : c;
  setPixels(c, c, c3);
}

static void blinkPixel3(uint32_t color) {
  for (uint8_t i = 0; i < kRecEndBlinkTimes; i++) {
    pixels3.setPixelColor(0, color);
    pixels3.show();
    delay(kRecEndBlinkOnMs);
    pixels3.setPixelColor(0, 0);
    pixels3.show();
    delay(kRecEndBlinkOffMs);
  }
}

static int computePeakFromRing() {
  int peak = 0;
  for (int i = 0; i < gAudioCount; i++) {
    const int idx = (gAudioWriteIdx - gAudioCount + i + kAudioRingSamples) % kAudioRingSamples;
    const int v = abs((int)gAudioRing[idx]);
    if (v > peak) peak = v;
  }
  return peak;
}

static void audioRingReset();
static void bleAudioSendRecordedPcm();

static void startRecording(uint32_t nowMs, uint32_t seq) {
  gRecState = RecState::Recording;
  gRecStartMs = nowMs;
  audioRingReset();
  pixels3.setPixelColor(0, pixels1.Color(190, 60, 255));
  pixels3.show();
  Serial.print("key3 rec start #");
  Serial.println(seq);
}

static void stopRecording(uint32_t nowMs, uint32_t seq, bool timedOut) {
  gRecState = RecState::Idle;
  const uint32_t durMs = nowMs - gRecStartMs;
  const int peak = computePeakFromRing();

  Serial.print("key3 rec stop #");
  Serial.print(seq);
  Serial.print(" durMs=");
  Serial.print(durMs);
  Serial.print(" samples=");
  Serial.print(gAudioCount);
  Serial.print(" peak=");
  Serial.print(peak);
  Serial.print(" reason=");
  Serial.println(timedOut ? "timeout" : "release");

  blinkPixel3(timedOut ? pixels1.Color(255, 60, 60) : pixels1.Color(255, 200, 40));

  // EN: After recording ends, push PCM to host via BLE UART (NUS)
  // 中文：录音结束后，通过 BLE UART（NUS）发送 PCM 到电脑端
  bleAudioSendRecordedPcm();

  setConnPixels(Bluefruit.Periph.connected());
}

static void audioRingReset() {
  gAudioWriteIdx = 0;
  gAudioCount = 0;
}

static void audioRingPush(const int16_t* samples, int n) {
  for (int i = 0; i < n; i++) {
    gAudioRing[gAudioWriteIdx] = samples[i];
    gAudioWriteIdx = (gAudioWriteIdx + 1) % kAudioRingSamples;
    if (gAudioCount < kAudioRingSamples) {
      gAudioCount++;
    }
  }
}

static void onPDMdata() {
  const int bytesAvailable = PDM.available();
  if (bytesAvailable <= 0) {
    return;
  }

  const int bytesToRead = min(bytesAvailable, (int)sizeof(gPdmSampleBuf));
  PDM.read(gPdmSampleBuf, bytesToRead);
  gPdmSamplesRead = bytesToRead / 2;
}

static void sendMacCommandShortcut(uint8_t keycode) {
  // EN: Send a macOS shortcut: Command + <key>
  // 中文：发送 macOS 快捷键：Command + <键>
  uint8_t keys[6] = {keycode, 0, 0, 0, 0, 0};

  // EN: Left GUI maps to Command on macOS
  // 中文：Left GUI 在 macOS 上对应 Command
  blehid.keyboardReport(KEYBOARD_MODIFIER_LEFTGUI, keys);
  delay(8);

  // EN: Release all keys, otherwise host may think keys are held
  // 中文：发送松开报告，否则主机可能认为按键一直按住
  blehid.keyRelease();
  delay(8);
}

static int16_t audioRingSampleAt(int i) {
  const int idx = (gAudioWriteIdx - gAudioCount + i + kAudioRingSamples) % kAudioRingSamples;
  return gAudioRing[idx];
}

static bool waitUartAck(uint16_t seq, uint32_t timeoutMs) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeoutMs) {
    while (bleuart.available() >= 3) {
      const int tag = bleuart.read();
      if (tag == 'A') {
        const uint8_t lo = (uint8_t)bleuart.read();
        const uint8_t hi = (uint8_t)bleuart.read();
        const uint16_t got = (uint16_t)(lo | (uint16_t)(hi << 8));
        if (got == seq) {
          return true;
        }
      }
    }
    delay(1);
  }
  return false;
}

static bool bleAudioSendHeader(uint32_t sessionId) {
  // EN: Header: 'AUD1' + session(u32) + rate(u16) + samples(u32) + bps(u8) + ch(u8)
  // 中文：头：'AUD1' + 会话(u32) + 采样率(u16) + 样本数(u32) + bytesPerSample(u8) + 声道(u8)
  uint8_t hdr[4 + 4 + 2 + 4 + 1 + 1] = {0};
  hdr[0] = 'A';
  hdr[1] = 'U';
  hdr[2] = 'D';
  hdr[3] = '1';

  hdr[4] = (uint8_t)(sessionId & 0xFF);
  hdr[5] = (uint8_t)((sessionId >> 8) & 0xFF);
  hdr[6] = (uint8_t)((sessionId >> 16) & 0xFF);
  hdr[7] = (uint8_t)((sessionId >> 24) & 0xFF);

  hdr[8] = (uint8_t)(kAudioSampleRateHz & 0xFF);
  hdr[9] = (uint8_t)((kAudioSampleRateHz >> 8) & 0xFF);

  const uint32_t samples = (uint32_t)gAudioCount;
  hdr[10] = (uint8_t)(samples & 0xFF);
  hdr[11] = (uint8_t)((samples >> 8) & 0xFF);
  hdr[12] = (uint8_t)((samples >> 16) & 0xFF);
  hdr[13] = (uint8_t)((samples >> 24) & 0xFF);

  hdr[14] = kAudioBytesPerSample;
  hdr[15] = kAudioChannels;

  return bleuart.write(hdr, sizeof(hdr)) == sizeof(hdr);
}

static bool bleAudioSendDataPacket(uint16_t seq, uint32_t offsetBytes, uint16_t payloadLen) {
  // EN: Packet: 'D' + seq(u16) + len(u16) + payload
  // 中文：分包：'D' + 序号(u16) + 长度(u16) + 负载
  uint8_t buf[1 + 2 + 2 + kBleAudioPayloadBytes];
  buf[0] = 'D';
  buf[1] = (uint8_t)(seq & 0xFF);
  buf[2] = (uint8_t)((seq >> 8) & 0xFF);
  buf[3] = (uint8_t)(payloadLen & 0xFF);
  buf[4] = (uint8_t)((payloadLen >> 8) & 0xFF);

  const uint32_t sampleStart = offsetBytes / 2;
  const uint32_t sampleCount = payloadLen / 2;
  for (uint32_t i = 0; i < sampleCount; i++) {
    const int16_t s = audioRingSampleAt((int)(sampleStart + i));
    buf[5 + (i * 2) + 0] = (uint8_t)(s & 0xFF);
    buf[5 + (i * 2) + 1] = (uint8_t)((s >> 8) & 0xFF);
  }

  const size_t total = (size_t)(5 + payloadLen);
  return bleuart.write(buf, total) == total;
}

static bool bleAudioSendEot(uint32_t sessionId) {
  // EN: EOT: 'E' + session(u32)
  // 中文：结束：'E' + 会话(u32)
  uint8_t eot[1 + 4];
  eot[0] = 'E';
  eot[1] = (uint8_t)(sessionId & 0xFF);
  eot[2] = (uint8_t)((sessionId >> 8) & 0xFF);
  eot[3] = (uint8_t)((sessionId >> 16) & 0xFF);
  eot[4] = (uint8_t)((sessionId >> 24) & 0xFF);
  return bleuart.write(eot, sizeof(eot)) == sizeof(eot);
}

static void bleAudioSendRecordedPcm() {
  if (!Bluefruit.Periph.connected()) {
    Serial.println("audio tx skipped: not connected");
    return;
  }
  if (!bleuart.notifyEnabled()) {
    Serial.println("audio tx skipped: uart notify disabled");
    return;
  }
  if (gAudioCount <= 0) {
    Serial.println("audio tx skipped: empty");
    return;
  }

  const uint32_t sessionId = millis();
  Serial.print("audio tx begin session=");
  Serial.print(sessionId);
  Serial.print(" samples=");
  Serial.println(gAudioCount);

  pixels3.setPixelColor(0, pixels1.Color(60, 120, 255));
  pixels3.show();

  if (!bleAudioSendHeader(sessionId)) {
    Serial.println("audio tx failed: header");
    return;
  }

  const uint32_t totalBytes = (uint32_t)gAudioCount * 2;
  uint32_t offset = 0;
  uint16_t seq = 0;

  while (offset < totalBytes) {
    uint16_t payloadLen = (uint16_t)min((uint32_t)kBleAudioPayloadBytes, totalBytes - offset);
    payloadLen = (uint16_t)(payloadLen & ~1u);

    bool ok = false;
    for (uint8_t r = 0; r < kBleAudioRetryMax; r++) {
      if (!bleAudioSendDataPacket(seq, offset, payloadLen)) {
        delay(5);
        continue;
      }
      if (waitUartAck(seq, kBleAudioAckTimeoutMs)) {
        ok = true;
        break;
      }
    }

    if (!ok) {
      Serial.print("audio tx failed: no ack seq=");
      Serial.println(seq);
      blinkPixel3(pixels1.Color(255, 60, 60));
      setConnPixels(Bluefruit.Periph.connected());
      return;
    }

    offset += payloadLen;
    seq++;
  }

  bleAudioSendEot(sessionId);
  Serial.println("audio tx done");
  blinkPixel3(pixels1.Color(60, 255, 140));
  setConnPixels(Bluefruit.Periph.connected());
}

static void startAdv() {
  // EN: Advertising as a keyboard + UART
  // 中文：以“蓝牙键盘 + UART”身份广播
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_KEYBOARD);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

static void onConnect(uint16_t conn_handle) {
  (void)conn_handle;
  // EN: Green = connected
  // 中文：绿色 = 已连接
  setAllPixels(pixels1.Color(30, 255, 60));
  Serial.println("ble connected");
}

static void onDisconnect(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  // EN: Red = advertising / disconnected
  // 中文：红色 = 未连接（广播中）
  setAllPixels(pixels1.Color(255, 40, 40));
  Serial.println("ble disconnected (advertising)");
}

void setup() {
  Serial.begin(kBaud);
  uint32_t startMs = millis();
  while (!Serial && (millis() - startMs) < 1500) {
    delay(10);
  }

  // EN: Button inputs
  // 中文：按键输入
  pinMode(kButtonPin1, kButtonActiveLow ? INPUT_PULLUP : INPUT);
  pinMode(kButtonPin2, kButtonActiveLow ? INPUT_PULLUP : INPUT);
  pinMode(kButtonPin3, kButtonActiveLow ? INPUT_PULLUP : INPUT);

  // EN: NeoPixel status LEDs
  // 中文：NeoPixel 状态灯
  pixels1.begin();
  pixels1.setBrightness(80);
  pixels1.clear();
  pixels1.show();

  pixels2.begin();
  pixels2.setBrightness(80);
  pixels2.clear();
  pixels2.show();

  pixels3.begin();
  pixels3.setBrightness(80);
  pixels3.clear();
  pixels3.show();

  // EN: Init PDM mic (mono, 16kHz)
  // 中文：初始化 PDM 麦克风（单声道，16kHz）
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("PDM begin failed");
  } else {
    Serial.println("PDM ready");
  }

  // EN: Init BLE stack
  // 中文：初始化蓝牙协议栈
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("XIAO MacroPad");
  Bluefruit.Periph.setConnectCallback(onConnect);
  Bluefruit.Periph.setDisconnectCallback(onDisconnect);

  // EN: Device info (shown in some hosts)
  // 中文：设备信息（部分系统会显示）
  bledis.setManufacturer("Seeed Studio");
  bledis.setModel("XIAO nRF52840");
  bledis.begin();

  // EN: Start HID + UART services
  // 中文：启动 HID + UART 服务
  blehid.begin();
  bleuart.begin();

  onDisconnect(0, 0);
  startAdv();

  Serial.println("ready: pair via Bluetooth settings, then press the key");
}

void loop() {
  static bool stablePressed1 = false;
  static bool lastReadingPressed1 = false;
  static uint32_t lastChangeMs1 = 0;
  static uint32_t pressEdgeCount1 = 0;

  static bool stablePressed2 = false;
  static bool lastReadingPressed2 = false;
  static uint32_t lastChangeMs2 = 0;
  static uint32_t pressEdgeCount2 = 0;

  static bool stablePressed3 = false;
  static bool lastReadingPressed3 = false;
  static uint32_t lastChangeMs3 = 0;
  static uint32_t pressEdgeCount3 = 0;
  static uint32_t releaseEdgeCount3 = 0;

  static uint32_t lastStatusPrintMs = 0;
  static bool didSendMacSetupKey = false;

  const uint32_t nowMs = millis();

  // EN: Read raw pin levels
  // 中文：读取三个按键引脚的原始电平
  const int level1 = digitalRead(kButtonPin1);
  const int level2 = digitalRead(kButtonPin2);
  const int level3 = digitalRead(kButtonPin3);
  const bool readingPressed1 = kButtonActiveLow ? (level1 == LOW) : (level1 == HIGH);
  const bool readingPressed2 = kButtonActiveLow ? (level2 == LOW) : (level2 == HIGH);
  const bool readingPressed3 = kButtonActiveLow ? (level3 == LOW) : (level3 == HIGH);

  // EN: Consume PDM samples (copy out atomically, then push when recording)
  // 中文：取出 PDM 采样（先原子复制，再在录音状态写入缓冲）
  int16_t localPdm[256];
  int localPdmCount = 0;
  noInterrupts();
  localPdmCount = gPdmSamplesRead;
  if (localPdmCount > 0) {
    memcpy(localPdm, gPdmSampleBuf, (size_t)localPdmCount * sizeof(int16_t));
    gPdmSamplesRead = 0;
  }
  interrupts();

  if (localPdmCount > 0 && gRecState == RecState::Recording) {
    audioRingPush(localPdm, localPdmCount);
  }

  // EN: Print status periodically to understand what's happening
  // 中文：周期性打印状态，方便定位触发/连接/录音问题
  if (nowMs - lastStatusPrintMs >= 500) {
    lastStatusPrintMs = nowMs;
    Serial.print("k1 level=");
    Serial.print(level1);
    Serial.print(" pressed=");
    Serial.print(readingPressed1 ? 1 : 0);
    Serial.print(" stable=");
    Serial.print(stablePressed1 ? 1 : 0);
    Serial.print(" | k2 level=");
    Serial.print(level2);
    Serial.print(" pressed=");
    Serial.print(readingPressed2 ? 1 : 0);
    Serial.print(" stable=");
    Serial.print(stablePressed2 ? 1 : 0);
    Serial.print(" | k3 level=");
    Serial.print(level3);
    Serial.print(" pressed=");
    Serial.print(readingPressed3 ? 1 : 0);
    Serial.print(" stable=");
    Serial.print(stablePressed3 ? 1 : 0);
    Serial.print(" rec=");
    Serial.print(gRecState == RecState::Recording ? 1 : 0);
    Serial.print(" samples=");
    Serial.print(gAudioCount);
    Serial.print(" | connected=");
    Serial.println(Bluefruit.Periph.connected() ? 1 : 0);
  }

  if (gRecState == RecState::Recording && (nowMs - gRecStartMs) >= kRecMaxMs) {
    gRecWaitRelease = true;
    stopRecording(nowMs, releaseEdgeCount3 + 1, true);
  }

  // EN: Debounce key #1
  // 中文：按键 #1 防抖
  if (readingPressed1 != lastReadingPressed1) {
    lastReadingPressed1 = readingPressed1;
    lastChangeMs1 = nowMs;
  }

  bool pressedEdge1 = false;
  if ((nowMs - lastChangeMs1) >= kDebounceMs && stablePressed1 != readingPressed1) {
    stablePressed1 = readingPressed1;
    if (stablePressed1) {
      pressedEdge1 = true;
    }
  }

  // EN: Debounce key #2
  // 中文：按键 #2 防抖
  if (readingPressed2 != lastReadingPressed2) {
    lastReadingPressed2 = readingPressed2;
    lastChangeMs2 = nowMs;
  }

  bool pressedEdge2 = false;
  if ((nowMs - lastChangeMs2) >= kDebounceMs && stablePressed2 != readingPressed2) {
    stablePressed2 = readingPressed2;
    if (stablePressed2) {
      pressedEdge2 = true;
    }
  }

  // EN: Debounce key #3 (need press + release edges)
  // 中文：按键 #3 防抖（需要按下沿+抬起沿）
  if (readingPressed3 != lastReadingPressed3) {
    lastReadingPressed3 = readingPressed3;
    lastChangeMs3 = nowMs;
  }

  bool pressedEdge3 = false;
  bool releasedEdge3 = false;
  if ((nowMs - lastChangeMs3) >= kDebounceMs && stablePressed3 != readingPressed3) {
    stablePressed3 = readingPressed3;
    if (stablePressed3) {
      pressedEdge3 = true;
    } else {
      releasedEdge3 = true;
    }
  }

  if (!pressedEdge1 && !pressedEdge2 && !pressedEdge3 && !releasedEdge3) {
    delay(5);
    return;
  }

  // EN: Key #3 state machine: press=start record, release=stop record
  // 中文：按键 #3 状态机：按下开始录音，抬起结束录音
  if (pressedEdge3) {
    pressEdgeCount3++;
    if (!gRecWaitRelease && gRecState != RecState::Recording) {
      startRecording(nowMs, pressEdgeCount3);
    } else {
      Serial.println("key3 rec start ignored");
    }
  }

  if (releasedEdge3) {
    releaseEdgeCount3++;
    gRecWaitRelease = false;
    if (gRecState == RecState::Recording) {
      stopRecording(nowMs, releaseEdgeCount3, false);
    }
  }

  if (pressedEdge1) {
    pressEdgeCount1++;
    Serial.print("key1 pressedEdge #");
    Serial.print(pressEdgeCount1);
    Serial.print(" connected=");
    Serial.println(Bluefruit.Periph.connected() ? 1 : 0);
  }

  if (pressedEdge2) {
    pressEdgeCount2++;
    Serial.print("key2 pressedEdge #");
    Serial.print(pressEdgeCount2);
    Serial.print(" connected=");
    Serial.println(Bluefruit.Periph.connected() ? 1 : 0);
  }

  // EN: If not connected, still show feedback and print event
  // 中文：未连接也给反馈并打印按键事件，避免误以为“没触发”
  if (!Bluefruit.Periph.connected()) {
    // EN: Orange flash = pressed but not connected
    // 中文：橙色闪烁 = 按下了但还未连接
    setAllPixels(pixels1.Color(255, 120, 20));
    delay(40);
    setAllPixels(pixels1.Color(255, 40, 40));
    Serial.println("pressed (not connected)");
    delay(5);
    return;
  }

  // EN: Blue flash when sending
  // 中文：发送时闪蓝色
  setAllPixels(pixels1.Color(60, 120, 255));

  if (!didSendMacSetupKey) {
    // EN: Help macOS Keyboard Setup Assistant once
    // 中文：首次发送用于帮助 macOS 键盘识别
    char seq[2] = {kMacSetupKey, 0};
    blehid.keySequence(seq);
    didSendMacSetupKey = true;
    Serial.print("sent mac setup key: ");
    Serial.println(kMacSetupKey);
  } else {
    if (pressedEdge1) {
      // EN: Key #1 = Copy (Command + C)
      // 中文：按键 #1 = 复制（Command + C）
      sendMacCommandShortcut(kHidKeycodeC);
      Serial.println("key1 cmd+c sent");
    }

    if (pressedEdge2) {
      // EN: Key #2 = Paste (Command + V)
      // 中文：按键 #2 = 粘贴（Command + V）
      sendMacCommandShortcut(kHidKeycodeV);
      Serial.println("key2 cmd+v sent");
    }
  }

  delay(10);
  setAllPixels(pixels1.Color(30, 255, 60));
  delay(5);
}
