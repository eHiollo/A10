// bus_servo.cpp
// C++17 single-file port of your Python BusServo implementation.
// Windows: WinAPI serial, Linux/macOS: termios + select timeout.

#include <cstdint>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <map>
#include <optional>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <utility>


#ifdef _WIN32
  #define NOMINMAX
  #include <windows.h>
#else
  #include <fcntl.h>
  #include <unistd.h>
  #include <termios.h>
  #include <sys/select.h>
  #include <sys/ioctl.h>
  #ifdef __APPLE__
    #include <IOKit/serial/ioss.h>
  #endif
#endif

// --------------------- Utilities ---------------------
static inline void sleep_ms(int ms) {
#ifdef _WIN32
  Sleep(ms);
#else
  usleep(ms * 1000);
#endif
}

static inline uint16_t u16_le(uint16_t x) { return x; } // for clarity

static inline void push_u16_le(std::vector<uint8_t>& v, uint16_t x) {
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}

static inline int16_t read_i16_le(const uint8_t* p) {
  uint16_t u = static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
  return static_cast<int16_t>(u);
}

// --------------------- SerialPort (cross-platform) ---------------------
class SerialPort {
public:
  SerialPort() = default;
  ~SerialPort() { close(); }

  bool open(const std::string& port, int baudrate, int timeout_ms) {
    timeout_ms_ = timeout_ms;

#ifdef _WIN32
    std::string dev = port;
    // For COM10+ you must use "\\.\\COM10"
    if (dev.rfind("COM", 0) == 0 && dev.size() > 4) {
      dev = "\\\\.\\" + dev;
    }

    h_ = CreateFileA(dev.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (h_ == INVALID_HANDLE_VALUE) {
      std::cerr << "Failed to open serial: " << port << "\n";
      return false;
    }

    SetupComm(h_, 4096, 4096);

    DCB dcb{};
    dcb.DCBlength = sizeof(DCB);
    if (!GetCommState(h_, &dcb)) {
      std::cerr << "GetCommState failed\n";
      close();
      return false;
    }

    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;

    if (!SetCommState(h_, &dcb)) {
      std::cerr << "SetCommState failed\n";
      close();
      return false;
    }

    COMMTIMEOUTS to{};
    // Conservative timeouts: overall constant timeout
    to.ReadIntervalTimeout         = timeout_ms_;
    to.ReadTotalTimeoutMultiplier  = 0;
    to.ReadTotalTimeoutConstant    = timeout_ms_;
    to.WriteTotalTimeoutMultiplier = 0;
    to.WriteTotalTimeoutConstant   = timeout_ms_;
    SetCommTimeouts(h_, &to);

    PurgeComm(h_, PURGE_RXCLEAR | PURGE_TXCLEAR);
    is_open_ = true;
    return true;

#else
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      ::perror("open serial");
      return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      ::perror("tcgetattr");
      close();
      return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;

    // Set baudrate (best effort across Linux/macOS)
    if (!set_baudrate_posix_(baudrate, tio)) {
      std::cerr << "Warning: could not set baudrate exactly to " << baudrate << "\n";
    }

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      std::perror("tcsetattr");
      close();
      return false;
    }

    // back to blocking-style reads via select in readExact()
    is_open_ = true;
    tcflush(fd_, TCIFLUSH);
    return true;
#endif
  }

  void close() {
#ifdef _WIN32
    if (h_ != INVALID_HANDLE_VALUE) {
      CloseHandle(h_);
      h_ = INVALID_HANDLE_VALUE;
    }
#else
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
#endif
    is_open_ = false;
  }

  bool isOpen() const { return is_open_; }

  void resetInputBuffer() {
#ifdef _WIN32
    if (is_open_) PurgeComm(h_, PURGE_RXCLEAR);
#else
    if (is_open_) tcflush(fd_, TCIFLUSH);
#endif
  }

  bool writeAll(const std::vector<uint8_t>& data) {
    if (!is_open_) return false;
#ifdef _WIN32
    DWORD written = 0;
    if (!WriteFile(h_, data.data(), (DWORD)data.size(), &written, NULL)) return false;
    return written == data.size();
#else
    size_t total = 0;
    while (total < data.size()) {
      ssize_t n = ::write(fd_, data.data() + total, data.size() - total);
      if (n > 0) total += (size_t)n;
      else sleep_ms(1);
    }
    return true;
#endif
  }

  // Read exactly n bytes with timeout (ms). Returns nullopt on timeout/failure.
  std::optional<std::vector<uint8_t>> readExact(size_t n, int timeout_ms_override = -1) {
    if (!is_open_) return std::nullopt;
    int tmo = (timeout_ms_override >= 0) ? timeout_ms_override : timeout_ms_;

    std::vector<uint8_t> out;
    out.resize(n);

    auto start = std::chrono::steady_clock::now();
    size_t got = 0;

    while (got < n) {
      int remain_ms = tmo - (int)std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now() - start).count();
      if (remain_ms <= 0) return std::nullopt;

#ifdef _WIN32
      DWORD readn = 0;
      if (!ReadFile(h_, out.data() + got, (DWORD)(n - got), &readn, NULL)) return std::nullopt;
      if (readn == 0) {
        // timed out slice, loop until global timeout
        continue;
      }
      got += (size_t)readn;
#else
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(fd_, &rfds);
      timeval tv{};
      tv.tv_sec  = remain_ms / 1000;
      tv.tv_usec = (remain_ms % 1000) * 1000;

      int r = select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
      if (r <= 0) return std::nullopt;

      ssize_t rd = ::read(fd_, out.data() + got, n - got);
      if (rd > 0) got += (size_t)rd;
#endif
    }
    return out;
  }

private:
#ifndef _WIN32
  bool set_baudrate_posix_(int baudrate, termios& tio) {
    speed_t sp = 0;

    // Try common constants first
#ifdef B1000000
    if (baudrate == 1000000) sp = B1000000;
#endif
#ifdef B115200
    if (baudrate == 115200) sp = B115200;
#endif
#ifdef B921600
    if (baudrate == 921600) sp = B921600;
#endif

    if (sp != 0) {
      cfsetispeed(&tio, sp);
      cfsetospeed(&tio, sp);
      return true;
    }

#ifdef __APPLE__
    // macOS: set arbitrary baud via ioctl IOSSIOSPEED after tcsetattr typically,
    // but we can still return true here and apply later if needed.
    // We'll try setting a "close-ish" speed first:
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    // We'll apply exact with ioctl after open:
    speed_t speed = (speed_t)baudrate;
    if (ioctl(fd_, IOSSIOSPEED, &speed) == 0) return true;
    return false;
#else
    // Linux arbitrary baud via termios2 would be more code; keep best-effort fallback:
    // If you really need exact custom baud on Linux without B1000000, tell me your distro/kernel.
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    return false;
#endif
  }
#endif

private:
  int timeout_ms_{100};

#ifdef _WIN32
  HANDLE h_{INVALID_HANDLE_VALUE};
#else
  int fd_{-1};
#endif
  bool is_open_{false};
};

// --------------------- BusServo ---------------------
class BusServo {
public:
  // Instructions
  static constexpr uint8_t INST_PING       = 0x01;
  static constexpr uint8_t INST_READ_DATA  = 0x02;
  static constexpr uint8_t INST_WRITE_DATA = 0x03;
  static constexpr uint8_t INST_REG_WRITE  = 0x04;
  static constexpr uint8_t INST_ACTION     = 0x05;
  static constexpr uint8_t INST_RESET      = 0x06;
  static constexpr uint8_t INST_SYNC_READ  = 0x82;
  static constexpr uint8_t INST_SYNC_WRITE = 0x83;

  static constexpr uint8_t ID_BROADCAST    = 0xFE;

  struct SensorData {
    uint8_t id{};
    int16_t position{};
    int16_t speed{};
    int16_t load{};
    uint8_t voltage{};
    uint8_t temperature{};
  };

  BusServo(const std::string& port, int baudrate = 115200, int timeout_ms = 100, bool verbose = true)
    : verbose_(verbose) {
    if (!serial_.open(port, baudrate, timeout_ms)) {
      throw std::runtime_error("Failed to open serial port: " + port);
    }
    init_calibration_();
  }

  ~BusServo() { close(); }

  void close() { serial_.close(); }

  // Ping: return error byte (0 means OK). If timeout, returns 0xFF.
  uint8_t ping(uint8_t servo_id) {
    send_packet_(servo_id, INST_PING, {});
    auto resp = receive_packet_();
    if (!resp.ok) return 0xFF;
    return resp.error;
  }

  std::optional<std::vector<uint8_t>> read_data(uint8_t servo_id, uint8_t address, uint8_t length) {
    send_packet_(servo_id, INST_READ_DATA, {address, length});
    auto resp = receive_packet_();
    if (!resp.ok) return std::nullopt;
    if (resp.params.size() == length) return resp.params;
    return std::nullopt;
  }

  // Write data: returns (params, error) if non-broadcast; broadcast returns error=0 and empty params.
  std::pair<std::vector<uint8_t>, uint8_t> write_data(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values) {
    std::vector<uint8_t> params;
    params.reserve(1 + values.size());
    params.push_back(address);
    params.insert(params.end(), values.begin(), values.end());

    send_packet_(servo_id, INST_WRITE_DATA, params);
    if (servo_id != ID_BROADCAST) {
      auto resp = receive_packet_();
      if (!resp.ok) return {{}, 0xFF};
      return {resp.params, resp.error};
    }
    return {{}, 0};
  }

  std::pair<std::vector<uint8_t>, uint8_t> reg_write(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values) {
    std::vector<uint8_t> params;
    params.reserve(1 + values.size());
    params.push_back(address);
    params.insert(params.end(), values.begin(), values.end());

    send_packet_(servo_id, INST_REG_WRITE, params);
    if (servo_id != ID_BROADCAST) {
      auto resp = receive_packet_();
      if (!resp.ok) return {{}, 0xFF};
      return {resp.params, resp.error};
    }
    return {{}, 0};
  }

  void action(uint8_t servo_id = ID_BROADCAST) {
    send_packet_(servo_id, INST_ACTION, {});
    // No response expected
  }

  // SYNC_WRITE (broadcast)
  void sync_write(uint8_t address, uint8_t data_len, const std::vector<std::vector<uint8_t>>& servo_data) {
    std::vector<uint8_t> params;
    params.push_back(address);
    params.push_back(data_len);
    for (const auto& item : servo_data) {
      params.insert(params.end(), item.begin(), item.end());
    }
    send_packet_(ID_BROADCAST, INST_SYNC_WRITE, params);
  }

  // move servo: write to 0x2A, data_len=6 (pos2 + pwm2 + speed2)
  void move_servo(uint8_t servo_id, uint16_t position, uint16_t speed) {
    const uint16_t pwm = 0x1000; // same as python

    std::vector<uint8_t> item;
    item.reserve(1 + 6);
    item.push_back(servo_id);
    push_u16_le(item, position);
    push_u16_le(item, pwm);
    push_u16_le(item, speed);

    sync_write(0x2A, 6, {item});
  }

  // --------- App APIs ----------
  void set_servo_id(uint8_t old_id, uint8_t new_id) {
    write_data(old_id, 0x05, {new_id});
  }

  void set_baudrate(uint8_t servo_id, int baudrate) {
    // X = 2000000/baudrate - 1
    int x = (int)(2000000 / baudrate) - 1;
    x = std::clamp(x, 0, 255);
    write_data(servo_id, 0x06, { (uint8_t)x });
  }

  void set_middle_position(uint8_t servo_id) {
    // Python used struct.pack('<H', 128) and address 0x28
    std::vector<uint8_t> v;
    push_u16_le(v, 128);
    write_data(servo_id, 0x28, v);
  }

  void set_servo_torque_enable(uint8_t servo_id, bool enable = true) {
    uint8_t val = enable ? 1 : 0;
    write_data(servo_id, 0x28, {val}); // same as your python (may be protocol-specific)
  }

  std::optional<int16_t> get_position(uint8_t servo_id) {
    auto data = read_data(servo_id, 0x38, 2);
    if (!data) return std::nullopt;
    return read_i16_le(data->data());
  }

  std::optional<SensorData> read_sensor_data(uint8_t servo_id) {
    auto data = read_data(servo_id, 0x38, 8);
    if (!data || data->size() != 8) return std::nullopt;

    SensorData s;
    s.id = servo_id;
    s.position = read_i16_le(&(*data)[0]);
    s.speed    = read_i16_le(&(*data)[2]);
    s.load     = read_i16_le(&(*data)[4]);
    s.voltage  = (*data)[6];
    s.temperature = (*data)[7];
    return s;
  }

  // gripper_type: "50mm" or "100mm"
  void set_gripper_position(uint8_t servo_id, const std::string& gripper_type, double position_mm, uint16_t speed) {
    auto it = gripper_calib_.find(gripper_type);
    if (it == gripper_calib_.end()) {
      std::cerr << "Error: Unknown gripper type '" << gripper_type << "'\n";
      return;
    }
    int target = interpolate_(position_mm, it->second);
    target = std::clamp(target, 0, 4096);

    if (verbose_) {
      std::cout << "--> Gripper Control: Type=" << gripper_type
                << ", Target=" << position_mm << "mm"
                << ", ServoPos=" << target << "\n";
    }

    move_servo(servo_id, (uint16_t)target, speed);
  }

private:
  struct Resp {
    bool ok{false};
    uint8_t error{0};
    std::vector<uint8_t> params;
    std::string msg;
  };

  void log_hex_(const std::string& dir, const std::vector<uint8_t>& data) {
    if (!verbose_ || data.empty()) return;
    std::cout << "[" << dir << "] ";
    for (auto b : data) {
      std::cout << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << (int)b << " ";
    }
    std::cout << std::dec << "\n";
  }

  uint8_t calc_checksum_(uint8_t id, uint8_t length, uint8_t instruction, const std::vector<uint8_t>& params) {
    uint32_t total = id + length + instruction;
    for (auto p : params) total += p;
    return (uint8_t)((~total) & 0xFF);
  }

  std::vector<uint8_t> send_packet_(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params) {
    uint8_t length = (uint8_t)(params.size() + 2);
    uint8_t checksum = calc_checksum_(id, length, instruction, params);

    std::vector<uint8_t> pkt;
    pkt.reserve(2 + 1 + 1 + 1 + params.size() + 1);
    pkt.push_back(0xFF);
    pkt.push_back(0xFF);
    pkt.push_back(id);
    pkt.push_back(length);
    pkt.push_back(instruction);
    pkt.insert(pkt.end(), params.begin(), params.end());
    pkt.push_back(checksum);

    log_hex_("TX", pkt);

    serial_.resetInputBuffer();
    serial_.writeAll(pkt);
    return pkt;
  }

  Resp receive_packet_() {
    Resp r;
    std::vector<uint8_t> raw;

    // 1) header
    auto hdr = serial_.readExact(2);
    if (!hdr) { r.msg = "Timeout (No Header)"; return r; }
    raw.insert(raw.end(), hdr->begin(), hdr->end());

    if ((*hdr)[0] != 0xFF || (*hdr)[1] != 0xFF) {
      log_hex_("RX", raw);
      r.msg = "Header Error";
      return r;
    }

    // 2) id
    auto idb = serial_.readExact(1);
    if (!idb) { log_hex_("RX", raw); r.msg = "Timeout (No ID)"; return r; }
    raw.push_back((*idb)[0]);
    uint8_t resp_id = (*idb)[0];

    // 3) length
    auto lenb = serial_.readExact(1);
    if (!lenb) { log_hex_("RX", raw); r.msg = "Timeout (No Length)"; return r; }
    raw.push_back((*lenb)[0]);
    uint8_t length = (*lenb)[0];

    // 4) remaining length bytes: error(1)+params(n)+checksum(1)
    auto remain = serial_.readExact(length);
    if (!remain) { log_hex_("RX", raw); r.msg = "Timeout/Incompleted Packet"; return r; }
    raw.insert(raw.end(), remain->begin(), remain->end());

    log_hex_("RX", raw);

    if (remain->size() != length) {
      r.msg = "Packet Incomplete";
      return r;
    }

    uint8_t error = (*remain)[0];
    uint8_t recv_checksum = (*remain)[length - 1];

    std::vector<uint8_t> params;
    if (length >= 2) {
      params.insert(params.end(), remain->begin() + 1, remain->end() - 1);
    }

    // checksum verify (python used: calc_checksum(resp_id, length, error, params))
    uint8_t calc = calc_checksum_(resp_id, length, error, params);
    if (calc != recv_checksum) {
      r.msg = "Checksum Error";
      return r;
    }

    r.ok = true;
    r.error = error;
    r.params = std::move(params);
    return r;
  }

  int interpolate_(double value, const std::vector<std::pair<double, int>>& table) {
    if (table.empty()) return 0;
    if (value <= table.front().first) return table.front().second;
    if (value >= table.back().first)  return table.back().second;

    for (size_t i = 0; i + 1 < table.size(); ++i) {
      double x0 = table[i].first;
      double x1 = table[i + 1].first;
      int y0 = table[i].second;
      int y1 = table[i + 1].second;

      if (x0 <= value && value <= x1) {
        double ratio = (value - x0) / (x1 - x0);
        double y = y0 + ratio * (double)(y1 - y0);
        return (int)std::lround(y);
      }
    }
    return table.back().second;
  }

  void init_calibration_() {
    gripper_calib_["100mm"] = {
      {0, 3434},
      {10, 3060},
      {20, 2824},
      {30, 2646},
      {40, 2490},
      {50, 2354},
      {60, 2226},
      {70, 2095},
      {80, 1965},
      {90, 1810},
      {100, 1594}
    };

    gripper_calib_["50mm"] = {
      {0, 2048},
      {5, 1900},
      {10, 1775},
      {15, 1645},
      {20, 1530},
      {25, 1430},
      {30, 1320},
      {35, 1215},
      {40, 1085},
      {45, 960},
      {50, 798}
    };
  }

private:
  SerialPort serial_;
  bool verbose_{true};
  std::map<std::string, std::vector<std::pair<double, int>>> gripper_calib_;
};

// --------------------- Test main (same as your Python) ---------------------
int main() {
  try {
    // Windows: "COM5" (COM10+ will auto add \\.  prefix in this code)
    // Linux:   "/dev/ttyUSB0"
    // macOS:   "/dev/tty.usbserial-xxx"
    BusServo servo("/dev/ttyUSB0", 1000000, 100, true);

    // // --- PING LOOP TO FIND SERVO ID ---
    // std::cout << "\n--- PING LOOP: Searching for connected servo IDs ---\n";
    // std::vector<uint8_t> found_ids;
    // for (uint8_t id = 1; id <= 253; ++id) {
    //   uint8_t err = servo.ping(id);
    //   if (err != 0xFF) {
    //     std::cout << "Found servo at ID: " << (int)id << ", error code: " << (int)err << "\n";
    //     found_ids.push_back(id);
    //   }
    // }
    // if (found_ids.empty()) {
    //   std::cout << "No servos found.\n";
    // } else {
    //   std::cout << "IDs found: ";
    //   for (auto id : found_ids) std::cout << (int)id << " ";
    //   std::cout << "\n";
    // }

    // --- Continue with original tests (use first found id if any) ---
    // uint8_t 10 = found_ids.empty() ? 10 : found_ids[0];

    std::cout << "\n--- TEST 2: READ SENSOR DATA ---\n";
    auto sensor = servo.read_sensor_data(10);
    if (sensor) {
      std::cout << "Sensor Data: {"
                << "id=" << (int)sensor->id
                << ", position=" << sensor->position
                << ", speed=" << sensor->speed
                << ", load=" << sensor->load
                << ", voltage=" << (int)sensor->voltage
                << ", temperature=" << (int)sensor->temperature
                << "}\n";
    } else {
      std::cout << "Failed to read sensor data.\n";
    }

    std::cout << "\n--- TEST 3: SET GRIPPER POSITION ---\n";
    servo.set_gripper_position(10, "100mm", 10.0, 1000);

    servo.close();
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
