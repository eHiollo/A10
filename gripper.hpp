#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <optional>
#include <utility>

// ===== Utilities =====
void sleep_ms(int ms);
void push_u16_le(std::vector<uint8_t>& v, uint16_t x);
int16_t read_i16_le(const uint8_t* p);

// ===================== SerialPort (Linux-only) =====================
class SerialPort {
public:
  SerialPort();
  ~SerialPort();

  bool open(const std::string& port, int baudrate, int timeout_ms);
  void close();

  bool isOpen() const;
  void resetInputBuffer();

  bool writeAll(const std::vector<uint8_t>& data);
  std::optional<std::vector<uint8_t>> readExact(size_t n, int timeout_ms_override = -1);

private:
  bool set_baudrate_linux_(int baudrate);

private:
  int fd_{-1};
  int timeout_ms_{100};
  bool is_open_{false};
};

// ===================== BusServo =====================
class BusServo {
public:
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

  explicit BusServo(const std::string& port,
                    int baudrate = 115200,
                    int timeout_ms = 100,
                    bool verbose = true);
  ~BusServo();

  void close();

  uint8_t ping(uint8_t servo_id);

  std::optional<std::vector<uint8_t>> read_data(uint8_t servo_id, uint8_t address, uint8_t length);

  std::pair<std::vector<uint8_t>, uint8_t>
  write_data(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values);

  std::pair<std::vector<uint8_t>, uint8_t>
  reg_write(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values);

  void action(uint8_t servo_id = ID_BROADCAST);

  void sync_write(uint8_t address, uint8_t data_len, const std::vector<std::vector<uint8_t>>& servo_data);

  void move_servo(uint8_t servo_id, uint16_t position, uint16_t speed);

  void set_servo_id(uint8_t old_id, uint8_t new_id);
  void set_baudrate(uint8_t servo_id, int baudrate);
  void set_middle_position(uint8_t servo_id);
  void set_servo_torque_enable(uint8_t servo_id, bool enable = true);

  std::optional<int16_t> get_position(uint8_t servo_id);
  std::optional<SensorData> read_sensor_data(uint8_t servo_id);

  void set_gripper_position(uint8_t servo_id,
                            const std::string& gripper_type,
                            double position_mm,
                            uint16_t speed);

private:
  struct Resp {
    bool ok{false};
    uint8_t error{0};
    std::vector<uint8_t> params;
    std::string msg;
  };

  void init_calibration_();
  int interpolate_(double value, const std::vector<std::pair<double, int>>& table);

  void log_hex_(const std::string& dir, const std::vector<uint8_t>& data);
  uint8_t calc_checksum_(uint8_t id, uint8_t length, uint8_t instruction, const std::vector<uint8_t>& params);
  std::vector<uint8_t> send_packet_(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params);
  Resp receive_packet_();

private:
  SerialPort serial_;
  bool verbose_{true};
  std::map<std::string, std::vector<std::pair<double, int>>> gripper_calib_;
};
