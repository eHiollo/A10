class BusServo {
public:
  struct SensorData {
    uint8_t id{0};
    int16_t position{0};
    int16_t speed{0};
    int16_t load{0};
    uint8_t voltage{0};
    uint8_t temperature{0};
  };

  struct Resp {
    bool ok{false};
    uint8_t error{0xFF};
    std::vector<uint8_t> params;
    std::string msg;
  };

public:
  BusServo(const std::string& port, int baudrate, int timeout_ms, bool verbose);
  ~BusServo();

  void close();

  uint8_t ping(uint8_t servo_id);

  // 兼容旧接口：返回 0~100 (%)，内部用回读 mm 计算
  uint8_t get_position(uint8_t servo_id);

  // ✅ 新增：回读夹爪实际开口（mm）
  std::optional<double> get_position_mm(uint8_t servo_id, const std::string& gripper_type);

  std::optional<std::vector<uint8_t>> read_data(uint8_t servo_id, uint8_t address, uint8_t length);
  std::pair<std::vector<uint8_t>, uint8_t> write_data(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values);
  std::pair<std::vector<uint8_t>, uint8_t> reg_write(uint8_t servo_id, uint8_t address, const std::vector<uint8_t>& values);

  void action(uint8_t servo_id);
  void sync_write(uint8_t address, uint8_t data_len, const std::vector<std::vector<uint8_t>>& servo_data);
  void move_servo(uint8_t servo_id, uint16_t position, uint16_t speed);

  std::optional<SensorData> read_sensor_data(uint8_t servo_id);

  void set_gripper_position(uint8_t servo_id,
                            const std::string& gripper_type,
                            double position_mm,
                            uint16_t speed);

  // cmd01: 0=close, 1=open
  void set_gripper_openclose(uint8_t servo_id,
                             const std::string& gripper_type,
                             int cmd01,
                             uint16_t speed);

private:
  void log_hex_(const std::string& dir, const std::vector<uint8_t>& data);
  uint8_t calc_checksum_(uint8_t id, uint8_t length, uint8_t instruction, const std::vector<uint8_t>& params);
  std::vector<uint8_t> send_packet_(uint8_t id, uint8_t instruction, const std::vector<uint8_t>& params);
  Resp receive_packet_();

  int interpolate_(double value, const std::vector<std::pair<double, int>>& table);

  // ✅ 新增：反向插值 servo_pos -> mm
  double inverse_interpolate_mm_(int servo_pos, const std::vector<std::pair<double, int>>& table);

  void init_calibration_();

private:
  SerialPort serial_;
  bool verbose_{false};
  std::map<std::string, std::vector<std::pair<double, int>>> gripper_calib_;
};
