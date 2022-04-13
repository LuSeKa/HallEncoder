#ifndef HallEncoder_h
#define HallEncoder_h

class HallEncoder{
  public:
  HallEncoder(uint8_t pin_A, uint8_t pin_B, uint8_t pin_Z);

  int Calibrate();
  void WriteCalibration(uint8_t pos_1, uint8_t pos_2, uint8_t pos_3, uint8_t pos_4, uint8_t pos_5, uint8_t pos_6);
  int Update();
  float GetVelocity();
  long int GetPosition();
  void Reset();
  int GetState();
  int GetRawPosition();
  
  private:
  static const unsigned long kWindowSize_ = 20;
  static const unsigned long kSamplingIntervalUs_ = 10000;
  uint8_t pin_A_ = 0;
  uint8_t pin_B_ = 0;
  uint8_t pin_Z_ = 0;
  uint8_t last_state_ = 0;
  long int position_ = 0;
  long int oldest_position_ = 0;
  unsigned long last_sampling_time_ = 0;
  unsigned long last_change_time_us_ = 0;
  float velocity_cps_ = 0;
  int hall_state_table_[8];
  // Sliding window for moving average velocity estimation.
  // Using a sliding averge to achieve a better trade-off between low speed accuracy and lag than a single dx/dt with fixed dt. 
  long int position_window_[kWindowSize_];
  uint8_t sampling_index_ = 0;
  
  uint8_t ReadHallState();
  int GetStateChange(uint8_t state);
    
};

#endif // HallEncoder_h
