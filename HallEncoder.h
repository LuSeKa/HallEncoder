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
  uint8_t pin_A_ = 0;
  uint8_t pin_B_ = 0;
  uint8_t pin_Z_ = 0;
  uint8_t last_state_ = 0;
  long int position_ = 0;
  unsigned long last_change_time_us_ = 0;
  float velocity_cps_ = 0;
  int hall_state_table_[8];
  
  uint8_t ReadHallState();
  int GetStateChange(uint8_t state);
    
};

#endif // HallEncoder_h
