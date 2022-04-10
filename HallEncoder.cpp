#include "Arduino.h"
#include "HallEncoder.h"

HallEncoder::HallEncoder(uint8_t pin_A, uint8_t pin_B, uint8_t pin_Z) {
    pin_A_ = pin_A;
    pin_B_ = pin_B;
    pin_Z_ = pin_Z;
    pinMode(pin_A_, INPUT_PULLUP);
    pinMode(pin_B_, INPUT_PULLUP);
    pinMode(pin_Z_, INPUT_PULLUP);
    // Index is the Hall state, value is the associated encoder position.
    hall_state_table_[0] = -1; // Invalid Hall state.
    hall_state_table_[1] = 0;
    hall_state_table_[2] = 0;
    hall_state_table_[3] = 0;
    hall_state_table_[4] = 0;
    hall_state_table_[5] = 0;
    hall_state_table_[6] = 0;
    hall_state_table_[7] = -1; // Invalid Hall state.
}

float HallEncoder::GetVelocity() {
    return velocity_cps_;
}

long int HallEncoder::GetPosition() {
    return position_;
}

void HallEncoder::WriteCalibration(uint8_t pos_1, uint8_t pos_2, uint8_t pos_3, uint8_t pos_4, uint8_t pos_5, uint8_t pos_6) {
    hall_state_table_[1] = pos_1;
    hall_state_table_[2] = pos_2;
    hall_state_table_[3] = pos_3;
    hall_state_table_[4] = pos_4;
    hall_state_table_[5] = pos_5;
    hall_state_table_[6] = pos_6;
}
    
int HallEncoder::GetState() {
    return last_state_;
}

int HallEncoder::GetRawPosition() {
    return hall_state_table_[last_state_];
}

// Blocking calibration routine.
// Notes the six Hall states as the encoder is turned.
// Returns 0 if calibrated with no errores.
int HallEncoder::Calibrate() {
    Serial.println("Slowly turn the wheel forwards.");
    uint8_t state_transition_counter = 0;
    uint8_t previous_hall_state = 255;
    while (state_transition_counter < 6) {
        uint8_t state = ReadHallState();
        if (state != previous_hall_state) {
            hall_state_table_[state] = state_transition_counter;
            Serial.println("Index: " + String(state)+" Position: " + String(state_transition_counter));
            state_transition_counter += 1;
            previous_hall_state = state;
        }
        if (state < 1 || state > 6) {
            // Invalid Hall state.
            Serial.println("Calibration failed. Result may be invalid.");
            return -1;
        }
        // ToDo(LuSeKa): Check if this state was previously seen.
    }
    Serial.println("Calibration finished successfully.");
    return 0;
}

void HallEncoder::Reset() {
    position_ = 0;
}


uint8_t HallEncoder::ReadHallState() {
  uint8_t a = digitalRead(pin_A_);
  uint8_t b = digitalRead(pin_B_);
  uint8_t z = digitalRead(pin_Z_);
  // a, b and z are the three least significant bits of the result.
  uint8_t state = a << 2 | b << 1 | z;
  if (state < 1 || state > 6) {
            // Invalid Hall state.
            Serial.println("Calibration failed. Result may be invalid.");
            return -1;
        }
        return state;
}

int HallEncoder::Update() {
  unsigned long now = micros();
  uint8_t state = ReadHallState(); // Fingers crossed that the state does not change while we sample it.
  int state_change = GetStateChange(state); // -1 or 1
  last_state_ = state;
  if (abs(state_change) == 1) {
      // Postion increased or decreased by one.
      position_ += state_change;
      velocity_cps_ = 1000000.0 * (float)state_change / (float)(micros() - last_change_time_us_);
      last_change_time_us_ = now;
  }
  else if (state_change != 0) {
      // A Hall error occured.
      return -1;
  }
  return 0;
}

/* Compare the current to the previous encoder state.
 * 0: Unchanged
 * -1: Decreased
 * 2: Increased
 * All others: Error codes.
 */
int HallEncoder::GetStateChange(uint8_t state) {
    // Check for Hall state validity. 
    if (hall_state_table_[last_state_] == -1) {
        // Last hall state is invalid.
        return 10;
    }
    if (hall_state_table_[state] == -1) {
        // current hall state is invalid.
        return 20;
    }
    
    int last_pos = hall_state_table_[last_state_];
    int new_pos = hall_state_table_[state];
    // See what changed.
    if (new_pos == (last_pos + 1) % 6) {
        // Serial.println("Encoder position has increased.");
        return 1;
    }
    else if (((last_pos - 1) % 6) == new_pos || (last_pos == 0 && new_pos == 5)) {
        // Serial.println("Encoder position has decreased.");
        return -1;
    }
    else if (new_pos == last_pos) {
        // Encoder position has not changed.
        return 0;
    }
    else {
        Serial.print(last_pos);
        Serial.print('\t');
        Serial.print(new_pos);
        Serial.print('\t');
        Serial.println("Encoder has skipped at least one step.");
        return 30;
    }
}
