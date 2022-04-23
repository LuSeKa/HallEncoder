#include <Command_processor.h> // https://github.com/LuSeKa/command_processor
#include <HallEncoder.h>
#include <Metro.h> // https://github.com/LuSeKa/Metro

Command_processor cmd;
HallEncoder hall = HallEncoder(8, 9, 10);
bool position_stream_enabled = 0;
bool velocity_stream_enabled = 0;
bool state_stream_enabled = 0;
Metro stream_metro = Metro(50);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  cmd.add_command('c', &CalibrateWrapper, 0, "Initiate calibration.");
  cmd.add_command('V', &VelocityStreamWrapper, 1, "Control velocity stream.");
  cmd.add_command('P', &PositionStreamWrapper, 1, "Control position stream.");
  cmd.add_command('H', &StateStreamWrapper, 1, "Control Hall state stream.");
  cmd.add_command('R', &RawPositionWrapper, 0, "Print raw position (0 - 5).");
  cmd.add_command('s', &StateWrapper, 0, "Print Hall state.");
  cmd.add_command('r', &ResetWrapper, 0, "Reset position.");

  // hall.WriteCalibration(1,5,0,3,2,4);
}

void loop() {
  // put your main code here, to run repeatedly:
  hall.Update();
  cmd.parse_command();
  if (stream_metro.check()) {
    Stream();
  }
}

void Stream() {
  if (state_stream_enabled) {
    Serial.print(hall.GetState());
    Serial.print('\t');
  }
  if (position_stream_enabled) {
    Serial.print(hall.GetPosition());
    Serial.print('\t');
  }
  if (velocity_stream_enabled) {
    Serial.print(hall.GetVelocity());
    Serial.print('\t');
  }
  if (position_stream_enabled || velocity_stream_enabled || state_stream_enabled) {
    Serial.println();
  }
}

void CalibrateWrapper(float foo, float bar) {
  Serial.println("Initiating Hall encoder calibration");
  hall.Calibrate();
}

void ResetWrapper(float foo, float bar) {
  hall.Reset();
}

void PositionStreamWrapper(float enabled, float bar) {
  position_stream_enabled = enabled != 0.0;
}

void VelocityStreamWrapper(float enabled, float bar) {
  velocity_stream_enabled = enabled != 0.0;
}

void StateStreamWrapper(float enabled, float bar) {
  state_stream_enabled = enabled != 0.0;
}

void RawPositionWrapper(float foo, float bar) {
  Serial.println(hall.GetRawPosition());
}

void StateWrapper(float foo, float bar) {
  Serial.println(hall.GetState());
}
