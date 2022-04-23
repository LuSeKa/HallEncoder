# HallEncoder

A simple class that provides access to the Hall sensors of a hoverboard motor and exposes an encoder-like interface for calibration, position and velocity.
Velocity is estimated using a moving average implemented as a ring buffer. 
The size of the ring buffer and the sample interval are the parameters that determine the characteristics of the velocity estimate. Choose wisely :)
The calibration routine determines the sequence of Hall states that make up one "Hall rotation" (i.e. six steps).
