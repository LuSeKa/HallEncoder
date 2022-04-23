# HallEncoder

A simple class that provides to the Hall sensors of a hoverboard motor and exposes an encoder-like interface for calibration, position and velocity.
Velocity is estmated using a moving average implemented as a ring buffer. 
The size of the ring buffer and the sample interval are the parameters that determine the characteristics of the velocity estimate. Choose wisely :)
The calibation routine determines the squence of Hall states that make up one "Hall rotation" (six steps).
