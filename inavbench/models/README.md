# JSBSim models from outside JSBSim's bundled set

Loaded through `Vehicle(model_dir=...)`: `aircraft/<model>/<model>.xml` and `engine/`.

- `aircraft/Rascal/Rascal.xml`, `engine/Zenoah_G-26A.xml`, `engine/18x8.xml`: the Rascal 110 RC trainer, copied from ArduPilot `Tools/autotest/aircraft/Rascal/` (`Rascal.xml`, `Engines/`) at ArduPilot 1f6e646d2a. ArduPilot adapted it from FlightGear's Rascal110. The one change: `Rascal.xml` no longer opens ArduPilot's JSBSim control socket (`<input port="5505"/>`); the bench drives JSBSim in-process. GPL-3.0, as is this bench.
