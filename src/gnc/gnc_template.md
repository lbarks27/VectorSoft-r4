# GNC Modules Template for Teensy Flight Controller

This template follows the **Manager → Methods → Modes** pattern optimized for:

- **Zero heap** (no dynamic allocation)
- **Minimal boilerplate** via a `BaseMode` default interface
- **Lean data buses** per module
- **Static mode switching** using enums and `switch` statements

---

## Directory Layout

```
/src/
└── gnc/
    ├── include/
    │   └── gnc/
    │       ├── IGuidance.h      ← guidance interface
    │       ├── INavigation.h    ← navigation interface
    │       └── IControl.h       ← control interface
    │
    ├── guidance/
    │   ├── GuidanceManager.cpp/.h
    │   ├── GuidanceMethods/     ← pure functions (Trajectory, Intercept)
    │   └── GuidanceModes/       ← each mode inherits BaseGuidanceMode
    │       ├── BaseGuidanceMode.h
    │       ├── WaypointMode.cpp/.h
    │       ├── LandingMode.cpp/.h
    │       └── TestMode.cpp/.h
    │
    ├── navigation/
    │   ├── NavManager.cpp/.h
    │   ├── NavMethods/          ← EkfModel, Quaternion ops
    │   └── NavModes/            ← BaseNavMode, InitMode, HoldMode, CruiseMode
    │       ├── BaseNavMode.h
    │       ├── InitMode.cpp/.h
    │       ├── HoldMode.cpp/.h
    │       └── CruiseMode.cpp/.h
    │
    ├── control/
    │   ├── ControlManager.cpp/.h
    │   ├── ControlMethods/      ← PID, LQR
    │   └── ControlModes/        ← BaseControlMode, AttitudeHold, ThrustVec
    │       ├── BaseControlMode.h
    │       ├── AttitudeHold.cpp/.h
    │       └── ThrustVec.cpp/.h
    │
    └── Scheduler.cpp/.h         ← cooperative scheduler runner
```

---

## 1. Lean Interfaces

```cpp
// IGuidance.h
#pragma once
#include "FlightState.h"
struct GuidanceCommand { float throttle; /* etc. */ };
class IGuidance {
public:
  virtual ~IGuidance() = default;
  virtual bool init() = 0;
  virtual bool update(const FlightState &state) = 0;
  virtual GuidanceCommand getCommand() const = 0;
};
```

```cpp
// INavigation.h
#pragma once
#include "SensorBus.h"
#include "NavigationState.h"
class INavigation {
public:
  virtual ~INavigation() = default;
  virtual bool init() = 0;
  virtual bool update(const SensorBus &bus, NavigationState &out) = 0;
};
```

```cpp
// IControl.h
#pragma once
#include "NavigationState.h"
#include "IGuidance.h"
struct ControlCommand { float pwm1, pwm2, pwm3; };
class IControl {
public:
  virtual ~IControl() = default;
  virtual bool init() = 0;
  virtual bool update(const NavigationState &nav,
                      const GuidanceCommand &gcmd,
                      ControlCommand &out) = 0;
};
```

---

## 2. BaseMode Classes

```cpp
// guidance/GuidanceModes/BaseGuidanceMode.h
#pragma once
#include "../../include/gnc/FlightState.h"
#include "../../include/gnc/IGuidance.h"

class BaseGuidanceMode {
public:
  virtual ~BaseGuidanceMode() = default;
  virtual bool onEnter() { return true; }
  virtual bool onUpdate(const FlightState &state,
                        GuidanceCommand &cmd) { return true; }
  virtual void onExit() {}
};
```

*(Similarly define **`BaseNavMode`** and **`BaseControlMode`** in their respective folders.)*

---

## 3. GuidanceManager (Static Modes)

```cpp
// guidance/GuidanceManager.h
#pragma once
#include "../../include/gnc/IGuidance.h"
#include "GuidanceModes/BaseGuidanceMode.h"
#include "GuidanceModes/WaypointMode.h"
#include "GuidanceModes/LandingMode.h"
#include "GuidanceModes/TestMode.h"

class GuidanceManager : public IGuidance {
public:
  enum class ModeId { Waypoint, Landing, Test };

  bool init() override {
    mode = ModeId::Waypoint;
    return enterMode();
  }

  bool update(const FlightState &state) override {
    switch (mode) {
      case ModeId::Waypoint: return waypoint.onUpdate(state, lastCmd);
      case ModeId::Landing:  return landing.onUpdate(state, lastCmd);
      case ModeId::Test:     return test.onUpdate(state, lastCmd);
    }
    return false;
  }

  GuidanceCommand getCommand() const override {
    return lastCmd;
  }

  void switchMode(ModeId newMode) {
    exitMode(); mode = newMode; enterMode();
  }

private:
  ModeId mode;
  GuidanceCommand lastCmd;
  WaypointMode waypoint;
  LandingMode  landing;
  TestMode     test;

  bool enterMode() {
    switch (mode) {
      case ModeId::Waypoint: return waypoint.onEnter();
      case ModeId::Landing:  return landing.onEnter();
      case ModeId::Test:     return test.onEnter();
    }
    return false;
  }
  void exitMode() {
    switch (mode) {
      case ModeId::Waypoint: waypoint.onExit(); break;
      case ModeId::Landing:  landing.onExit();  break;
      case ModeId::Test:     test.onExit();     break;
    }
  }
};
```

*(Repeat a similar pattern for **`NavManager`** with **`NavModes`** and **`ControlManager`** with **`ControlModes`**.)*

---

## 4. Lean Data Buses

```cpp
// include/gnc/SensorBus.h
#pragma once
#include "../drivers/IImu.h"
#include "../drivers/IBarometer.h"
struct SensorBus {
  ImuData  imu;
  BaroData baro;
};
```

```cpp
// include/gnc/NavigationState.h
#pragma once
struct NavigationState {
  float x, y, z;
  float vx, vy, vz;
  float q[4];  // quaternion
};
```

*(ControlCommand and GuidanceCommand defined in interfaces.)*

---

## 5. Scheduler & Flight Logic (Skeleton)

```cpp
// Scheduler.cpp
#include <functional>
#include <array>
#include "../utils/Timer.h"
#include "include/gnc/SensorBus.h"

using TaskFn = std::function<void()>;
struct Task { uint32_t period; uint32_t next; TaskFn fn; };

static std::array<Task,5> tasks;

void setupTasks(IGuidance &guid, INavigation &nav, IControl &ctrl) {
  SensorBus bus;
  NavigationState navState;
  GuidanceCommand gcmd;
  ControlCommand ccmd;

  tasks = {{
    {2, Timer::now()+2,  [&]{ /* read sensors into bus */ }},
    {20,Timer::now()+20,[&]{ nav.update(bus, navState); }},
    {50,Timer::now()+50,[&]{ guid.update(/* FlightState stub */); gcmd = guid.getCommand(); }},
    {50,Timer::now()+50,[&]{ ctrl.update(navState, gcmd, ccmd); }},
    {100,Timer::now()+100,[&]{ /* telemetry, logging */ }}
  }};
}

void loopTasks() {
  uint32_t now = Timer::now();
  for (auto &t : tasks) {
    if (now >= t.next) { t.fn(); t.next += t.period; }
  }
}
```

Call `setupTasks(...)` in `setup()`, then repeatedly `loopTasks()` in `loop()`.

---

**This template uses**

- **Static mode instances** (no heap)
- **Enum + switch** for mode logic
- **BaseMode** defaults to reduce boilerplate
- **Minimal buses** to pass only needed data

*Save as **`GNC_MODULES_TEMPLATE.md`**. Adapt concrete math and mode logic as you implement each subsystem.*

