# Hardware Drivers Template for Teensy Flight Controller

This template implements all hardware peripherals (IMU, ESC, LIDAR, Servo, Pyro, Comm, SD Logger) following the **Interface → Stub → Real → Factory** pattern.

---

## Directory Layout

```
/  
├── include/  
│   └── drivers/  
│       ├── IImu.h  
│       ├── IEsc.h  
│       ├── ILidar.h  
│       ├── IServo.h  
│       ├── IPyro.h  
│       ├── IComm.h  
│       └── ISdLogger.h  
│
├── src/  
│   ├── drivers/  
│   │   ├── imu_stub.cpp/.h  
│   │   ├── imu_real.cpp/.h  
│   │   ├── esc_stub.cpp/.h  
│   │   ├── esc_real.cpp/.h  
│   │   └── ... (other peripherals)  
│   │   ├── comm_stub...  
│   │   └── sdcard_real...  
│   └── factories/  
│       ├── ImuFactory.h  
│       ├── EscFactory.h  
│       └── ... (one per peripheral)  
│  
└── main.cpp  
```

---

## 1. Interfaces (`include/drivers/*.h`)

```cpp
// IImu.h
#pragma once
#include <cstdint>

struct ImuData {
    float ax, ay, az;       // Acceleration (m/s²)
    float gx, gy, gz;       // Angular rate (rad/s)
    uint32_t timestamp;     // Micros or millis timestamp
};

class IImu {
public:
    virtual ~IImu() = default;
    virtual bool begin() = 0;
    virtual bool isHealthy() const = 0;
    virtual bool read(ImuData &out) = 0;
};
```

Repeat similarly for IEsc, ILidar, IServo, IPyro, IComm, and ISdLogger, defining appropriate data/command structs.

---

## 2. Stubs (`src/drivers/*_stub.cpp/.h`)

```cpp
// imu_stub.h
#pragma once
#include "drivers/IImu.h"

class ImuStub : public IImu {
public:
    bool begin() override { return true; }
    bool isHealthy() const override { return true; }
    bool read(ImuData &out) override {
        out = {0,0,0, 0,0,0, millis()};
        return true;
    }
};
```

All stub implementations simply return success or default data.

---

## 3. Real Drivers (`src/drivers/*_real.cpp/.h`)

```cpp
// imu_real.h
#pragma once
#include "drivers/IImu.h"
#include <Adafruit_BNO055.h>
#include <Wire.h>

class ImuReal : public IImu {
public:
    ImuReal(TwoWire &i2c = Wire): _bno(55), _i2c(i2c) {}
    bool begin() override {
        _i2c.begin();
        return _bno.begin() && _bno.setExtCrystalUse(true);
    }
    bool isHealthy() const override {
        uint8_t sys, err, st;
        return _bno.getSystemStatus(&sys, &err, &st) && err == 0;
    }
    bool read(ImuData &out) override {
        imu::Vector<3> a = _bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
        imu::Vector<3> g = _bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
        out.ax = a.x(); out.ay = a.y(); out.az = a.z();
        out.gx = g.x(); out.gy = g.y(); out.gz = g.z();
        out.timestamp = micros();
        return true;
    }

private:
    Adafruit_BNO055 _bno;
    TwoWire &_i2c;
};
```

Implement each real driver with the actual hardware calls (e.g., `analogWrite` for ESC, `VL53L0X` library for LIDAR, `Servo` library for Servo, digital pin for Pyro, SoftwareSerial/RFD900x for Comm, `SD` for SD Logger).

---

## 4. Factories (`src/factories/*.h`)

```cpp
// ImuFactory.h
#pragma once
#include "drivers/IImu.h"
#include <drivers/imu_stub.h>
#include <drivers/imu_real.h>

struct ImuFactory {
    enum class Type { Stub, Real };
    static IImu* create(Type t, TwoWire &i2c = Wire) {
        static ImuStub stub;
        static ImuReal real{i2c};
        return (t == Type::Real) ? &real : &stub;
    }
};
```

Repeat for each peripheral, passing any required pins or interface objects into the real constructor.

---

## 5. Example Usage (`main.cpp`)

```cpp
#include <Arduino.h>
#include "factories/ImuFactory.h"
#include "factories/EscFactory.h"

IImu *imu;
IEsc *esc;

void setup() {
  Serial.begin(115200);

  imu = ImuFactory::create(ImuFactory::Type::Real, Wire);
  esc = EscFactory::create(EscFactory::Type::Real, /*pwmPin=*/9);

  if (!imu->begin() || !imu->isHealthy()) {
    Serial.println("IMU init failed");
    while (1) delay(100);
  }
  if (!esc->begin() || !esc->isHealthy()) {
    Serial.println("ESC init failed");
    while (1) delay(100);
  }
}

void loop() {
  // 1) Read IMU
  ImuData m;
  if (imu->read(m)) {
    Serial.printf("AX=%.2f AY=%.2f AZ=%.2f\n", m.ax, m.ay, m.az);
  }

  // 2) Command ESC at 50%
  EscCommand cmd{ .throttle = 0.5f };
  esc->command(cmd);

  delay(100); // 10 Hz
}
```

---

**This markdown file is ready to add to your repo (e.g., **``**).**

