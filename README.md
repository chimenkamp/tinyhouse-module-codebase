# 🌡️ IoT Sensor Management System

Raspberry Pi and Arduino Nano configuration that automatically detects sensors, collects data, and provides real-time monitoring with alerts.

![Version](https://img.shields.io/badge/version-1.0.0-blue)
![Python](https://img.shields.io/badge/python-3.11+-green)
![Arduino](https://img.shields.io/badge/arduino-nano-red)
![License](https://img.shields.io/badge/license-MIT-purple)
![Platform](https://img.shields.io/badge/platform-AlmaLinux-orange)

## ✨ Features

### Arduino Features
- 🔍 **Auto-Detection**: Automatically identifies connected sensors
- 📊 **Multi-Sensor Support**: DHT22, DS18B20, BMP280, HC-SR04, analog sensors
- 💬 **UART Communication**: Reliable serial protocol with error handling
- ⚡ **Real-time Data**: Configurable sampling intervals
- 💓 **Heartbeat Monitoring**: Connection health checks

### Raspberry Pi Features
- 💾 **SQLite Database**: Persistent storage with automatic backups
- 🚨 **Alert System**: Configurable thresholds with notification support
- 📈 **Statistics**: Min/max/average calculations and trends
- 🔄 **Auto-Reconnection**: Handles connection failures gracefully
- 🖥️ **Interactive CLI**: Real-time management interface
- 🔧 **Hot Configuration**: Change settings without restart
- 📤 **Data Export**: CSV export for analysis
- 🎯 **Service Mode**: Runs as systemd daemon

## 🏗️ System Architecture

```
┌──────────────────┐           UART/Serial        ┌──────────────────┐
│                  │◄────────────────────────────►│                  │
│   Arduino Nano   │         TX/RX Connection     │   Raspberry Pi   │
│                  │                              │   (AlmaLinux)    │
└────────┬─────────┘                              └──────────────────┘
         │                                                  │
    ┌────▼────┐                                       ┌─────▼─────┐
    │ Sensors │                                       │  SQLite   │
    └─────────┘                                       │ Database  │
                                                      └───────────┘
```

### Communication Protocol

Messages use a structured format: `<TYPE|TIMESTAMP|DATA>`

**Message Types:**
- `DATA`: Sensor readings
- `INVENTORY`: Sensor list
- `HEARTBEAT`: Connection status
- `CONFIG`: Configuration commands
- `STATUS`: System status

## 🔧 Hardware Requirements

### Minimum Requirements
- **Raspberry Pi**: Any model (tested on Pi 3B+, 4)
- **Arduino Nano**: ATmega328P based
- **Storage**: 1GB free space minimum
- **Connection**: USB cable (USB-A to Mini-USB)

### Supported Sensors
| Sensor | Type | Interface | Measurements |
|--------|------|-----------|--------------|
| DHT22 | Digital | OneWire | Temperature, Humidity |
| DS18B20 | Digital | OneWire | Temperature |
| BMP280 | Digital | I2C | Temperature, Pressure, Altitude |
| HC-SR04 | Digital | Trigger/Echo | Distance |
| Analog | Analog | ADC | Voltage (0-5V) |

### Wiring Diagram
```
Arduino Nano Pin Layout:
├── D2  → DHT22 Data Pin
├── D3  → DS18B20 Data Pin (4.7kΩ pullup)
├── D4  → HC-SR04 Trigger
├── D5  → HC-SR04 Echo
├── A0-A5 → Analog Sensors
├── A4  → SDA (BMP280)
├── A5  → SCL (BMP280)
├── 5V  → Sensor VCC
├── GND → Sensor GND
└── USB → Raspberry Pi
```

## 💻 Software Requirements

### Raspberry Pi (AlmaLinux)
- Python 3.7+
- pip3
- SQLite3
- Git
- Arduino CLI (optional, for uploading sketches)

### Python Dependencies
```
pyserial>=3.5
```

### Arduino Libraries
- DHT sensor library
- OneWire
- DallasTemperature
- Adafruit BMP280 Library

## 📦 Installation

### Quick Install (Recommended)

1. **Clone the repository:**
```bash
git clone https://github.com/chimenkamp/tinyhouse-module-codebase.git
cd tinyhouse-module-codebase
```

2. **Run the setup script:**
```bash
chmod +x install_setup_raspberry.sh
./install_setup_raspberry.sh
```

3. **Upload Arduino sketch:**
```bash
./upload_sketch.sh
```

### Manual Installation

1. **Install Python dependencies:**
```bash
python3 -m venv venv
source venv/bin/activate
pip install pyserial
```

2. **Configure serial port permissions:**
```bash
sudo usermod -a -G dialout $USER
# Logout and login again for changes to take effect
```

3. **Upload Arduino sketch:**
   - Open `arduino/main.ino` in Arduino IDE
   - Select Board: Arduino Nano
   - Select Processor: ATmega328P (Old Bootloader)
   - Upload sketch

4. **Configure system:**
```bash
cp iot_config.ini.example iot_config.ini
# Edit configuration as needed
nano iot_config.ini
```

## ⚙️ Configuration

### Configuration File (`iot_config.ini`)

```ini
[SERIAL]
port = /dev/ttyUSB0        # Serial port
baudrate = 115200           # Communication speed
timeout = 1                 # Read timeout in seconds

[DATABASE]
path = iot_sensors.db       # Database file location
retention_days = 30         # Days to keep data
backup_enabled = true       # Enable automatic backups
backup_interval_hours = 24  # Backup frequency

[MONITORING]
sensor_read_interval = 2000 # Milliseconds between readings
heartbeat_timeout = 30      # Seconds before timeout
auto_reconnect = true       # Auto-reconnect on failure
max_reconnect_attempts = 10 # Maximum reconnection attempts

[ALERTS]
enabled = true              # Enable alert system
temp_min = -10             # Minimum temperature (°C)
temp_max = 50              # Maximum temperature (°C)
humidity_min = 20          # Minimum humidity (%)
humidity_max = 80          # Maximum humidity (%)
distance_min = 5           # Minimum distance (cm)
distance_max = 200         # Maximum distance (cm)

[LOGGING]
level = INFO               # DEBUG, INFO, WARNING, ERROR
file = iot_system.log      # Log file location
max_size_mb = 100          # Maximum log size
backup_count = 5           # Number of log backups
```

## 🚀 Usage

### Starting the System

**Interactive Mode (with CLI):**
```bash
./start_iot.sh
```

**Daemon Mode (background service):**
```bash
# Enable service
sudo systemctl enable iot-manager.service

# Start service
sudo systemctl start iot-manager.service

# Check status
sudo systemctl status iot-manager.service

# View logs
journalctl -u iot-manager.service -f
```

### CLI Commands

| Command | Description | Example |
|---------|-------------|---------|
| `status` | Show system status | `> status` |
| `sensors` | List active sensors | `> sensors` |
| `detect` | Force sensor detection | `> detect` |
| `config` | Display configuration | `> config` |
| `stats` | Show statistics | `> stats` |
| `alerts` | View alerts | `> alerts` |
| `export` | Export data to CSV | `> export` |
| `set` | Update configuration | `> set MONITORING sensor_read_interval 5000` |
| `quit` | Exit the program | `> quit` |

### Monitoring

**View real-time logs:**
```bash
./monitor.sh
# or
tail -f iot_system.log
```

**Query database directly:**
```bash
./query_db.sh "SELECT * FROM sensor_data ORDER BY timestamp DESC LIMIT 10;"
```

**Export data:**
```bash
# Via CLI
> export

# Via script
sqlite3 iot_sensors.db ".headers on" ".mode csv" \
  "SELECT * FROM sensor_data;" > export.csv
```

## 📡 Supported Sensors

### DHT22 (Temperature & Humidity)
- **Pin**: D2
- **Range**: -40°C to 80°C, 0-100% RH
- **Accuracy**: ±0.5°C, ±2% RH

### DS18B20 (Temperature)
- **Pin**: D3 (with 4.7kΩ pullup)
- **Range**: -55°C to 125°C
- **Accuracy**: ±0.5°C

### BMP280 (Pressure & Altitude)
- **Interface**: I2C (A4/A5)
- **Range**: 300-1100 hPa
- **Accuracy**: ±1 hPa

### HC-SR04 (Ultrasonic Distance)
- **Pins**: D4 (Trigger), D5 (Echo)
- **Range**: 2cm to 400cm
- **Accuracy**: ±3mm

### Analog Sensors
- **Pins**: A0-A5
- **Range**: 0-5V
- **Resolution**: 10-bit (0-1023)

## 🗄️ Database Schema

### Tables

**sensors**
```sql
CREATE TABLE sensors (
    id INTEGER PRIMARY KEY,
    sensor_id TEXT UNIQUE,
    sensor_type INTEGER,
    pin INTEGER,
    first_seen TIMESTAMP,
    last_seen TIMESTAMP,
    active BOOLEAN,
    metadata TEXT
);
```

**sensor_data**
```sql
CREATE TABLE sensor_data (
    id INTEGER PRIMARY KEY,
    sensor_id TEXT,
    timestamp TIMESTAMP,
    value1 REAL,
    value2 REAL,
    value3 REAL,
    unit1 TEXT,
    unit2 TEXT,
    unit3 TEXT,
    raw_data TEXT
);
```

**alerts**
```sql
CREATE TABLE alerts (
    id INTEGER PRIMARY KEY,
    timestamp TIMESTAMP,
    sensor_id TEXT,
    alert_type TEXT,
    value REAL,
    threshold REAL,
    message TEXT,
    acknowledged BOOLEAN
);
```

## 📚 API Reference

### Arduino → Raspberry Pi Messages

**Data Message:**
```
<DATA|12345|DHT22,23.5,65.2,temp_C,humidity_%>
```

**Inventory Message:**
```
<INVENTORY|12345|3|DHT22:1,DS18B20_0:2,BMP280:3>
```

**Heartbeat Message:**
```
<HEARTBEAT|12345|OK|3|1024>
```

### Raspberry Pi → Arduino Commands

**Configuration:**
```
<CONFIG|INTERVAL|5000>
<CONFIG|DEBUG|1>
<CONFIG|AUTODETECT|0>
```

**Control:**
```
<DETECT>
<RESET>
<STATUS>
```

## 🔧 Troubleshooting

### Common Issues

**Serial Port Not Found**
```bash
# List available ports
ls /dev/tty*

# Update configuration
nano iot_config.ini
# Change port to correct value (e.g., /dev/ttyACM0)
```

**Permission Denied**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Logout and login again
```

**No Sensor Data**
1. Check Arduino serial monitor (115200 baud)
2. Verify sensor connections
3. Run sensor detection: `> detect`
4. Check logs: `tail -f iot_system.log`

**Database Locked**
```bash
# Stop the service
sudo systemctl stop iot-manager.service
# Remove lock
rm iot_sensors.db-journal
# Restart service
sudo systemctl start iot-manager.service
```

**High CPU Usage**
- Increase `sensor_read_interval` in config
- Check for sensor errors in logs
- Reduce logging level from DEBUG to INFO

### Debug Mode

Enable debug logging:
```ini
[LOGGING]
level = DEBUG
```

Monitor Arduino serial directly:
```bash
screen /dev/ttyUSB0 115200
# Press Ctrl+A, then K to exit
```

## 🤝 Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit changes (`git commit -m 'Add AmazingFeature'`)
4. Push to branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

### Development Setup

```bash
# Clone repository
git clone https://github.com/yourusername/iot-sensor-system.git
cd iot-sensor-system

# Create virtual environment
python3 -m venv venv
source venv/bin/activate

# Install development dependencies
pip install -r requirements-dev.txt

# Run tests
python -m pytest tests/
```

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
