# OpenArm Transport

CAN bus message multiplexer for filtering messages by arbitration ID. Wraps `python-can` with message queuing.

## Quick Start

```python
import can
from openarm.transport import Bus

# Create CAN interface
can_bus = can.Bus('can0', interface='socketcan')

# Wrap with multiplexer
bus = Bus(can_bus)

# Send message
msg = can.Message(arbitration_id=0x123, data=[0x01, 0x02, 0x03, 0x04])
bus.send(msg)

# Receive specific message by ID
response = bus.recv(arbitration_id=0x124, timeout=1.0)
if response:
    print(f"Received: {response.data.hex()}")
```

## Message Filtering

The multiplexer queues messages by arbitration ID, allowing selective reading:

```python
# Send motor command
motor_cmd = can.Message(arbitration_id=0x001, data=[0xFF, 0xFF, 0xFF, 0xFF])
bus.send(motor_cmd)

# Send sensor request
sensor_req = can.Message(arbitration_id=0x105, data=[0x33])
bus.send(sensor_req)

# Read responses independently
motor_response = bus.recv(0x001, timeout=0.1)    # Only motor messages
sensor_response = bus.recv(0x105, timeout=0.1)   # Only sensor messages
```

## Network Extension

For remote CAN access, see [`netcan/`](netcan/) submodule.
