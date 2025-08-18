# NetCAN

Network bridge for exposing CAN interfaces over TCP. Enables remote CAN access.

## Server

Expose local CAN interface over network:

```python
from openarm.transport.netcan import Server

# Start server
server = Server('can0', port=9999)
server.start()
print("CAN exposed at tcp://0.0.0.0:9999")
```

## Client

Connect to remote CAN interface:

```python
from openarm.transport.netcan import Client

# Connect to remote CAN
client = Client('tcp://robot.local:9999')

# Use like local Bus
client.send(0x123, [0x01, 0x02, 0x03])

# Read messages
for msg in client.read():
    print(f"ID: 0x{msg.arbitration_id:03X}")
```

## CLI Tools

```bash
# Start server
python -m openarm.transport.netcan serve can0 --port 9999

# Send test message
python -m openarm.transport.netcan send tcp://localhost:9999 0x123 0x01,0x02

# Monitor messages
python -m openarm.transport.netcan monitor tcp://localhost:9999
```
