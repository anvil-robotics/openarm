# Damiao Package

## Code Architecture Plan

This package follows a structured approach to organize the codebase:

### File Structure

- **`__init__.py`** - High-level code and main package interface
  - Contains the primary APIs and entry points
  - Exposes the main functionality to package users
  - High-level abstractions and orchestration

- **`encoding.py`** - Encoder/decoder and low-level implementation
  - Protocol encoding and decoding logic
  - Low-level communication primitives
  - Data serialization/deserialization

- **`__main__.py`** - Tools and executables
  - Command-line interface
  - Utility scripts
  - Executable entry points for package functionality

### Design Principles

The architecture separates concerns into distinct layers:
- **Interface Layer** (`__init__.py`) - What users interact with
- **Implementation Layer** (`encoding.py`) - How things work internally  
- **Tool Layer** (`__main__.py`) - Command-line and utility access

This structure promotes maintainability and clear separation of responsibilities.

## Encoding Implementation Details

### Bus Integration

The `encoding.py` module integrates with the CAN bus system through the `openarm.bus` package:

- **Bus Dependency**: Uses the `Bus` class from `openarm.bus` for all CAN message operations
- **Synchronous Sending**: Data transmission is immediate and synchronous - the SDK sends requests without async/await
- **Asynchronous Receiving**: Data reception must be implemented asynchronously to handle incoming messages

### Communication Pattern

```python
from typing import Coroutine, Any
import can
import struct
from openarm.bus import Bus

# Encoding (synchronous)
def encode_set_position(bus: Bus, motor_id: int, position: float) -> None:
    # Pack position value as little-endian 32-bit binary data
    # Format: '<f' = little-endian (<) + float (f)
    # Reference: Position command format in docs/damiao_protocol.pdf section 3.1
    # IMPORTANT: All binary operations MUST include comments explaining format and byte order
    data = struct.pack('<f', position)
    
    # Construct CAN message with motor ID as arbitration ID
    # Reference: CAN ID mapping table in docs/damiao_protocol.pdf section 3.2
    message = can.Message(arbitration_id=motor_id, data=data)
    bus.send(message)  # Immediate transmission

# Decoding (asynchronous) 
async def decode_set_position(bus: Bus, motor_id: int) -> dict:
    # Calculate response arbitration ID from motor ID (static offset from docs)
    # Reference: Response ID mapping in docs/damiao_protocol.pdf section 4.1
    response_id = motor_id + 0x100  # Static offset defined in protocol
    
    # Wait for CAN message with calculated arbitration ID (blocking call in thread pool)
    # Set timeout to prevent indefinite blocking
    # Reference: Timeout values in docs/damiao_protocol.pdf section 5.1
    message = bus.recv(response_id, timeout=0.1)
    
    # Unpack binary response data according to protocol format
    # Format: '<f' = little-endian float for position feedback
    # Reference: Response format tables in docs/damiao_protocol.pdf section 4.2
    position = struct.unpack('<f', message.data[:4])[0]
    return {'motor_id': motor_id, 'position': position}

```

### What NOT to Do - Common Mistakes

#### ❌ Wrong Examples

```python
# WRONG: Importing inside functions
def send_request(bus: Bus, motor_id: int, value: float) -> None:
    import struct  # BAD: Import should be at top level
    data = struct.pack('<f', value)

# WRONG: Using helper functions for encoding/decoding
def send_request(bus: Bus, motor_id: int, value: float) -> None:
    message = encode_request(motor_id, value)  # BAD: Assemble can.Message directly
    bus.send(message)

# WRONG: Missing comments on binary operations
data = struct.pack('<f', value)  # BAD: No explanation of format

# WRONG: Using await on blocking bus.recv
async def receive_data(bus: Bus, motor_id: int) -> dict:
    message = await bus.recv(response_id)  # BAD: bus.recv is blocking, not async

# WRONG: Using helper functions for decoding
return decode_message(message)  # BAD: Decode inline with struct.unpack

# WRONG: Nested function definitions
def send_command(bus: Bus, motor_id: int) -> Coroutine:
    async def _execute():  # BAD: Avoid nested functions
        # implementation
    return _execute()

# WRONG: Missing protocol references
response_id = motor_id + 0x100  # BAD: No reference to documentation

# WRONG: No timeout on recv (can block indefinitely)
message = bus.recv(response_id)  # BAD: Missing timeout parameter
```

## Data Structures for Complex Encoding/Decoding

### Simple Data Types
For simple data types (float, int), functions can accept them directly:
```python
# Simple data - accept float directly
def set_position(bus: Bus, motor_id: int, position: float) -> None:
    # Pack position as little-endian 32-bit binary data
    # Reference: Position command format in docs/damiao_protocol.pdf section 3.1
    data = struct.pack('<f', position)
    message = can.Message(arbitration_id=motor_id, data=data)
    bus.send(message)
```

### Complex Data Types - Custom Dataclasses
For complex data structures, functions must handle dataclasses:
```python
from dataclasses import dataclass

@dataclass
class MotorConfig:
    max_velocity: float
    max_torque: float
    pid_kp: float
    pid_ki: float
    pid_kd: float
    enable_limits: bool

# Complex data - function handles dataclass
def configure_motor(bus: Bus, motor_id: int, config: MotorConfig) -> None:
    # Pack complex dataclass into binary format
    # Reference: Configuration packet format in docs/damiao_protocol.pdf section 6.1
    data = struct.pack('<fffff?', 
        config.max_velocity,    # Little-endian float
        config.max_torque,      # Little-endian float  
        config.pid_kp,          # Little-endian float
        config.pid_ki,          # Little-endian float
        config.pid_kd,          # Little-endian float
        config.enable_limits    # Boolean as byte
    )
    
    message = can.Message(arbitration_id=motor_id, data=data)
    bus.send(message)
```

### Decoding Complex Data
For complex response data, return dataclasses:
```python
@dataclass
class MotorStatus:
    position: float
    velocity: float
    torque: float
    temperature: float
    error_code: int

async def get_motor_status(bus: Bus, motor_id: int) -> MotorStatus:
    response_id = motor_id + 0x100
    message = bus.recv(response_id, timeout=0.1)
    
    # Unpack complex response into dataclass
    # Format: position(f) + velocity(f) + torque(f) + temperature(f) + error_code(H)
    # Reference: Status packet format in docs/damiao_protocol.pdf section 6.2
    position, velocity, torque, temperature, error_code = struct.unpack('<ffffH', message.data)
    
    return MotorStatus(
        position=position,
        velocity=velocity, 
        torque=torque,
        temperature=temperature,
        error_code=error_code
    )
```

### ❌ Wrong Data Structure Examples

```python
# WRONG: Using type unions (bad practice)
def send_request(bus: Bus, motor_id: int, data: MotorConfig | float) -> None:
    if isinstance(data, MotorConfig):  # BAD: Avoid union types
        # implementation
    else:
        # implementation

# WRONG: Generic functions with string commands
def send_command(bus: Bus, motor_id: int, command: str, data: Any) -> None:
    # BAD: Too generic, unclear what data types are expected

# WRONG: Not using dataclasses for complex data
def configure_motor(bus: Bus, motor_id: int, max_vel: float, max_torque: float, 
                   pid_kp: float, pid_ki: float, pid_kd: float, limits: bool) -> None:
    # BAD: Too many parameters, should use dataclass
```

## Key Considerations

- **Send Operations**: Direct, blocking calls for immediate message transmission
- **Receive Operations**: Async pattern required for message filtering and queuing
- **Request-Response Pattern**: Functions that send requests and receive responses are not async methods but return coroutines that can be awaited
- **Binary Operations**: Comments must explain struct format strings, byte order, and reference protocol documentation
- **Message Filtering**: Leverage the bus multiplexer's arbitration ID filtering capabilities
- **Error Handling**: Handle timeouts and message validation in the encoding layer

## Usage Examples

### Single Command
```python
response = await set_position(bus, motor_id=1, position=1.5)
```

### Multiple Async Requests (Concurrent)
```python
import asyncio

# Send position commands to multiple motors concurrently and unpack responses
response1, response2, response3 = await asyncio.gather(
    set_position(bus, motor_id=1, position=1.0),
    set_position(bus, motor_id=2, position=2.0),
    set_position(bus, motor_id=3, position=3.0),
)
```

#### ❌ Wrong Usage Examples

```python
# WRONG: Sequential execution (slow)
response1 = await set_position(bus, motor_id=1, position=1.0)
response2 = await set_position(bus, motor_id=2, position=2.0)
response3 = await set_position(bus, motor_id=3, position=3.0)

# WRONG: Using intermediate tasks variable
tasks = [set_position(bus, motor_id=1, position=1.0)]
responses = await asyncio.gather(*tasks)

# WRONG: Forgetting await
response = set_position(bus, motor_id=1, position=1.5)  # Returns coroutine, not result
```

## High-Level Motor Class (`__init__.py`)

The high-level interface uses a Motor class that combines encode/decode functions:

```python
from typing import Coroutine, Any
from openarm.bus import Bus
from .encoding import encode_set_position, decode_set_position

class Motor:
    def __init__(self, bus: Bus, motor_id: int):
        self.bus = bus
        self.motor_id = motor_id
    
    def set_position(self, position: float) -> Coroutine[Any, Any, dict]:
        """Set motor position. Returns coroutine to be awaited."""
        # Encode position and send request
        encode_set_position(self.bus, self.motor_id, position)
        
        # Return coroutine from asynchronous decode function
        return decode_set_position(self.bus, self.motor_id)

# Usage:
motor = Motor(bus, motor_id=1)
response = await motor.set_position(1.5)
```