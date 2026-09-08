# UartTrans

Custom UART data transmission format that can be used to transmit commands (executes user-registered callback functions upon receiving commands) or data, using CRC16 verification.

## Protocol Details

| 2Byte  | 1Byte           | 2Byte        | nByte  | 2Byte    | 2Byte  |
| :----- | :-------------- | :----------- | :----- | :------- | :----- |
| Start  | Command or Data | Data Length  | Data   | CRC Check| End    |

* Start: 0xaaff

* Data Type: 1. Command 0. Data

* Data Length: 16bit

* CRC16 Check: (H)

* End: 0xddff(H)

## Quick Start

* Import module

```python
import UartTrans # import 
```

* Create UartTrans

```python
fm.register(22, fm.fpioa.UART1_TX, force=True)
fm.register(21, fm.fpioa.UART1_RX, force=True)
uart1 = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)
uart_t = UartTrans(uart1)
```

* Define custom function and register command

```python
def cus_cmd(uart, s):
    print("execute cus cmd {}".format(s))
    uart.write("execute cus cmd {}".format(s))

uart_t.reg_cmd("cus", cus_cmd, uart1, 'hello') # Register callback function cus_cmd to execute when 'cus' command is received, with parameters uart1 and 'hello'
```

* Send commands or strings

```python
uart_t.write('cus', True) # send 'cus' cmd
uart_t.write('hello') # send 'hello' data
```

* Send numbers (see [pack_num API](#pack_num) for `pack_num` usage)

```python
nums = pack_num(3.1415, 'f') + pack_num(16, 'H') + pack_num(-8, 'b') # pack num data
uart_t.write(nums) # send num 3.1415(float) 16(uint16) -8(int8)
```

* Receive commands or data

```python
udatas = uart_t.read()
```

* Execute received commands, return received data

```python
d = uart_t.parse(udatas)
```

## API: UartTrans

### Register Command

Register a new command and set callback. If the command already exists, it will be overwritten.

```python
reg_cmd(cmd, fun, *args):
```

#### Parameters

* `cmd` Command name, string type
* `fun` Command callback function
* `*args` Callback function parameters

#### Return Value

None

### Unregister Command

```python
unreg_cmd(cmd):
```

#### Parameters

`cmd` Command name, string type.

#### Return Value

None

### Send Command or Data

```python
write(s, is_cmd)
```

#### Parameters

`s` String to send, packed numbers, command (also string), `is_cmd` whether to send as a command

#### Return Value

None

### Pack Numbers<div id="pack_num"></div>

Pack numbers into transmission format

```python
pack_num(n, fl)
```

#### Parameters
 
 * `n` Number
 * `fl` Number type, uint8_t(B), int8_t(b), uint16_t(H), int16_t(h), uint32_t(I), int32_t(i), uint64_t(Q), int64_t(q), float(f), double(d), str(s)

#### Return Value

Packed number

#### Example

Pack and send 3.1415 as float type

```python
pi = uart_t.pack_num(3.1415, 'f')
uart_t.write(pi)
```

### Receive Commands or Data

Read and parse data or commands

```python
read()
```

#### Return Value

List of received data or commands in 2D list format: `[[is_cmd, data]]` is_cmd: whether data is command(1) or data(0), data: the data or command itself.
Returns empty list if no data received

### Parse Data

Parse data received from read(). If it's a command and registered, executes the corresponding callback function immediately. If command doesn't exist, prints reminder message. If it's data, stores it in a list and returns it.

```python
parse(udatas)
```

#### Parameters
`udatas` Data received from read()

#### Return Value

List of parsed data

#### Example

Read and parse data/commands

```python
udatas = uart_t.read()
uart_t.parse(udatas)
```

## Format Example

1. String `aaaaaaaaaaaaaaaaaaa`

> Serial port receives: DD FF 00 00 13 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 61 CE 3A AA FF
> 
> Analysis:
> 
> DD FF: head
> 
> 00: data type
> 
> 00 13: data len
> 
> 61 ... 61: data
> 
> CE 3A: crc16
> 
> AA FF: end
> 
> read() returns:
> 
> ```python
> [(0, b'aaaaaaaaaaaaaaaaaaa')] # type: data
> ```
> 
> parse() returns:
> 
> ```python
> ['aaaaaaaaaaaaaaaaaaa']
> ```

2. Numbers `3.1415, 16, -8` 

> Serial port receives: DD FF 00 00 0A 66 40 49 0E 56 48 00 10 62 F8 4C E0 AA FF 
> 
> Analysis:
> 
> DD FF: head
> 
> 00: data type
> 
> 0A: data len
> 
> 66: 'f' float
> 
> 40 49 0e 56: 3.1415
> 
> 48: 'H' uint16_t
> 
> 00 10: 16
> 
> 62: 'b' int8_t
> 
> F8: -8
> 
> 4C E0: crc16
> 
> AA FF: end
> 
> read() returns:
> 
> ```python
> [(0, b'f@I\x0eVH\x00\x10b\xf8')] # type: data
> ```
> 
> parse() returns:
> 
> ```python
> [[3.1415, 16, -8]]
> ```

3. Command `cus`

> Serial port receives: DD FF 01 00 03 63 75 73 E6 AB AA FF
> 
> Analysis:
> 
> DD FF: head
> 
> 01: data type
> 
> 00 03: data len
> 
> 63 75 73: 'cus'
> 
> E6 AB: crc16
> 
> AA FF: end
> 
> read() returns:
> 
> ```python
> [(1, b'cus')] # type: cmd
> ```
> 
> parse() executes the corresponding callback function immediately.
