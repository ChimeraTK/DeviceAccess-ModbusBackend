# Modbus backend

The modbus backend is based on the NumericAddressedBackend. This means registers are accessed via an address consisting of two parts ("bar" and "address"). Register addresses are specified in a map file (just like for any other NumericAddressedBackend including e.g. the PCIe backend), which looks typically like this:

    #Name             #N_ELEMS    #ADDR  #N_BYTES  #BAR  #WIDTH  #N_FRAC  SIGNED  #ACCESS
    holding.reg1      1           0      2         3     16      0        1       RW
    holding.reg2      1           2      2         3     16      0        0       RW
    holding.reg3      1           4      2         3     16      8        1       RW
    holding.reg32     1           6      4         3     32      0        1       RW
    holding.float     1           10     4         3     32      IEEE754  1       RW
    holding.reg8      1           15     1         3     8       0        1       RW
    holding.array     10          16     20        3     16      0        1       RW
    input.reg1        1           1024   2         4     16      0        1       RO
    coil.bit1         1           0      1         0     1       0        0       RW
    coil.bit2         1           1      1         0     1       0        0       RW
    coil.array        8           2      8         0     1       0        0       RW
    discreteinp.array 10          0      10        1     1       0        0       RO
    discreteinp.bit1  1           10     1         1     1       0        0       RO

For more details about the map file format, please refer to the [DeviceAccess documentation](https://chimeratk.github.io/DeviceAccess/).

The #BAR column represents the Modbus object type:
- 0: Coil / writeable bit
- 1: Discrete input / read-only bit
- 3: Holding register / writeable register
- 4: Input register / read-only register

This number is used as a leading digit of the address in most other Modbus applications, see [Wikipedia](https://en.wikipedia.org/wiki/Modbus#Coil,_discrete_input,_input_register,_holding_register_numbers_and_addresses).

The #ADDR column contains a byte address of the register counting from 0. Compared to the standard Modbus register number (16 bit registers and counting from 1) the difference is a factor of 2 and an offset of 1. Hence, in the above example `holding.reg1` would be address `30001`, while `holding.reg2` corresponds to `30002` etc. For bits (bars 0 and 1), the address from the map file matches the Modbus register number apart from the offest of 1.

Since Modbus supports transferring mulitple registers with a single command, registers with a bigger width than 16 bits are possible (cf. `holding.reg32` in the above example). In case the given byte address is not aligned with the Modbus 16 bit register width (cf. `holding.reg8`), a less-efficient unaligned transfer needs to be done, which involves read-modify-write in case of write operations.

Reads/writes of consecutive registers can be merged into single multi-register accesses. To disable this (e.g. for broken Modbus servers), the parameter `disableMerging` can be set.

The fixed-point resp. floating-point conversions specified in the map file are executed just like for any other NumericAddressedBackend. If no conversion is required for a standard 16-bit Modbus register, specify the "#WIDTH  #N_FRAC  SIGNED" columns as "16 0 1" for signed resp. "16 0 0" for unsigned registers. The specified bit width must not exceed the bit width of the register, i.e. if #N_BYTES is e.g. 2, #WIDTH must be <= 16.

The CDD (ChimeraTK device descriptor) syntax (as used e.g. in the DMAP file) is as follows:

* TCP mode: `(modbus:myserver?type=tcp&map=device.map&port=502)`
* RTU over TCP mode: `(modbus:myserver?type=tcp&map=device.map&port=502&slaveid=2)`
* RTU mode: `(modbus:/dev/ttyUSB0?type=rtu&map=device.map&parity=N&baud=115200&data_bits=8&stop_bits=1)`

As can be seen in the example above two types of modbus communication are supported:
 - type: rtu
 - type: tcp
 
Both offer different additional parameters. If no parameters are given the following defaults are used:

* rtu: 
    parity = N (other allowed values are E or O)
    baud = 115200
    data bits = 8
    stop bits = 1 (other allowed value is 2)
    slaveid = 1
    disableMerging = 0
* tcp: 
    port = 502
    slaveid = 255
    disableMerging = 0
