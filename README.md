# Modbus backend

The modbus backend is based on the NumericAddressedBackend. This means registers are accessed via an address. Currently the NumericAdressedBacked uses uint32\_t words, but modbus is using uint16\_t words. In consequence, when reading a modbus word (16 bit) the result is a 32 bit word, where only the first 16 bits are used.

The mapping is done using the same parser as ChimeraTK's pcie backend. A typical address in the mapping file loos like:

    sspa.RF_slice.1.current_Idc1            2   18       4     2  32  0 0 RO
    
Here 2 elemnets are read starting from address 18. The total resulting length is 4 byte (2 times 16 bits). The bar information (2), width (32), number of fractional bits (0) and signed/unsigned flag (0) is not used in the backend. Finally, the access right (RO) is set. For more details see the [MapFileParser.cpp](https://github.com/ChimeraTK/DeviceAccess/blob/master/fileparsers/src/MapFileParser.cpp).

The device mapping file syntax is as follows:

    test1 modbus:168.1.1.1?type=tcp&map=sigma_phi_FI004250.map&port=502)
    test2 modbus:myserver?type=tcp&map=sigma_phi_FI004250.map&port=502)
    test3 modbus:/dev/ttyUSB0?type=rtu&map=sigma_phi_FI004250.map&parity=N&baud=115200&data_bits=8&stop_bits=1)

As can be seen in the example above two types of modbus communication are supported:
 - type: rtu
 - type: tcp
 
Both offer different additional parameters. If no parameters are given the following defaults are used:

* rtu: 
    parity = N (other allowed values are E or O)
    baud = 115200
    data bits = 8
    stop bits = 1 (other allowed value is 2)
* tcp: 
    port = 502

SDM URI is only supported using default settings as listed above:
    
    test1 sdm://./168.1.1.1:tcp  sigma_phi_FI004250.map)
    

## Remark 

Since e.g. information of a float (2 times 16 bit, thus 2 modbus registers) is put into 2 32 bit words it can not be decoded by e.g. Qthardmon. Currently the decoding has to be done in the application. This can be done as follows:

    union UFloat{
      uint16_t data16[2];
      int32_t  idata;
      float  fdata;
    };
    
    uint32_t modbusDataEncoded[2];
    
    UFloat tmp[2];
    UFloat modbusDataDecoded;
    
    tmp.data32[0] = modbusDataEncoded[0];
    tmp.data32[1] = modbusDataEncoded[1];
    modbusDataDecoded.data16[0] = tmp[0].data16[0];
    modbusDataDecoded.data16[1] = tmp[1].data16[0];
    
    std::cout << "Resulting float is: " << modbusDataDecoded.fdata << std::endl;
    
**All this should be done in the backend itself in the future.**