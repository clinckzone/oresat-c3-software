"""
FM24CL64B F-RAM driver

The FM24CL64B is a 64-Kbit F-RAM (ferroelectric random access memory) with an I2C interface. 
It's logically organised as 8192 memory locations of 8 bytes each and hence needs addresses 
from 0 to 8191 to access each location. Uses 2 bytes are used to send the completed memory address
from 0000000000000000 to 0001111111111111 (x0000 to x1FFF) where first 3 bits are ignored.

Notes on conversion:
(a) 1Kb (1-Kbit) = 2^(10) bits = 1024 bits
(b) 64Kb (64-Kbits)= 64 x 2^10 bits = 64 x 1024 bits, or 
(c) 8KB (8-Kbytes) = 8 x 2^10 bytes = 8 x 1024 bytes
(d) 1Mb (1-Mbit) = 1Kb x 2^10 bits = 1024 x 1024 bits
"""

import os

from smbus2 import SMBus, i2c_msg

class Fm24cl64bError(Exception):
    """Error with `Fm24cl64b`"""

class Fm24cl64b:
    """'FM24CL64B F-RAM driver"""

    # Address format for slave device - 1010[A2][A1][A0][R/W]
    # A2, A1, A0 bits describe the device address whereas R/W bit
    # describes read (1) or write (0) operation.
    ADDR_MIN = 0x50
    ADDR_MAX = 0x5F
    ADDRESSES = list(range(ADDR_MIN, ADDR_MAX + 1))

    SIZE = 8192  # size of F-RAM in bytes

    MOCK_FILE = "/tmp/FM24CL64B.bin"

    def __init__(self, bus_num: int, addr: int, mock: bool = False):
        """
        Parameters
        ----------
        bus: int
            The I2C bus.
        addr: int
            The I2C address, must be between `ADDR_MIN` and `ADDR_MAX`.
        mock: bool:
            Mock the FM24CL64B.
        """

        # Throw an error if the slave address is invalid
        if addr not in self.ADDRESSES:
            raise Fm24cl64bError(
                f"arg addr 0x{addr:X} is not between 0x{self.ADDR_MIN:X} "
                f"and 0x{self.ADDR_MAX:X}"
            )

        self._bus_num = bus_num
        self._addr = addr
        self._mock = mock
        
        # If in mock mode, and there's no file to store
        # mock data, then create one. 
        if mock and not os.path.isfile(self.MOCK_FILE):
            with open(self.MOCK_FILE, "wb") as f:
                f.write(bytearray([0] * self.SIZE))

    def read(self, offset: int, size: int) -> bytes:
        """
        Read a bytes from F-RAM.

        Raises
        ------
        Fm24cl64bError
            The read failed.

        Parameters
        -----------
        offset: int
            The offset from the start of F-RAM to read at.
        size: int
            The number of bytes to read.

        Returns
        -------
        bytes
            The requested bytes.
        """

        # Number of bytes to be read shouldn't be less than 1.
        if size < 1:
            raise Fm24cl64bError("read size must be greater than 1")
        # Address to read from shouldn't be out of bounds of 8192 F-RAM addresses.
        if offset < 0 or offset > self.SIZE:
            raise Fm24cl64bError(f"read offset must be greater than 0 and less than {self.SIZE}")
        # This is actually valid as the read address will wrap around, but it simplifies things.
        if offset + size > self.SIZE:
            raise Fm24cl64bError(f"read offset and size are greater than {self.SIZE}")

        # Convert the offset into 2 bytes using the big-endian format.
        # This I2C memory device expects addresses in big-endian format.
        address = offset.to_bytes(2, "big")

        # If in mock mode, read the entire file, and return the requested bytes.
        if self._mock:
            with open(self.MOCK_FILE, "rb") as f:
                data = bytearray(f.read())
            result = data[offset : offset + size]
        # Else, if reading from an actual device
        else:
            # Performing a selective address read where you
            # first set the internal address you want to read from
            write = i2c_msg.write(self._addr, address)
            
            # then, read 'size' number of bytes starting from that address
            read = i2c_msg.read(self._addr, size)

            try:
                # Open a I2C bus connection and executes the R/W operations in sequence
                with SMBus(self._bus_num) as bus:
                    bus.i2c_rdwr(write, read)
            except OSError:
                raise Fm24cl64bError(f"FM24CL64B at address 0x{self._addr:02X} does not exist")

            # Get the result as a list of i2c_msg struct's of length 'size'
            result = list(read)  # type: ignore

        # Return the result as bytes
        return bytes(result)

    def write(self, offset: int, data: bytes):
        """
        Write bytes to F-RAM.

        Raises
        ------
        Fm24cl64bError
            The write failed.

        Parameters
        -----------
        offset: int
            The offset from the start of F-RAM to write to.
        data: bytes
            The data to write.
        """

        # If data is not an instance of bytes or bytearray, throw an error 
        if not isinstance(data, bytes) and not isinstance(data, bytearray):
            raise Fm24cl64bError(f"write data must be a bytes or bytearray type not {type(data)}")

        # Address to write to shouldn't be out of bounds of 8192 F-RAM addresses.
        if offset < 0 or offset > self.SIZE:
            raise Fm24cl64bError(f"write offset must be greater than 0 and less than {self.SIZE}")
        # THe data that's to be written shouldn't be out of bounds of 8192 F-RAM addresses. 
        if offset + len(data) > self.SIZE:
            raise Fm24cl64bError(f"write offset and data length are greater than {self.SIZE}")
        # There should be some data to write
        if len(data) == 0:
            raise Fm24cl64bError("no data to write")

        size = len(data)
        
        # Convert the offset into a address of 2 bytes in big-endian format.
        # I2C sends data in big-endian format where bits are send from MSB to LSB. 
        address = offset.to_bytes(2, "big")

        # If in mock mode
        if self._mock:
            # Open the file in binary read mode, and read it entirely
            with open(self.MOCK_FILE, "rb") as f:
                tmp = bytearray(f.read())
                
            # Store write the data at the specified location
            tmp[offset : offset + size] = data
            
            #Open the file in binary write mode, and write it entirely
            with open(self.MOCK_FILE, "wb") as f:
                f.write(tmp)
        # Else, if reading from an actual device
        else:
            # Create an I2C write message that will write 'data' (multiple bytes)
            # starting from 'address' to device specified by 'self._addr'  
            write = i2c_msg.write(self._addr, address + data)

            try:
                # Open an I2C bus connection and execute the write operation
                with SMBus(self._bus_num) as bus:
                    bus.i2c_rdwr(write)
            except OSError:
                raise Fm24cl64bError(f"FM24CL64B at address 0x{self._addr:02X} does not exist")

    def clear(self):
        """Clear the bytes in the F-RAM."""

        # Starting from the start of the file, write x00 
        # in half of all the memory locations in F-RAM.
        self.write(0, b"\x00" * (self.SIZE // 2))
