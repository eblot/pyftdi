
Supported SPI flash devices
===========================

============= ============= ========== ======== ====== ======= ========== ==========
              ATMEL         ATMEL      Macronix SST    Winbond EON        Numonix
              AT45          AT25       MX25D/E  SST25  W25Q    EN25Q      M25P
============= ============= ========== ======== ====== ======= ========== ==========
Status        Not supported Not tested Tested   Tested Tested  Not tested Not tested
------------- ------------- ---------- -------- ------ ------- ---------- ----------
Sizes (MiB)             2,4      2,4,8 2,4,8,16    2,4     2,4
------------- ------------- ---------- -------- ------ ------- ---------- ----------
Read (KiB/s)                               1255    642    1252
------------- ------------- ---------- -------- ------ ------- ---------- ----------
Write (KiB/s)                                63      2      63
------------- ------------- ---------- -------- ------ ------- ---------- ----------
Erase (KiB/s)                                31    500      60
============= ============= ========== ======== ====== ======= ========== ==========

Notes
~~~~~
 * Read operation is synchronous with SPI bus clock: it therefore only depends on
   the achievable frequency on the SPI bus, which is bound to the highest supported
   frequency of the flash device.
 * Write operation depends mostly on the flash device performance, whose upper
   limit comes mostly from the maximum write packet size of the device, as the device
   needs to be polled for completion after each packet: the shorter the packet,
   the higher traffic on the SPI and associated overhead.
 * Erase operation depends mostly on the flash device performance, whose fully 
   depends on the flash device internal technology, as very few and short packets
   are exchanged over the SPI bus.
