Search.setIndex({docnames:["api/eeprom","api/ftdi","api/gpio","api/i2c","api/index","api/misc","api/spi","api/uart","api/usbtools","authors","defs","eeprom","features","gpio","index","installation","license","pinout","requirements","testing","tools","troubleshooting","urlscheme"],envversion:{"sphinx.domains.c":2,"sphinx.domains.changeset":1,"sphinx.domains.citation":1,"sphinx.domains.cpp":3,"sphinx.domains.index":1,"sphinx.domains.javascript":2,"sphinx.domains.math":2,"sphinx.domains.python":2,"sphinx.domains.rst":2,"sphinx.domains.std":1,sphinx:56},filenames:["api/eeprom.rst","api/ftdi.rst","api/gpio.rst","api/i2c.rst","api/index.rst","api/misc.rst","api/spi.rst","api/uart.rst","api/usbtools.rst","authors.rst","defs.rst","eeprom.rst","features.rst","gpio.rst","index.rst","installation.rst","license.rst","pinout.rst","requirements.rst","testing.rst","tools.rst","troubleshooting.rst","urlscheme.rst"],objects:{"pyftdi.eeprom":{FtdiEeprom:[0,1,1,""],FtdiEepromError:[0,4,1,""]},"pyftdi.eeprom.FtdiEeprom":{CBUS:[0,2,1,""],CBUSH:[0,2,1,""],CBUSX:[0,2,1,""],CFG1:[0,2,1,""],CHANNEL:[0,2,1,""],DRIVE:[0,2,1,""],UART_BITS:[0,2,1,""],VAR_STRINGS:[0,2,1,""],cbus_mask:[0,3,1,""],cbus_pins:[0,3,1,""],close:[0,3,1,""],commit:[0,3,1,""],connect:[0,3,1,""],data:[0,3,1,""],device_version:[0,3,1,""],dump_config:[0,3,1,""],erase:[0,3,1,""],initialize:[0,3,1,""],is_empty:[0,3,1,""],load_config:[0,3,1,""],open:[0,3,1,""],properties:[0,3,1,""],reset_device:[0,3,1,""],save_config:[0,3,1,""],set_manufacturer_name:[0,3,1,""],set_product_name:[0,3,1,""],set_property:[0,3,1,""],set_serial_number:[0,3,1,""],size:[0,3,1,""],sync:[0,3,1,""]},"pyftdi.ftdi":{Ftdi:[1,1,1,""],FtdiError:[1,4,1,""],FtdiFeatureError:[1,4,1,""],FtdiMpsseError:[1,4,1,""]},"pyftdi.ftdi.Ftdi":{BitMode:[1,1,1,""],DEFAULT_VENDOR:[1,2,1,""],DEVICE_NAMES:[1,2,1,""],FIFO_SIZES:[1,2,1,""],FTDI_VENDOR:[1,2,1,""],PRODUCT_IDS:[1,2,1,""],SCHEME:[1,2,1,""],VENDOR_IDS:[1,2,1,""],add_custom_product:[1,3,1,""],add_custom_vendor:[1,3,1,""],baudrate:[1,3,1,""],bitbang_enabled:[1,3,1,""],calc_eeprom_checksum:[1,3,1,""],close:[1,3,1,""],create_from_url:[1,3,1,""],decode_modem_status:[1,3,1,""],device_port_count:[1,3,1,""],device_version:[1,3,1,""],enable_3phase_clock:[1,3,1,""],enable_adaptive_clock:[1,3,1,""],enable_drivezero_mode:[1,3,1,""],enable_loopback_mode:[1,3,1,""],fifo_sizes:[1,3,1,""],find_all:[1,3,1,""],frequency_max:[1,3,1,""],get_cbus_gpio:[1,3,1,""],get_cd:[1,3,1,""],get_cts:[1,3,1,""],get_device:[1,3,1,""],get_dsr:[1,3,1,""],get_error_string:[1,3,1,""],get_identifiers:[1,3,1,""],get_latency_timer:[1,3,1,""],get_ri:[1,3,1,""],has_cbus:[1,3,1,""],has_drivezero:[1,3,1,""],has_mpsse:[1,3,1,""],has_wide_port:[1,3,1,""],ic_name:[1,3,1,""],is_H_series:[1,3,1,""],is_bitbang_enabled:[1,3,1,""],is_connected:[1,3,1,""],is_legacy:[1,3,1,""],is_mpsse:[1,3,1,""],is_mpsse_interface:[1,3,1,""],list_devices:[1,3,1,""],modem_status:[1,3,1,""],mpsse_bit_delay:[1,3,1,""],open:[1,3,1,""],open_bitbang:[1,3,1,""],open_bitbang_from_device:[1,3,1,""],open_bitbang_from_url:[1,3,1,""],open_from_device:[1,3,1,""],open_from_url:[1,3,1,""],open_mpsse:[1,3,1,""],open_mpsse_from_device:[1,3,1,""],open_mpsse_from_url:[1,3,1,""],overwrite_eeprom:[1,3,1,""],poll_modem_status:[1,3,1,""],port_index:[1,3,1,""],port_width:[1,3,1,""],purge_buffers:[1,3,1,""],purge_rx_buffer:[1,3,1,""],purge_tx_buffer:[1,3,1,""],read_data:[1,3,1,""],read_data_bytes:[1,3,1,""],read_data_get_chunksize:[1,3,1,""],read_data_set_chunksize:[1,3,1,""],read_eeprom:[1,3,1,""],read_pins:[1,3,1,""],reset:[1,3,1,""],set_baudrate:[1,3,1,""],set_bitmode:[1,3,1,""],set_break:[1,3,1,""],set_cbus_direction:[1,3,1,""],set_cbus_gpio:[1,3,1,""],set_dtr:[1,3,1,""],set_dtr_rts:[1,3,1,""],set_dynamic_latency:[1,3,1,""],set_error_char:[1,3,1,""],set_event_char:[1,3,1,""],set_flowctrl:[1,3,1,""],set_frequency:[1,3,1,""],set_latency_timer:[1,3,1,""],set_line_property:[1,3,1,""],set_rts:[1,3,1,""],show_devices:[1,3,1,""],usb_dev:[1,3,1,""],usb_path:[1,3,1,""],validate_mpsse:[1,3,1,""],write_data:[1,3,1,""],write_data_get_chunksize:[1,3,1,""],write_data_set_chunksize:[1,3,1,""],write_eeprom:[1,3,1,""]},"pyftdi.gpio":{GpioAsyncController:[2,1,1,""],GpioException:[2,4,1,""],GpioMpsseController:[2,1,1,""],GpioPort:[2,1,1,""],GpioSyncController:[2,1,1,""]},"pyftdi.gpio.GpioAsyncController":{open_from_url:[2,3,1,""],read:[2,3,1,""],read_port:[2,3,1,""],set_frequency:[2,3,1,""],write:[2,3,1,""],write_port:[2,3,1,""]},"pyftdi.gpio.GpioMpsseController":{read:[2,3,1,""],set_frequency:[2,3,1,""],write:[2,3,1,""]},"pyftdi.gpio.GpioSyncController":{exchange:[2,3,1,""],set_frequency:[2,3,1,""]},"pyftdi.i2c":{I2cController:[3,1,1,""],I2cGpioPort:[3,1,1,""],I2cIOError:[3,4,1,""],I2cNackError:[3,4,1,""],I2cPort:[3,1,1,""],I2cTimeoutError:[3,4,1,""]},"pyftdi.i2c.I2cController":{configure:[3,3,1,""],configured:[3,3,1,""],direction:[3,3,1,""],exchange:[3,3,1,""],flush:[3,3,1,""],force_clock_mode:[3,3,1,""],frequency:[3,3,1,""],frequency_max:[3,3,1,""],ftdi:[3,3,1,""],get_gpio:[3,3,1,""],get_port:[3,3,1,""],gpio_all_pins:[3,3,1,""],gpio_pins:[3,3,1,""],poll:[3,3,1,""],poll_cond:[3,3,1,""],read:[3,3,1,""],read_gpio:[3,3,1,""],set_gpio_direction:[3,3,1,""],set_retry_count:[3,3,1,""],terminate:[3,3,1,""],validate_address:[3,3,1,""],width:[3,3,1,""],write:[3,3,1,""],write_gpio:[3,3,1,""]},"pyftdi.i2c.I2cGpioPort":{all_pins:[3,3,1,""],direction:[3,3,1,""],pins:[3,3,1,""],read:[3,3,1,""],set_direction:[3,3,1,""],width:[3,3,1,""],write:[3,3,1,""]},"pyftdi.i2c.I2cPort":{address:[3,3,1,""],configure_register:[3,3,1,""],exchange:[3,3,1,""],flush:[3,3,1,""],frequency:[3,3,1,""],poll:[3,3,1,""],poll_cond:[3,3,1,""],read:[3,3,1,""],read_from:[3,3,1,""],shift_address:[3,3,1,""],write:[3,3,1,""],write_to:[3,3,1,""]},"pyftdi.misc":{EasyDict:[5,1,1,""],add_custom_devices:[5,5,1,""],hexdump:[5,5,1,""],hexline:[5,5,1,""],is_iterable:[5,5,1,""],pretty_size:[5,5,1,""],show_call_stack:[5,5,1,""],to_bool:[5,5,1,""],to_bps:[5,5,1,""],to_int:[5,5,1,""],xor:[5,5,1,""]},"pyftdi.misc.EasyDict":{copy:[5,3,1,""],mirror:[5,3,1,""]},"pyftdi.spi":{SpiController:[6,1,1,""],SpiGpioPort:[6,1,1,""],SpiIOError:[6,4,1,""],SpiPort:[6,1,1,""]},"pyftdi.spi.SpiController":{active_channels:[6,3,1,""],channels:[6,3,1,""],configure:[6,3,1,""],configured:[6,3,1,""],direction:[6,3,1,""],exchange:[6,3,1,""],flush:[6,3,1,""],force_control:[6,3,1,""],frequency:[6,3,1,""],frequency_max:[6,3,1,""],ftdi:[6,3,1,""],get_gpio:[6,3,1,""],get_port:[6,3,1,""],gpio_all_pins:[6,3,1,""],gpio_pins:[6,3,1,""],is_inverted_cpha_supported:[6,3,1,""],read_gpio:[6,3,1,""],set_gpio_direction:[6,3,1,""],terminate:[6,3,1,""],width:[6,3,1,""],write_gpio:[6,3,1,""]},"pyftdi.spi.SpiGpioPort":{all_pins:[6,3,1,""],direction:[6,3,1,""],pins:[6,3,1,""],read:[6,3,1,""],set_direction:[6,3,1,""],width:[6,3,1,""],write:[6,3,1,""]},"pyftdi.spi.SpiPort":{cs:[6,3,1,""],exchange:[6,3,1,""],flush:[6,3,1,""],force_select:[6,3,1,""],frequency:[6,3,1,""],mode:[6,3,1,""],read:[6,3,1,""],set_frequency:[6,3,1,""],set_mode:[6,3,1,""],write:[6,3,1,""]},"pyftdi.usbtools":{UsbTools:[8,1,1,""],UsbToolsError:[8,4,1,""]},"pyftdi.usbtools.UsbTools":{build_dev_strings:[8,3,1,""],enumerate_candidates:[8,3,1,""],find_all:[8,3,1,""],find_backend:[8,3,1,""],flush_cache:[8,3,1,""],get_device:[8,3,1,""],get_string:[8,3,1,""],list_devices:[8,3,1,""],parse_url:[8,3,1,""],release_all_devices:[8,3,1,""],release_device:[8,3,1,""],show_devices:[8,3,1,""]},pyftdi:{eeprom:[0,0,0,"-"],ftdi:[1,0,0,"-"],gpio:[2,0,0,"-"],i2c:[3,0,0,"-"],misc:[5,0,0,"-"],spi:[6,0,0,"-"],usbtools:[8,0,0,"-"]}},objnames:{"0":["py","module","Python module"],"1":["py","class","Python class"],"2":["py","attribute","Python attribute"],"3":["py","method","Python method"],"4":["py","exception","Python exception"],"5":["py","function","Python function"]},objtypes:{"0":"py:module","1":"py:class","2":"py:attribute","3":"py:method","4":"py:exception","5":"py:function"},terms:{"001":3,"002":7,"0110":13,"0111":13,"018":7,"036":7,"040":7,"0403":15,"052":7,"056":7,"0664":15,"0b0001":13,"0b1001":13,"0x00":[3,6,13],"0x01":[0,13],"0x02":13,"0x04":3,"0x06":[3,13],"0x07":13,"0x08":3,"0x0f":13,"0x1":13,"0x10":6,"0x12":[3,6],"0x1234":15,"0x20":6,"0x21":3,"0x30":6,"0x34":[3,6],"0x403":[5,20,22],"0x53":3,"0x56":3,"0x5678":15,"0x6001":22,"0x6010":22,"0x6011":22,"0x6014":22,"0x6015":22,"0x666":[5,20],"0x76":13,"0x78":13,"0x7f":3,"0x80":13,"0x84":13,"0x9999":[5,20],"0x9f":6,"0xcafe":[5,20],"0xff":13,"1000000":[1,6],"1024":[1,5,7],"1027":1,"10485760":5,"108":7,"10e6":6,"10mhz":6,"114":6,"115":7,"115200":7,"123456":0,"128":1,"1280":1,"12e6":6,"12mbp":14,"12mhz":6,"1536":1,"1792":1,"1e6":13,"1mbp":7,"1ms":3,"2008":16,"2020":[16,18],"2048":1,"2232":[1,7,15,22],"2232c":1,"2232d":[1,22],"2232h":[0,1,3,6,7,22],"2304":1,"230x":[1,22],"231x":1,"232":[1,7,12,22],"232h":[1,3,6,15,22],"232r":[1,22],"234x":1,"24577":1,"24592":1,"24593":1,"24596":1,"24597":1,"255":1,"256":1,"2b8717a3647cc650625c566259e00305f7fb60aa":15,"3000000":7,"384":1,"3mbaud":7,"3mbp":14,"4096":1,"4232":[1,22],"4232h":[1,6,22],"4432h":[3,6],"460":7,"461":7,"512":1,"5th":18,"6000000":1,"6001":15,"6010":15,"6011":15,"6014":15,"6015":15,"667":7,"727":7,"857":7,"964":7,"boolean":[3,5,6],"break":[1,8,13,14],"byte":[0,1,2,3,5,6,7,12,13,14,21],"case":[2,3,6,13,19,22],"char":[5,7],"cl\u00e9ment":9,"class":5,"default":[0,1,3,5,6,7,8,11,14,19],"export":[0,3,6,21],"final":[5,21],"float":[1,2,3,5,6],"function":[1,7,13],"import":[0,3,7,13,15],"int":[0,1,2,3,5,6,8],"long":[2,3,16],"new":[0,1,2,5,6,8,11,13,14,18,19,21],"public":[14,16],"return":[0,1,2,3,5,6,8,14],"short":[1,2,5,13,20],"static":1,"switch":[1,5],"true":[0,1,3,5,6,7,13],"try":[8,11,15,21],"while":[0,1,3,6,7,12,14,15],AND:16,ARE:16,BUS:13,BUT:16,CTS:[1,7,17],FOR:16,For:[12,13,16,19],IDs:22,IOs:[3,6],Its:[1,15],NOT:16,Not:1,OSes:15,RTS:[1,7,17],SUCH:16,THE:16,TMS:17,That:13,The:[0,1,2,3,5,6,7,8,11,12,13,14,15,19,20,22],Then:7,There:[1,3,6,7,8,13,15,19,22],These:[3,13,20],USE:16,Use:[1,3,6,11],Useful:1,Will:9,With:[3,13,15,19,22],Yes:7,_a_:5,_b_:5,_from_url:22,abbrevi:5,abcd1234:11,abl:[1,3,19],abort:2,about:[7,8,13,14,15],abov:[1,5,7,14,15,16,18],acbu:[13,17],acbus0:17,acbus1:17,acbus2:17,acbus3:17,acbus4:17,acbus5:17,acbus6:17,acbus7:17,accept:[1,2,3,5,6,7,13,20,22],access:[0,1,2,5,6,11,12,15,19],achiev:[1,3,5,6,7,13],ack:3,acknowledg:3,act:7,action:11,activ:[1,6,11,13],active_channel:6,actual:[0,1,2,3,6,7,11,13,15,19],ad0:[3,6,13],ad15:13,ad1:[3,6,13],ad2:[3,6,13],ad3:[6,13],ad4:[6,13],ad5:6,ad6:6,ad7:[3,13],adam:9,adapt:[1,3,12],adbu:[13,17],adbus0:17,adbus1:17,adbus2:17,adbus3:17,adbus4:17,adbus5:17,adbus6:17,adbus7:17,adc:6,add:[1,2,3,5,6,15,20],add_custom_devic:5,add_custom_product:[1,15],add_custom_vendor:[1,15],added:3,addess:22,addit:[7,11,13],addition:13,addr:[1,8],address:[1,3,6,7,12,13,14,22],addus:15,advanc:2,advis:16,adxl345:[3,6],affect:[1,13],after:[0,6,7,13],again:[6,11],against:[3,14],ahead:19,aim:[13,14,19],alexforencich:9,alia:[0,5,20,21],alias:[5,20,22],align:[6,12],all:[0,1,2,3,6,7,8,11,12,13,14,15,16,19,22],all_pin:[3,6],allow:[1,5,15],allow_int:5,along:[3,11,13,18,20],alreadi:[1,19,21],also:[1,3,6,12,13,14,15,19,20,21,22],alter:11,altern:[0,3,6,7,11,14,15,20],although:[7,14,18,19],alwai:[1,2,3,6,8,13,14,15],amanita:9,among:22,amount:1,analog:13,ander:9,andrea:9,ani:[0,1,2,3,5,6,7,8,11,12,13,15,16,18,21],annot:18,anoth:[6,7,13,19,21,22],anymor:[8,16],api:[1,8,11,12,14,15,16,19,20,22],appear:[8,15],append:5,appl:21,appleusbftdi:21,appli:[1,3,6,13],applic:[6,7,11,13],appropri:[3,6,11],approxim:2,apt:15,arbitrari:[1,2,6,7,15],arbitrarili:13,archiv:7,argument:[0,1,3,5,6,7,8,11,13,20],aris:16,arm:[1,3],arrai:[1,3,6,14],ascii:[5,11],ask:22,assert:[3,6,13],assign:[7,11,13],associ:[3,11],async:19,asynchron:[1,2,13,14],atom:[2,7,13],attach:3,attemp:1,attempt:[1,3,11,21],attr:15,attribut:[1,7],augment:14,author:[14,16,21],automat:[1,12,13,15,19,21],avaialbl:22,avail:[1,2,3,6,7,11,12,13,14,15,17,20,22],avoid:[11,19],awak:11,awar:[1,7],b15:2,back:[1,3,7,8,11,15],backend:[7,8,12,15,19],backward:14,bail:3,balanc:1,bandwidth:1,bang:[2,11],bare:7,base:[1,2,12],basi:[1,7,18],basic:14,bat_detect:11,bat_ndetect:11,batteri:11,baudrat:[1,5,7,14,19],baudrate_toler:1,bb_rd:11,bb_wr:11,bcbu:[13,17],bcbus0:17,bcbus1:17,bcbus2:17,bcbus3:17,bcbus4:17,bcbus5:17,bcbus6:17,bcbus7:17,bd0:[3,13],bd15:13,bd1:13,bd2:13,bd3:13,bd4:13,bd6:13,bd7:[3,13],bdbu:13,bdbus0:17,bdbus1:17,bdbus2:17,bdbus3:17,bdbus4:17,bdbus5:17,bdbus6:17,bdbus7:17,becaus:[1,19],becom:13,bee:18,been:[0,1,2,3,6,8,11,13,15,16,19,20,21],befor:[1,2,3,6,11,13,15,21],behav:5,being:[1,2,14],below:[7,15,20],bench:19,benureau:9,best:[1,22],beta:12,better:[3,13],between:[1,5,7,13],bewar:[1,2,14],bia:[3,6],bidirect:3,big:3,bigendian:3,bin:[7,11,12,20,21],binari:[3,5,16],binaryio:0,bit:[0,1,2,3,5,6,7,11,12,13,14,15,17,18,20],bitbang:[1,2,13,14,17],bitbang_en:1,bitfield:[1,2,3,6],bitmap:0,bitmask:[0,1],bitmod:1,bitrat:3,blank:11,block:[1,6,11],blot:[9,16],blue:17,board:[2,11,19],bool:[0,1,2,3,5,6,8],both:[2,3,12,13],bouaziz:9,boundari:12,bps:[1,7],branch:15,break_:1,brew:15,brick:[1,11],bridg:[1,14],bring:14,brown:17,buffer:[1,2,3,5,6,7,11,13],bug:[3,19],build:[8,14,15],build_dev_str:8,bundl:21,bus0:3,bus1:3,bus2:3,bus3:6,bus4:6,bus7:3,bus:[1,6,8,12,13,14,17,19,20,22],buse:[1,22],busi:[3,16],byffer:1,bypass:[1,3,8],bytearrai:[1,2,3,5,6,14],c232hd:[15,17],cabl:17,cach:[1,8],calc_eeprom_checksum:1,calcul:[1,14],call:[1,3,5,6,7,8,13,14,15,19],callabl:1,caller:13,can:[0,1,2,3,5,6,7,8,11,12,13,14,15,19,20,21,22],candid:[5,8],cannot:[1,2,3,5,7,12,13,14,19,21],capabl:[1,3,7,13,14,17],captur:13,care:[1,6,11],carefulli:3,carrier:1,caus:[11,16,21],caution:11,caveat:12,cbu:[0,1,2,14],cbus0:[11,13],cbus1:13,cbus2:13,cbus3:13,cbus5:13,cbus6:13,cbus8:13,cbus9:13,cbus_func_0:11,cbus_func_1:11,cbus_func_2:11,cbus_func_3:11,cbus_func_4:11,cbus_func_5:11,cbus_func_6:11,cbus_func_7:11,cbus_func_8:11,cbus_func_9:11,cbus_mask:[0,13],cbus_pin:[0,13],cbush:0,cbusx:0,cdbu:13,cfg1:0,challeng:19,chang:[0,1,2,3,6,7,11,13,15],channel:[0,6,15],channel_a_driv:11,channel_a_typ:11,chapter:[7,11,12,15],charact:[1,5,7,11],characterist:1,charg:19,charger:11,check:[1,2,3,21],checkout:[3,6],checksum:[1,11],chip:[1,6,11],chunk:1,chunksiz:1,circumv:1,classmethod:[1,3,5,8],clear:[1,8,13],click:15,client:[2,7,14],clk12:11,clk15:11,clk24:11,clk30:11,clk48:11,clk6:11,clk7_5:11,clock:[1,2,6,7,11,12,13,14,17],clock_polar:11,clockstrech:3,clockstretch:3,clone:15,close:[0,1,3,6,7,13,16],closest:[1,2,5],cmd:15,code:[5,9,15,16,18,21],collect:2,collector:[1,13],column:11,com:[1,15,21],come:[7,20,21],command:[1,2,3,6,7,11,13,15,19,20],commerci:16,commit:0,common:[1,5,11,12,13,19],commun:[1,3,6,12,13,15,19,21],compact:11,companion:[11,12,20],compat:[1,14,16,22],complet:[2,5,6,15,20],complex:3,compli:16,compliant:[3,12],composit:15,comprehens:19,comput:1,concern:19,concil:9,condit:[3,11,16],confifur:19,config:11,configur:[0,1,2,3,6,15,17,20,21],configure_regist:3,conflict:[3,21],connect:[0,1,3,6,7,8,12,13,15,17,20],consequ:6,consequenti:16,consid:[6,13],constant:[1,2,11],constrain:1,constraint:8,constructor:6,contact:21,contain:[0,1,6,11,12,13,18,19,21],content:[0,1,11,12,20],contigu:13,contin:2,continu:[1,19],contract:16,contrari:2,contributor:16,control:[1,2,3,6,13,15],convers:5,convert:[5,14],cope:[3,6],copi:[3,5,6,11,13,15],copyright:16,core:1,corrupt:[11,22],could:20,count:[1,3,6,7,8],coupl:[19,20],cowardli:3,cpha:[6,12],cpol:[6,12],cpu:1,creat:[1,15,19],create_from_url:[1,15],criteria:8,crlf:7,cs0:[6,17],cs1:[6,17],cs2:17,cs3:17,cs4:17,cs_count:6,cs_epilog:6,cs_hold:6,cs_prolog:6,ctrl:[3,6,7],cts:7,current:[0,1,2,3,5,6,7,11,13,14,18,21],custom:[0,1,5,7,11,20],cycl:[0,1,3,6,7,13],d2xx:21,dac:6,daemon:15,damag:16,darren:9,data:[0,1,2,3,5,6,7,11,13,16,21],datasheet:[1,13],dave:9,davidwc:9,dbu:13,dcd:[7,17],dce:7,ddbu:13,ddhsp:15,deal:[8,15],debug:[1,3,5,6,7,11,12,20,21],decid:13,decim:[5,20,22],declar:15,decod:[1,11,19],decode_modem_statu:1,decoded:1,decreas:1,dedic:[1,2,3,6,7,13,19],default_vendor:[1,8,15],defin:[0,1,2,3,6,7,11,13,19,22],definit:[1,5,19],delai:[1,2,6,13],demonstr:[12,19],denot:13,depend:[0,1,3,6,7,11,13,14,18,20,22],depict:13,deprec:22,depth:13,deriv:[6,16],desactivet:6,describ:[5,11,20],descript:[1,11],descriptor:[1,8],deserv:13,design:[1,2,3,6,20],desir:1,detail:[2,3,6,7,13,15,22],detect:[0,1,7,11,15,19,20],determin:6,devclass:8,devdesc:8,develop:[11,12,14,15,16,20,21],deviat:1,devic:[0,1,2,3,5,6,7,8,11,13,14,15,19,20],device_nam:1,device_port_count:1,device_vers:[0,1],dict:8,dictionari:5,did:13,differ:[1,3,8,11,13],difficult:7,difficulti:15,diod:3,direct:[1,2,3,6,7,16,17],directli:[1,3,6,13],directori:[7,12,19,20,21],disabl:[1,2,6,19],discard:[2,3,12],disclaim:16,disconnect:[8,11],discov:[12,20],discrimin:[1,8],dispatch:12,distinct:13,distribut:[15,16,19],divid:[2,14],dividor:1,dlharmon:9,doc:15,document:[1,7,13,16,20],doe:[0,1,2,6,7,13,16,17,18,21],done:[1,7],doubl:1,down:[3,11],download:20,drain:[3,13],drift:1,drive0:11,drive1:11,drive:[0,1,2,3,6,11,13],driven:[1,7],driver:[0,4,12,14,15,21,22],drop:3,droptail:6,dry:11,dry_run:[0,1],dsr:[1,7,17],dtr:[1,7,17],dual:[14,15],duck:[2,13],due:[3,6],dump:[0,5,11],dump_config:0,duplex:[6,12],durat:[6,7],dure:11,duti:[6,7],dyld_library_path:21,dylib:21,dynam:[1,19,21],each:[1,2,3,6,7,8,11,13,15,19],earli:[19,21],eas:19,easi:6,easiest:15,easydict:5,eblot:15,ebouaziz:9,echo:7,edit:19,eeprom:[1,3,4,14,15,20,22],eeprom_s:1,effect:[1,3,11,15],effort:19,egg:15,either:[1,8,13,17,20,22],electr:2,email:21,emit:[3,11,20,21],emmanuel:[9,16],empti:[1,2,3,6,11],emul:[12,19],enabl:[1,3,5,6,7,11,12,13,19,20],enable_3phase_clock:1,enable_adaptive_clock:1,enable_drivezero_mod:1,enable_loopback_mod:1,encod:[3,11],end:[1,2,8,15,18,22],endian:3,endlesscoil:9,endors:16,endpoint:1,enforc:[3,6],engin:[1,3,6,19],enhanc:21,enough:22,ensur:[2,21],entri:15,enumer:[1,8,19,21,22],enumerate_candid:8,environ:[19,20],epilog:6,equal:11,equip:19,equival:13,eras:[0,11,22],error:[0,1,2,3,6,8],error_onli:1,errorch:1,essenti:11,etc:[6,13,15],etherfi:9,evalu:[0,1],eveeri:2,even:[1,11,13,16,19],event:[1,3,16],eventch:1,everi:[1,13],evolv:19,exact:[1,2,5,6,13],exampl:[0,1,2,3,6,7,15,18,20,21,22],except:[7,19,20],exchang:[1,2,3,6,13],exclus:2,execut:[0,1,6,11,22],exemplari:16,exhibit:[3,7],exist:[0,1,13,14,21,22],exit:[7,11,20,22],expand:3,expect:[1,3,6],experi:21,experiment:[12,13,14,19],explain:13,expos:[11,17],express:[5,16,20],extend:[6,7],extens:[7,12,21],extern:[1,7,11],extra:[6,13],extract:1,extrem:[6,11],fabien:9,fail:[3,5,8,11],fake:3,fall:11,fallback:5,fals:[0,1,2,3,5,6,8],famili:20,faq:1,far:3,faster:3,fatal:21,featur:[0,1,2,3,6,7,17,19,20],feedback:1,feuer:9,few:[1,6],fifo:[1,2,3,6],fifo_s:1,fig:[3,6],file:[0,11,13,15,16,19,21],fill:2,filter:22,find:[1,8,22],find_al:[1,8],find_backend:8,fine:19,first:[1,3,5,6,7,11,13,17],fit:[0,3,16,19,22],fix:[11,15],flag:[0,1,11],flash:[6,12],floor:5,flow:1,flow_control:11,flowcontrol:1,flowctrl:1,flush:[2,3,6,8],flush_cach:8,fluter:7,fmt:3,follow:[1,3,5,6,7,11,13,14,15,16,18,20],forc:[0,1,3,6,13,22],force_clock_mod:3,force_control:6,force_select:6,form:16,format:[3,5,20],former:15,fortun:3,forward:3,found:19,four:[0,13],framework:[13,20],free:[9,13,14,16],freebsd:14,freeli:[3,6,16],freq:[2,6],frequenc:[1,2,3,6,13,19],frequency_max:[1,3,6],fresh:[1,3,6,15],from:[0,1,2,3,6,7,8,11,12,13,14,16,19,20,21,22],fstring:[18,19],ft0fmf6v:22,ft1pwz0q:15,ft2232:[1,15],ft2232c:[1,14,15],ft2232d:[1,3,13,15],ft2232h:[0,1,3,6,7,8,12,13,14,15,17],ft230x:[0,1,2,7,13,14,15,19],ft231x:[1,2,7,13,14,15],ft232:1,ft232am:[1,15],ft232bm:[1,15],ft232h:[0,1,2,3,7,11,12,14,15,17,19,22],ft232r:[0,1,2,7,11,13,14,15],ft234x:[1,14,15],ft4222h:3,ft4232:[1,15],ft4232h:[0,1,3,8,12,13,14,15,17],ft_prog:11,ftconf:[11,12],ftdi2232c:17,ftdi:[0,2,3,4,5,6,7,8,11,12,13,14,15,16,19,20,21,22],ftdi_devic:[0,3,6],ftdi_loglevel:[3,6,19],ftdi_recoveri:11,ftdi_url:[7,22],ftdi_vendor:1,ftdichip:1,ftdicl:5,ftdieeprom:[0,13],ftdieepromerror:0,ftdierror:1,ftdifeatureerror:1,ftdimpsseerror:1,ftdivirt:19,ftdu_url:15,ftmani:19,ftxxx:[1,7],full:[0,1,2,5,6,7,11,12,13],fulli:[2,3,13],fullmod:7,further:[3,6],garnier:9,gener:[2,5,6,8,11,12,16,19],get:[1,2,3,6,11,13,15,19],get_cbus_gpio:[1,13],get_cd:1,get_ct:1,get_devic:[1,8],get_dsr:1,get_error_str:1,get_gpio:[3,6,13],get_identifi:1,get_latency_tim:1,get_port:[3,6],get_ri:1,get_str:8,git:[15,19],github:[3,6,13,14,15,21],give:[1,13],given:13,gnd:13,gnu:16,goe:11,good:[7,16],gpi:6,gpio0:17,gpio10:17,gpio11:17,gpio12:17,gpio13:17,gpio14:17,gpio15:17,gpio1:17,gpio2:17,gpio3:17,gpio4:17,gpio5:17,gpio6:17,gpio7:[3,17],gpio8:17,gpio9:17,gpio:[0,1,4,11,12,14,19],gpio_all_pin:[3,6],gpio_pin:[3,6],gpioasynccontrol:[2,13],gpiocontrol:13,gpioexcept:2,gpiompssecontrol:[2,13],gpioport:2,gpiosynccontrol:[2,13],gpo:6,grain:19,grant:21,greater:[0,7],green:17,grei:17,group:[2,15],group_0_driv:11,group_0_schmitt:11,group_0_slew:11,group_1_driv:11,group_1_schmitt:11,group_1_slew:11,grow:19,guarante:11,guid:21,hack:[3,13],had:[16,19],half:[1,6,12],handi:0,handl:[3,13,19],hannesweisbach:9,hard:[2,19],hardcod:7,hardwar:[1,6,13,15],has:[0,1,2,3,6,7,13,15,16,17,18,19,20],has_cbu:[1,13],has_drivezero:1,has_mpss:1,has_seri:11,has_usb_vers:11,has_wide_port:1,have:[0,1,2,3,7,8,11,13,15,18,19,21,22],hello:[6,7],help:[0,7,11,12,19,20,21,22],helper:[4,8,22],here:[1,7,13,14,15,22],hexa:[11,13],hexabyt:0,hexadecim:[5,11,20,22],hexblock:11,hexdump:[0,5,11],hexlin:5,hide:19,high:[1,2,3,6,7,11,13,21],higher:1,highest:13,highli:8,hint:[0,14],histor:[13,16],hold:6,holder:16,host:[1,3,8,15,18,20],how:[1,2,3,5,6,12,13,20],howev:[3,7,13,16,18,22],html:15,http:[1,15],hub:15,human:[0,5,20],humm:9,hwflow:7,i2c:[1,2,4,13,17,20],i2c_rxf:11,i2c_tx:11,i2ccontrol:[3,13],i2cgpio:3,i2cgpioport:[3,13],i2cioerror:3,i2cnackerror:3,i2cport:3,i2cscan:12,i2ctimeouterror:3,ibackend:8,ic_nam:1,ident:5,identif:5,identifi:[1,8,15,20,22],idproduct:15,ids:8,idvendor:15,ifac:[1,8],ignor:[0,3,6,11,13,15],illeg:13,immedi:[1,22],implement:[1,6,7,11,13,14,16,19],impli:16,imposs:[7,13],improv:14,in_isochron:11,incept:19,incident:16,includ:[1,3,6,12,14,16,19],increas:[1,3,6,7,11,20],inde:11,indent:11,independ:13,index:[1,5,6,8,14,15,22],indic:[1,7,21],indirect:16,individu:15,info:19,inform:[1,8,15],infrastructur:19,ini:[0,11],initi:[0,1,2,3,6,8,16],initialiaz:13,input:[0,1,2,3,5,6,7,11,13,17,19],ins:13,instal:[7,14,18,20,21],instanc:[0,1,3,5,6,7,8,12,13,20],instanci:[3,5,6,8,13],instant:6,instantan:2,instanti:[0,3,6],instead:[1,13,14,15],instruct:14,integ:[1,2,3,5,6,13,15,20,22],integr:[5,19],intend:[1,13],intent:12,interfac:[0,1,2,3,6,7,8,15,17,22],intermedi:14,intern:[1,2,11],interpret:[13,20,22],interrupt:[1,16,22],introduc:14,introduct:3,invalid:[1,3,21],invers:[0,11],invert:[1,6],invok:[3,6,15],ioerror:13,is_bitbang_en:1,is_connect:1,is_empti:0,is_h_seri:1,is_inverted_cpha_support:6,is_iter:5,is_legaci:1,is_mpss:1,is_mpsse_interfac:1,ish:13,issu:[11,15,18,19,21],item:15,iter:[2,3,5,6],its:[1,2,3,11,13,16,19],itself:18,jedec:6,jedec_id:6,jitter:6,jnmacd:9,joker:8,jtag:[1,2,3,7,13,14,17,19],kbp:7,keep:[1,3,6,7,14,19],kei:11,kept:13,kernel:[18,21],kextunload:21,kib:5,kilo:5,kind:[1,8,19,21],kludg:6,know:2,known:[6,7],kwarg:[2,3,5,6],lack:6,languag:14,larg:[1,19],larger:1,last:[1,6,8,18],latenc:[1,3,6],latenti:1,later:13,latest:14,latter:21,layer:[12,16],ld_library_path:21,lead:[6,8],least:[1,6],leav:3,led:[11,13],legaci:[0,1,7,13],legrand:9,length:[0,1,2],leonard:9,less:6,lesser:16,level:[2,3,4,6,12,13,19,21,22],lgpl:16,liabil:16,liabl:16,lib:21,libftdi:16,librari:[6,18,21],libusb:[15,18,19,21],licens:[11,14],life:18,like:[3,6,13],lim_k:5,lim_m:5,limit:[1,2,3,5,11,12,13,16,19],line:[1,2,3,5,6,11,14,15,17,20],link:1,linux:[14,18,20,21],list:[0,1,2,7,8,11,15,16,20],list_devic:[1,8,22],lmax:1,lmin:1,load:[0,1,8,11,13,15,19,20,21],load_config:0,loader:[19,21],local:[7,14,20],localecho:7,locat:[1,7,8,11,12,20],log:[0,1,3,6,15],logger:21,logic:[1,3,5,7,13],longer:[2,6,13,21],loopback:[1,7],lose:1,loss:[1,16,21],low:[2,3,4,6,7,11,12,13,19,22],lower:[2,7],lowest:[3,6,13],lsb:[2,3,6,13],lsb_data:11,mach:13,maco:[14,18,20,21],made:[1,7,11],mai:[0,1,2,3,5,6,7,8,11,12,13,14,15,16,17,18,20,21,22],main:16,maintain:[1,14],make:15,manag:[0,1,3,6,7,12,14,19,20],mandatori:[0,5,11,13,19,20],mani:[2,7,11,13,15,19,20],manner:[6,13],manufactur:[0,11,22],map:[3,5,6,8,11,13],marcq:9,mark:9,markmelvin:9,mask:[0,1,3,13],massiv:19,master:[3,6,14,15],match:[1,2,5,8,13,15,19,20,22],materi:16,maverick:21,max:3,maxim:3,maximum:[1,3,6],mbp:[7,12],mccoi:9,mean:[1,2,3,6,7,13],meaningless:1,measur:7,medium:3,mega:5,meierphil:9,melvin:9,member:5,menu:15,merchant:16,mess:19,messag:[7,11],met:16,method:[2,3,6,8,13,14,15,20,21],mhz:[11,14],mib:5,michael:9,might:[0,1,2],mileag:18,millisecond:1,mind:7,mini:[11,20],minim:12,minimum:1,mirror:5,misc:[0,4],miscellan:4,miscellean:5,miso:[6,17],miss:[15,19,21],mock:19,mockusb:19,mode:[0,1,2,3,7,11,12,15,17,20],model:[11,13],modem:[1,7],modem_statu:1,modif:[11,16],modifi:[11,14,19],modul:[1,3,6,7,11,12,18,19,20],mojav:21,more:[1,3,5,6,7,8,11,13,14,19,20],moreov:7,mosi:[6,17],most:[1,3,7,11,13,14,19],mostli:1,move:21,mpratt14:9,mpsee:1,mpsse:[1,2,3,6,12,14,17,19],mpsse_bit_delai:1,msb:[2,6,13],much:[3,19],multi:[5,14],multipl:[1,15,19],multipli:5,multithread:13,muscaria:9,must:[1,13,15,16],mutual:2,mx25l1606e:6,mycompani:[5,20],myproduct:[5,15,20],myvendor:15,nack:3,name:[0,1,3,5,6,7,8,11,12,13,15,16,20],namw:11,narrow:13,nativ:[15,18,19,21],natur:0,naushir:9,nearli:19,need:[1,2,3,6,8,11,13,15,19,21,22],neglig:16,neither:[3,11,16],neotion:16,never:[3,6,15,21],nevertheless:[7,14,19],nevetherless:3,newgrp:15,next:[5,21],niku:9,nocach:[1,8],node:6,noflush:2,non:[1,2,3,6,12,13,14],none:[0,1,2,3,5,6,8],nopeppermint:9,nor:[11,16],normal:[1,7],note:[0,1,2,3,6,7,8,11,12,13,15,17,19,21,22],notic:16,now:[0,3,12,13,14,19],number:[0,1,6,11,13,15,22],numer:15,obj:5,object:[5,13,22],obtain:[1,3,6,7,8,11,22],occasion:[1,7],occur:[2,3],octavian:9,odd:1,offer:[11,12],offici:[1,6,11,14,15,18,20],offset:[3,6],ofter:13,old:[2,22],omit:[5,20],onc:[1,2,3,6,7,11,13,15],one:[1,2,3,6,7,8,11,13,14,18,19],ones:[1,3,6,7,13],onli:[0,1,2,3,5,6,8,11,12,13,14,15,17,18,19,20],onlin:14,onto:13,open:[0,1,2,7,8,12,13,15,16,20],open_:14,open_bitbang:[1,22],open_bitbang_from_devic:[1,22],open_bitbang_from_url:[1,22],open_from_devic:[1,22],open_from_url:[1,2,13,20,22],open_mpss:[1,22],open_mpsse_from_devic:[1,22],open_mpsse_from_url:[1,22],oper:[1,3,5,7],oppos:[1,7],optim:[1,3],option:[0,1,2,3,5,6,7,8,13,14,15,22],orang:17,order:[0,1,8,17,20,22],origin:[11,16],other:[1,2,3,6,16,18,22],otherwis:[2,3,16],ouput:13,out:[0,1,2,3,6,7,8,13,15,16,21,22],out_isochron:11,output:[0,1,2,3,6,7,8,11,13,17,19,22],over:[1,3,6,7,11,13],overrun:[1,7],overwrit:0,overwrite_eeprom:1,own:1,pace:[1,2,13],packag:[14,19],packet:[1,7],pad:11,pair:[1,8,11,15,22],param:3,paramet:[0,1,2,3,5,6,8,11,14,22],paran:0,parent:15,pariti:1,pars:[1,5,8,11],parse_url:8,part:[0,1,8,13,19],parti:1,partial:[8,19],particular:16,particularli:22,path:21,pattern:[1,8],payload:[1,21],pdf:1,pdict:8,peacefulli:21,peak:2,peek:2,peer:3,pep_498:18,pep_526:18,per:[2,6,14,15],perform:[1,3,6,11,12,13,14,19],permiss:[5,15,16],permit:16,perspect:[1,16],pete:9,phase:[1,6],physic:[0,1],pid:[1,5,7,8,11,20,22],pid_guidelin:1,pidnam:1,piec:19,pin:[0,1,2,3,6,7,11,12,14,17],pinout:[3,6,14],pip3:15,place:11,platform:[18,19],pleas:[11,13,15,19,21],plug:[8,11,15],plugdev:15,plural:5,point:[1,11],polar:6,poll:[1,3],poll_cond:3,poll_modem_statu:1,popular:14,port:[0,1,2,3,6,7,8,11,12,14,15,17,20],port_index:1,port_width:1,posit:[1,3,6,7,11,13],possibl:[2,3,5,6,7,11,13,14,16,19,22],potenti:[1,7,14],power:[11,13],power_max:11,powersav:11,preced:[3,6,11],precis:[6,13],predefin:13,preempt:21,prefer:[1,8,15,21,22],prefix:[7,22],present:[1,7,11,13,15],preserv:13,pretti:1,pretty_s:5,prettyfi:5,prevent:[3,13],previou:[0,1,2,3,6,13,14],previous:[6,8],primari:7,primarili:19,print:[0,1,5,7,11,22],prior:16,probabl:15,problem:[1,7],process:[1,15],procur:16,produc:7,product:[0,1,5,8,11,16,20,22],product_id:[1,5,11,15,20],product_nam:[5,20],profit:16,programm:13,progress:19,project:[12,16],prolog:6,promot:16,proper:[3,13,19],properi:0,properli:[3,6],properti:[0,1,2,3,6,11],proprietari:1,protect:11,protocol:[1,3,8,14],protocol_ftdi:7,proven:7,provic:13,provid:[0,1,2,3,6,8,13,14,16,17],publish:14,pull:[3,17],puls:[6,12],purdila:9,pure:[12,13,14,19],purge_buff:1,purge_rx_buff:1,purge_tx_buff:1,purpl:17,purpos:[1,5,12,16],pwd:7,pwren:11,pyftdi:[0,1,2,3,5,6,7,8,11,12,13,15,16,18,19,20,21,22],pyftditool:20,pyi2cflash:[3,12],pypi:14,pyseri:[1,7,12,14,15,18],pyspiflash:[6,12],pyterm:7,python3:[0,3,6,7,15],python:[7,12,13,14,18,19,20,22],pythonpath:[0,3,6,7,19],pyusb:[1,8,15,18,19,21,22],quad:14,queri:[1,13,22],quick:20,quot:7,rais:[1,3,5,13,22],random:[1,8],rang:[3,13],rate:2,rather:13,ratio:7,rational:13,raw:[0,11,13,21],rclk:17,reach:[1,3,6,18],reachabl:2,read:[1,2,3,6,7,8,11,12,13,14,15,19,21],read_buf:6,read_data:1,read_data_byt:1,read_data_get_chunks:1,read_data_set_chunks:1,read_eeprom:1,read_from:3,read_gpio:[3,6],read_pin:1,read_port:2,readabl:[0,5,20],readi:1,readlen:[2,3,6],readlin:14,readthedoc:15,real:[3,7,13],reason:[1,13,16],rebuilt:1,receiv:[1,3,6,7,11],recent:14,recogn:[15,20],recommand:8,recommen:13,recommend:[1,3,6,11,22],reconfigur:[3,6,13],recov:11,recoveri:11,redistribut:16,refer:[13,15,19,21],referenc:1,refus:3,regaddr:3,regener:11,regist:[2,3,21],regress:19,regular:[0,1,3,6,7,8,13,18,19,20],reject:[1,11],rel:3,relax:3,releas:[0,3,6,8,14,15],release_all_devic:8,release_devic:8,reli:[6,15,18,19,21],reliabl:[1,6,7,8],reload:[15,21],remain:[3,6],rememb:[13,15],remot:[1,3,6],remote_wakeup:11,remov:3,repeat:[3,7,11,20],replac:[5,7,13,15,19],report:[0,1,3,6,7,13,15,18,19,22],repositori:[3,6,15,19],repres:[3,6,13,15,19],represent:[5,11],reproduc:[16,19],requ:7,request:[1,2,3,6,11,13,19,21],request_gen:1,requir:[1,2,3,6,7,11,13,14,15,17,19,20],reserv:[3,6,14,16],reset:[0,1,13],reset_devic:0,resistor:[3,17],resolv:[5,20],resourc:19,respons:1,restor:11,restrict:[1,13],result:[6,7],resum:6,retain:16,retri:3,retriev:[1,2,3,6,8,14,22],reveal:19,review:6,rfda2125:6,richei:9,right:16,ring:1,risk:1,rout:19,rs232:[1,7,15],rs485:11,rsck:17,rtfm:1,rts:[1,7],rule:[13,15],run:[3,6,7,11,15,19,21],runtim:12,rxd:[1,17],rxled:11,safe:11,saint:9,sake:13,same:[1,2,7,8,11,12,13,16,21,22],sampl:[2,3,6,13],satisfi:3,satisifi:3,save:[0,11],save_config:0,scalar:13,scan:12,schemat:3,scheme:[1,8,14,19],schwamb:9,sck:[3,17],scl:[3,13],sclk:[6,17],script:[11,12,15,21],sda:[3,13,17],sda_i:3,sda_o:3,sdai:13,sdao:13,seamless:18,search:1,sebastian:9,second:[1,2,5,6,7,13,21],secondari:[17,19],section:[0,3,7,11,14,21],see:[0,1,2,3,6,7,8,11,12,13,14,15,19,20,21,22],seem:[2,11,15],seen:[1,7],seldom:7,select:[0,1,2,3,6,8,11,12,14,15,17,22],selector:[1,2,8,22],self:[3,6],self_pow:11,send:[1,3,6,7],sensibl:0,sent:1,sep:5,separ:[5,11],septemb:18,sequenc:[1,2,3,5,6,8,13],seri:[3,6,7,11,12,14,17],serial:[0,1,2,8,11,12,13,14,15,17,20,22],serial_for_url:[7,13],serial_numb:11,serialext:[4,12],sernum:1,serv:[5,20],servic:16,session:[7,22],set:[0,1,2,3,6,7,11,19],set_baudr:1,set_bit:1,set_bitmod:1,set_break:1,set_cbus_direct:[1,13],set_cbus_gpio:[1,13],set_direct:[3,6,13],set_dtr:1,set_dtr_rt:1,set_dynamic_lat:1,set_error_char:1,set_event_char:1,set_flowctrl:1,set_frequ:[1,2,6],set_gpio_direct:[3,6],set_latency_tim:1,set_line_properti:1,set_manufacturer_nam:0,set_mod:6,set_product_nam:0,set_properti:0,set_retry_count:3,set_rt:1,set_serial_numb:0,setup:[3,6,15,19,20],setuptool:15,seventh:13,sever:[1,2,3,7,8,11,12,13,14,19,20,21,22],sgoadhous:9,shall:16,shallow:5,share:[2,16,20],sheet:6,shell:15,shift:1,shift_address:3,ship:21,shortcut:1,shorter:[1,3],shoud:[1,7],should:[0,1,2,3,5,6,7,8,11,13,15,17,18,19,20,21,22],show:[0,7,8,11,13,20],show_call_stack:5,show_devic:[1,8,15,22],shpinx:15,side:1,sierra:21,sign:11,signal:[1,3,6,11],silent:7,similar:[11,13,16],simpl:[7,11,15,19,20,22],simpli:15,simplifi:[14,19],simultan:[12,14],sinc:8,singl:[1,6,13,14,15,19,22],size:[0,1,3,5,6,12,14],slace:6,slave:[1,3,6,12,14,19],sleep:[3,11],slight:2,slightli:13,slot:6,slow:[1,3],slower:[2,13],small:0,smaller:5,soc:12,sof:11,softwar:[1,3,13,16,19],solut:[1,8,12],some:[0,1,2,3,6,7,8,11,13,14,15,19,21,22],someth:11,sometim:13,somewhat:[7,13],soon:2,sound:13,sourc:[3,12,16,18],space:[1,14,15],spec:2,special:[1,2,3,6,11,13,16,22],specif:[1,3,6,13,15,16,19,22],specifi:[0,1,2,3,5,6,7,8,11,14,15,22],speed:11,sphinx:[14,15],sphinx_autodoc_typehint:15,sphinx_rtd_them:15,spi:[1,2,4,7,13,14,17,19],spi_mod:6,spicontrol:[6,13,14],spigpio:6,spigpioport:[6,13],spiioerror:6,spiport:6,split:[1,8],stabil:7,stabl:[7,14,15,21],stack:[3,5,19],stage:[12,19],standard:[1,6,11,18,22],standpoint:13,start:[1,3,6,7,8,13,14,15,20,21,22],state:[1,6,7,11],statu:[1,11],stdout:[0,1,8],stefan:9,stiebr:9,still:[12,13,14,21],stop:[1,2,3,6,7,22],stopbit:1,store:[0,15],str:[0,1,2,3,5,6,8],stream:[0,1,2,8],stretch:[3,13,17],strict:16,stridx:8,string:[0,1,3,5,6,8,11,15,20,22],strip:1,strobe:11,struct:3,studi:3,subdirectori:20,subsequ:[2,7,11],subset:[0,1],subshel:15,substitut:[16,19],subsystem:15,success:[15,18],sudo:[15,21],suffer:7,suffix:22,suport:14,support:[0,1,2,3,5,6,8,13,15,17,18,19,20,22],sure:[3,6,15,21],suspend:11,suspend_pull_down:11,suspsend:11,sync:[0,1,19],synchron:[1,2,6,11,12,14],synomym:13,synonym:13,syntax:[5,8,18,19,20,22],system:[21,22],systemexit:22,tabl:[7,11],tag:15,take:[1,3,5,6,11,21],tavip:9,tca9555:3,tck:17,tdi:17,tdo:17,technicalnot:1,tedku:9,tell:[1,3,5,15],term:11,termin:[1,3,6,12,20],terminolog:[1,13],test:[1,2,5,7,11,12,13,14,15,18,20],textio:[0,1,8],than:[1,2,6,7,11,13,14,20],thank:11,thanwith:3,thei:[1,2,5,13,17,18,19,20],them:[2,7],theme:15,theori:[7,16],therefor:[2,3,21,22],thi:[0,1,2,3,5,6,7,8,11,12,13,14,15,16,19,20,21,22],third:[1,6],those:[7,12,13,15,20],three:13,threshold:1,through:[0,1,3,7,11,12,13,21,22],throughput:[1,6],tim:9,time:[1,2,3,7,8,11,13,20],time_stamp:11,timeout:3,timer:1,timestamp:[6,11],tini:[20,21],tn_100_usb_vid:1,to_bool:5,to_bp:5,to_int:5,togeth:17,toggl:11,told:19,toler:[1,13],too:1,tool:[4,7,12,13,14,15,19,22],top:3,topolog:[1,19],tort:16,total:1,trace:1,traceback:20,tracer:[1,3,6],tracker:19,trail:12,transact:[3,6],transfer:[1,2,3,6,12,14],translat:13,transmit:[1,7,11],transpar:12,treatment:13,tri:[6,11,14,19,22],trick:3,tricki:13,trigger:[1,6,13,15,21],tristat:11,troubleshoot:14,tupl:[1,2,8],tuppl:1,turbo:6,tweak:[1,3],twice:7,two:[1,2,3,6,11,13,17,22],txd:[1,17],txden:11,txled:11,txrxled:11,txt:15,type:[0,1,2,3,5,6,7,8,11,13,14,20],typic:[5,11,13,15],uart:[1,2,4,13,14,17,19],uart_bit:0,uartbit:0,uartbridg:11,udev:[15,21],udevadm:15,unavail:13,uncheck:15,under:[11,16],underli:1,undesir:11,unexpect:[6,11],unfortun:2,uninstal:21,union:[0,1,2,3,5,6],uniqu:[1,15,22],unit:[1,5,19],unitless:6,unload:21,unmask:[3,6],unplug:[8,11,15],unspecifi:1,unstabl:11,unsupport:[3,11],until:2,unus:7,updat:[0,1,2,11,13,14],upgrad:14,uphi:21,upper:[7,11],uppercas:1,url:[0,1,2,3,6,7,8,11,14,15,19,20],urlpart:8,urlstr:8,usabl:7,usag:[0,1,8,13,20],usb:[0,1,3,4,5,6,7,11,13,14,19,20,22],usb_dev:[1,8],usb_path:1,usb_reset:1,usb_vers:11,usbdev:8,usbdevicedescriptor:[1,8],usbtool:[1,4,19],usbtoolserror:8,usbvirt:19,use:[0,1,2,3,5,6,7,8,11,12,13,15,16,19,20,21,22],used:[1,2,3,6,7,8,11,12,13,14,16,17,19,20],useful:[0,1,3,5,6,11,13,20,21,22],user:[8,14,15,18,21],uses:[3,18,19,21],using:[2,3,6,7,11,13,19,20,21,22],usual:[13,21],util:15,valid:[0,1,6,8,11,13,18],validate_address:3,validate_mpss:1,valu:[0,1,2,3,5,6,11,14,15,19,20,22],valueerror:[1,5],var_str:0,vari:[18,22],variabl:[0,18],variant:[13,14],vbu:11,vbus_sens:11,vcom:21,vdd:13,vdict:8,vector:13,vendor:[1,5,8,19,20,22],vendor_id:[1,5,11,15,20],vendor_nam:[5,20],verbos:[3,6,7,11,20],veri:[1,3,6,7,11,13],verifi:[19,21],version:[0,1,3,14,18,21],vestom:9,via:[11,13],viannei:9,vid:[1,5,7,8,11,20,22],vidnam:1,vidpid:[7,11,20],virtual:[7,11,13,20,21],volt:13,voltag:3,vps:[1,8],vpstr:5,wai:[0,2,11,13,15,16,21,22],wait:[2,3],want:[3,6,13,15,21],warn:[6,11],warranti:16,watch:3,weird:13,well:[7,13],were:19,wether:1,what:[0,1,3,7],whatev:[6,13],whatsoev:11,wheel:15,when:[0,1,2,3,5,6,7,8,11,13,14,17,22],whenev:[11,19,21],where:[1,2,3,6,11,12,13,14,15,21,22],wherea:1,whether:[0,1,2,3,5,6,7,13,16],which:[0,1,2,3,6,7,11,12,13,14,15,18,19,20,22],white:17,whole:[0,1,11,13],whose:[5,13],wich:[3,6],wide:[1,2,3,6,13],wider:13,width:[1,2,3,6,13],wildcard:11,win32:15,window:[14,18,20],winusb:15,wire:[12,19],wise:1,with_output:[3,6],within:[1,7,19],without:[1,3,11,13,14,16,19,22],word:[1,2,6],work:[1,3,11,18,19,20],workaround:[1,6,12],workload:19,world:[6,7],worth:2,would:20,wrapper:1,write:[1,2,3,6,7,11,12,13,19],write_buf:6,write_data:1,write_data_get_chunks:1,write_data_set_chunks:1,write_eeprom:1,write_gpio:[3,6],write_port:2,write_to:3,written:[0,1,2,3,11,13,16],wrong:11,wrongli:11,www:1,x00:3,x01:[3,6],x02:6,x03:6,xd0:3,xd7:3,xff:6,xor:5,yaml:[7,11,19],year:19,yellow:17,yet:[1,2,7,13,14],ymmv:6,you:[1,2,3,6,11,15,16,19,21,22],your:[1,3,11,13,14,15,18,19,22],zero:[1,3,6]},titles:["<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">eeprom</span></code> - EEPROM API","<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">ftdi</span></code> - FTDI low-level driver","<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">gpio</span></code> - GPIO API","<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">i2c</span></code> - I<sup>2</sup>C API","API documentation","<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">misc</span></code> - Miscellaneous helpers","<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">spi</span></code> - SPI API","<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">serialext</span></code> - UART API","<code class=\"xref py py-mod docutils literal notranslate\"><span class=\"pre\">usbtools</span></code> - USB tools","Authors","&lt;no title&gt;","EEPROM management","Features","GPIOs","PyFtdi","Installation","License","FTDI device pinout","Requirements","Testing","Tools","Troubleshooting","URL Scheme"],titleterms:{"class":[0,1,2,3,6,8],"function":[5,11],"switch":[11,20],Bus:21,IDs:15,OSes:14,The:21,Using:13,about:2,access:[7,13,21],api:[0,2,3,4,6,7,13],author:9,avail:[19,21],backend:21,base:22,bewar:19,bitmap:13,bsd:16,bug:21,bus:3,capitan:21,caveat:3,cbu:[7,11,13],chang:14,check:15,claus:16,clock:3,code:14,collector:3,common:[20,21],configur:[11,13,19],connect:22,content:19,contributor:9,control:7,custom:15,debian:15,definit:13,deni:21,depend:15,detail:14,develop:[9,18],devic:[12,17,21,22],direct:13,document:[4,14,15],driver:1,eeprom:[0,11,12,13,19],error:21,exampl:[11,13,19],except:[0,1,2,3,6,8,13],featur:[11,12,13,14],find:20,flow:7,framework:19,from:15,ft232h:13,ftconf:20,ftdi:[1,17],ftdi_url:20,gener:15,gpio:[2,3,6,7,13],hardwar:[7,19],has:21,helper:5,homebrew:15,host:14,i2c:[3,12],i2cscan:20,info:2,initialis:21,instal:15,insuffici:21,interfac:13,jtag:12,langid:21,legaci:22,level:1,licens:16,limit:[6,7],linux:15,log:21,low:1,maco:15,main:9,major:14,manag:11,master:12,messag:21,method:22,mini:7,misc:5,miscellan:5,mode:[6,13],modifi:13,open:[3,21,22],option:[11,20],other:13,overview:[13,14,19,20],permiss:21,pid:15,pin:13,pinout:17,pip:15,port:[13,21],post:15,prerequisit:15,product:15,pyftdi:14,pyterm:20,python:15,quickstart:[0,2,3,6,7],report:21,requir:18,reserv:13,retriev:13,saniti:15,scheme:22,script:20,sensit:6,serial:[7,21],serialexcept:21,serialext:7,serialutil:21,set:13,slow:21,sourc:[14,15],speed:3,spi:[6,12],state:13,statu:12,strech:3,support:[7,11,12,14],synchron:13,termin:7,test:[0,3,6,19],time:6,tool:[8,11,20],troubleshoot:21,uart:[7,12],ubuntu:15,unabl:21,url:22,usag:[6,7,19],usb:[8,15,21],usbtool:8,valu:13,vendor:15,vid:15,violat:21,virtual:19,warn:14,where:20,window:15,wip:19,wire:[3,6],zadig:15}})