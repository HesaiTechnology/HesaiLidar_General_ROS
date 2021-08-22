# field		size(byte)
# SOB 		2
# angle		2
# measure	5
# block		SOB + angle + measure * 40
# timestamp	4
# factory	2
# reserve	8
# rpm		2
# tail		timestamp + factory + reserve + rpm
# packet	block * 6 + tail

builtin_interfaces/Time stamp
uint8[] data
uint32 size