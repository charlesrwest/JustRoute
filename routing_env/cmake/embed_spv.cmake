# Embed a SPIR-V binary as a C array header. Usage:
#   cmake -DIN=<file.spv> -DOUT=<header.h> [-DSYM=<symbol>] -P embed_spv.cmake
if(NOT DEFINED SYM)
  set(SYM g_wave_spv)
endif()
file(READ ${IN} HEXDATA HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," C_ARR ${HEXDATA})
file(WRITE ${OUT}
     "static const unsigned char ${SYM}[] = {${C_ARR}};\n"
     "static const unsigned ${SYM}_len = sizeof(${SYM});\n")
