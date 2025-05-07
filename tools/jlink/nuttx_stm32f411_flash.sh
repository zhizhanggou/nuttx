#!/bin/bash
JLinkExe -device STM32F411CE -ip 192.168.31.124 -if SWD -speed 4000 -autoconnect 1 -CommandFile ./tools/jlink/nuttx_stm32f411_cmd_file.jlink

