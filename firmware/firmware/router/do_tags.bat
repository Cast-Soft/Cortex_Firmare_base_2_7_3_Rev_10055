@echo off
del tags.txt
ectags -x -R --exclude=startup --languages=-html App CMSIS CoOS STM32_USB-FS-Device_Driver STM32F10x_StdPeriph_Driver > tags.txt