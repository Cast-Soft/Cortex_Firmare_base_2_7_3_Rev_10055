Version 3.0.4.12262
https://blackbox/svn/blackbox/blacktrax/branches/H2.7/firmware
Changed clock configuration to operate on modified TK board with only the precise clock as source to system and peripherals.  
Adjusted baudrate for optihub output to match expected output rate as a workaround.  The configured rate is different than the actual output, but the cause has not yet been determined.
Fix to inline ASM so it will compile in IAR 7.