openocd -f interface/stlink.cfg -f target/stm32f3x.cfg -c init -c "reset halt" -c "flash write_image erase build/hover.hex 0 ihex" -c "reset run"
