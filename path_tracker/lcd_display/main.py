import i2c_lcd
from time import *

mylcd = i2c_lcd.lcd()

mylcd.lcd_display_string("180")
sleep(0.1)
mylcd.lcd_clear()
