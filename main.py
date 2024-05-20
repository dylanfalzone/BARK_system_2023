import machine
import math
import utime
import LCD
from neopixel import Neopixel


"""FUNCTIONS========================================================================================================================="""

#Button ISR changes value of var: switch
def interrupt(pin):
    global flag
    global button
    if flag == 1:
        button = 1
        beep(1)
        utime.sleep_ms(10) #somewhat essential debouncing
    flag = 0

#Beep speaker for given length
def beep(length):
    for k in range(20*length):
        for i in sine:
            buf = (0xF000 | (i << 2)).to_bytes(2,1)
            cs(0)
            spi.write(buf)
            cs(1)
  
#Probe ADC0 for pot value 1-7    
def potprobe():
    pot_values = [None] * 10
    for i in range(10):
        #Read ADC pins
        pot_v = pot_pin.read_u16()
        ground_v = ground_pin.read_u16()
        #Turn them into voltages
        pot_v = (pot_v / 65535.0) * 3.3
        ground_v = (ground_v / 65535.0) * 3.3
        #Get a voltage without noise
        pot_v = (pot_v-ground_v)
        #Edge case for 0v
        if pot_v <= 0:
            pot_v = 0.000805676
        #Fill array
        pot_values[i] = pot_v
    #Average the array
    pot_values_total = sum(pot_values)
    pot_v = pot_values_total/10
    pot_v = pot_v*10000
    
    if 1<=pot_v<65:
        pot =1
    elif 65<=pot_v<130:
        pot =2
    elif 130<=pot_v<195:
        pot = 3
    elif 195<=pot_v<260:
        pot = 4
    elif 260<=pot_v<325:
        pot = 5
    elif 325<=pot_v<390:
        pot = 6
    elif 390<=pot_v<455:
        pot = 7
    else:
        pot = 8
    return(pot)
    

def tempprobe():
    temp_values = [None] * 10
    for i in range(10):
        #Read ADC pins
        temp_v = bio_pin.read_u16()
        ground_v = ground_pin.read_u16()
        #Turn it into voltage
        temp_v = (temp_v / 65535.0) * 3.3
        ground_v = (ground_v / 65535.0) * 3.3
        #Get a voltage without noise
        temp_v = (temp_v-ground_v)
        #Edge case for 0v
        if temp_v <= 0:
            temp_v = 0.000805676
        #Fill array
        temp_values[i] = temp_v
    #Average the array
    temp_values_total = sum(temp_values)
    temp_v = temp_values_total/10
    temp_c = (temp_v-0.5)/.01
    temp = temp_c*(9/5) + 32
    return(temp)

def hrprobe():
    global hr_v_last
    beats = 0
    start_t = utime.time()
    #while utime.time() - start_t < 10:
    hr_values2 = [None] * 990
    for j in range(990):
        hr_values = [None] * 100
        for i in range(100):
            #Read ADC pins
            hr_v = bio_pin.read_u16()
            ground_v = ground_pin.read_u16()
            #Turn it into voltage
            hr_v = (hr_v / 65535.0) * 3.3
            ground_v = (ground_v / 65535.0) * 3.3
            #Get a voltage without noise
            hr_v = (hr_v-ground_v)*1000
            #Edge case for 0v
            if hr_v <= 0:
                hr_v = 0.000805676
            #Fill array
            hr_values[i] = int(hr_v)
        #Average the array
        hr_values_total = sum(hr_values)
        hr_v = int(hr_values_total/100)
        
        hr_values2[j] = hr_v
    print(hr_values2)
    smooth = moving_average(hr_values2,2)
    #for k in range(989):
    peaks = find_peaks(smooth, 99)
    hr = peaks*10
    return(hr)


def moving_average(data, window_size):
    """Apply a simple moving average filter to the data."""
    filtered_data = []
    for i in range(len(data) - window_size + 1):
        window = data[i : i + window_size]
        average = sum(window) / window_size
        filtered_data.append(average)
    return filtered_data


def find_peaks(data, threshold_percent):
    """Count the number of peaks in the data based on a relative threshold."""
    max_value = max(data)
    threshold = threshold_percent / 100.0 * max_value

    peak_count = 0
    for i in range(1, len(data) - 1):
        if data[i] > threshold and data[i] > data[i - 1] and data[i] > data[i + 1]:
            peak_count += 1
    return peak_count


#Write variable to LCD  
def lcdvar(a):
    LCD.lcd_clear()
    LCD.lcd_home()
    LCD.lcd_puts("{:.2f}".format(a))

#Write variable to LCD  
def homescreen(a):
    LCD.lcd_clear()
    LCD.lcd_home()
    if a == 1:
        LCD.lcd_puts("LED: [BRT] COL  ")
        LCD.lcd_goto(0,1)
        LCD.lcd_puts("BIO:  TMP  H.R. ") 
    elif a ==2:
        LCD.lcd_puts("LED:  BRT [COL] ")
        LCD.lcd_goto(0,1)
        LCD.lcd_puts("BIO:  TMP  H.R. ") 
    elif a ==3:
        LCD.lcd_puts("LED:  BRT  COL  ")
        LCD.lcd_goto(0,1)
        LCD.lcd_puts("BIO: [TMP] H.R. ") 
    else:
        LCD.lcd_puts("LED:  BRT  COL  ")
        LCD.lcd_goto(0,1)
        LCD.lcd_puts("BIO:  TMP [H.R.]")
        
def brightscreen(a):
    LCD.lcd_clear()
    LCD.lcd_home()
    LCD.lcd_puts("BRIGHTNESS:")
    progressbar(a)
    
def colorscreen(a, p):
    LCD.lcd_clear()
    LCD.lcd_home()
    if p == 0:
        LCD.lcd_puts("RED:")
        progressbar(i)
    elif p == 1:
        LCD.lcd_puts("GREEN:")
        progressbar(i)
    elif p== 2:
        LCD.lcd_puts("BLUE:")
        progressbar(i)
        
def tempscreen(a):
    if a == 0:
        LCD.lcd_clear()
        LCD.lcd_home()
        LCD.lcd_puts("TEMPERATURE:")
    t = tempprobe()
    LCD.lcd_goto(5,1)
    LCD.lcd_puts("{:.2f}".format(t))
    LCD.lcd_goto(9,1)
    LCD.lcd_putch(chr(0xdf))
    LCD.lcd_goto(10,1)
    LCD.lcd_puts("F")
    
def hrscreen(a):
    if a == 0:
        LCD.lcd_clear()
        LCD.lcd_home()
        LCD.lcd_puts("HEART RATE:")
    hr = hrprobe()
    LCD.lcd_goto(5,1)
    LCD.lcd_puts("{:.2f}".format(hr))

    
        
    
        
def progressbar(a):
    LCD.lcd_goto(3,1)
    LCD.lcd_puts("[")
    for b in range(a):
        LCD.lcd_goto(4+b,1)
        LCD.lcd_putch(chr(0xff))
    """
    for b in range(8-a):
        LCD.lcd_goto(11-b,1)
        LCD.lcd_puts(" ")
    """
    LCD.lcd_goto(12, 1)
    LCD.lcd_puts("]")
    
    
        


"""PINS============================================================================================================================="""
#Function Pins
pot_pin = machine.ADC(27) #ADC1 = POT
ground_pin = machine.ADC(28)#ADC2 = Ground ref
bio_pin = machine.ADC(26)#ADC0 = temp or HR
button_pin = machine.Pin(15, machine.Pin.IN) #GP15, pin20 = button
temp_toggle_pin = machine.Pin(22, machine.Pin.OUT) #GP22, pin 29 = power for TMP
hr_toggle_pin = machine.Pin(21, machine.Pin.OUT) #GP21, pin 27 = power for HR

#DAC pins
spi = machine.SPI(1, baudrate=1000000, polarity=0, phase=0, sck=machine.Pin(10), mosi=machine.Pin(11))
cs = machine.Pin(13, mode=machine.Pin.OUT, value=1)
ref = machine.Pin(12, mode=machine.Pin.OUT, value=1)

#LCD pins
EN = machine.Pin(0, machine.Pin.OUT)
RS = machine.Pin(1, machine.Pin.OUT)
D4 = machine.Pin(2, machine.Pin.OUT)
D5 = machine.Pin(3, machine.Pin.OUT)
D6 = machine.Pin(4, machine.Pin.OUT)
D7 = machine.Pin(5, machine.Pin.OUT)
PORT = [D4, D5, D6, D7]


"""VARIABLES/SETUP===================================================================================================================="""

sine = [0x200,0x27f,0x2f7,0x35e,0x3b0,0x3e7,0x3ff,0x3f7,
        0x3cf,0x38b,0x32d,0x2bc,0x240,0x1c0,0x144,0xd3,
        0x75,0x31,0x9,0x1,0x19,0x50,0xa2,0x109,
        0x181,0x200]

#FSM contorl variable: 0-4
state = 0

#Repeat counters
presses = 0 #0-3 for cycling colorscreen
repeats = 0 #0-1 for temp
hr_v_last = 0
hr_v_last2 = 0


#Button interrupt variables: both binary
flag = 1
button = 0

#LED variables
strip = Neopixel(60, 0, 21, "RGB")
brightness = 100 #0-255 
green = 0      #0-255
red = 0        #0-255
blue = 255     #0-255
strip.fill((green,red,blue), brightness)
strip.show()


#LCD setup
screenhold = 0 #0-4 but also 10 when changing state
LCD.Configure()
LCD.lcd_init()

# Configure the button pin to trigger the interrupt on a rising edge
button_pin.irq(trigger=machine.Pin.IRQ_RISING, handler=interrupt)

#Beep for system on and to reset speaker
beep(1)

"""MAIN LOOP========================================================================================================================="""
while True:

    #Home Screen - Default State
    if state == 0:
        i = potprobe()
        i = (i + 2 - 1) // 2 #map i range 1-8 to state/function range 1-4
        if i != screenhold:
            homescreen(i)
        if button == 1:
            state = i
            screenhold = 10
        else:
            state = state
            screenhold = i
        
        
    #Brightness - State 1
    elif state == 1: 
        i = potprobe()

        brightness = (i-1)*30
        if i != screenhold:
            brightscreen(i)
            
        strip.fill((green,red,blue), brightness)
        strip.show()
        #Return to Home Screen
        if button == 1:
            state = 0
            screenhold = 10
        
        
    #Color - State 2
    elif state == 2:
        i = potprobe()
        #Tune Red
        if presses == 0:
            if i != screenhold:
                colorscreen(i,presses)
                red = (i-1)*30
                strip.fill((green,red,blue), brightness)
                strip.show()
            if button ==1:
                presses = 1        
        #Tune Green
        elif presses == 1:
            if i != screenhold:
                colorscreen(i,presses)
                green = (i-1)*30
                strip.fill((green,red,blue), brightness)
                strip.show()
            if button ==1:
                presses = 2
        #Tune Blue
        elif presses == 2:
            if i != screenhold:
                colorscreen(i,presses)
                blue = (i-1)*30
                strip.fill((green,red,blue), brightness)
                strip.show()
            if button ==1:
                presses = 3
        #Leave
        else:
            presses = 0
            state = 0
            screenhold = 10
            
            
    #Temperature - State 3    
    elif state == 3:
        temp_toggle_pin.value(1) #Turn on temp
        tempscreen(repeats)
        repeats = 1
        #Return to Home Screen
        if button == 1:
            state = 0
            screenhold = 10
            repeats = 0
            temp_toggle_pin.value(0) #Turn off temp
            
            
    #Heart Rate - State 4
    elif state == 4:
        hr_toggle_pin.value(1) #Turn on hr
        hrscreen(repeats)
        repeats = 1

        #Return to Home Screen
        if button == 1:
            state = 0
            screenhold = 10
            repeats = 0
            hr_toggle_pin.value(1) #Turn off hr
    
    #Return home if state is somehow not between 0 and 4
    else:
        state = 0
        
    
    
      

    #Refresh button interrupt
    button = 0
    flag = 1
    #Sleep
    utime.sleep_ms(100)
    





