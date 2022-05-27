from write_to_arduino import send_arduino_cmd_motor,init_arduino_line
while True:
    arduino, data = init_arduino_line()
    send_arduino_cmd_motor(arduino,60,0)
    