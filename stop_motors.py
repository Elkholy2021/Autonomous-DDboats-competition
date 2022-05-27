from write_to_arduino import send_arduino_cmd_motor,init_arduino_line

arduino, data = init_arduino_line()
send_arduino_cmd_motor(arduino,0,0)