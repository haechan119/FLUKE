def Send_command(command):
    # Write Command text
    text = open("Command.txt",'w')
    text.write(str(command) + '\n')
    text.close()
    

while True:
    
    command = input("Send command : ")
    Send_command(command)