from FLUKE_CLASS import *

def Read_command():
    text = open("Command.txt","r")
    datas = text.readlines()
    if len(datas) > 0:    
        datas = str(datas[0][:-1])
        print("Recieve Command - %s"%(datas))
        text.close()
        text = open("Command.txt","w")
    text.close()
    return datas

Controller = Control_motor()

Controller.Intial_Setting()

while 1:
    # print("\nPress any key to continue! (or press ESC to quit!)")
    c = Read_command()
    if c == chr(Controller.ESC_ASCII_VALUE):
        Controller.shutdown()
        break
    
    elif c == chr(Controller.W_ASCII_VALUE) or c == "w":
        Controller.control_up()
        
    elif c == chr(Controller.A_ASCII_VALUE):
        Controller.control_left()
        
    elif c == chr(Controller.S_ASCII_VALUE) or c == "s":
        Controller.control_down()
            
    elif c == chr(Controller.D_ASCII_VALUE):
        Controller.control_right()
        
    elif c == chr(Controller.H_ASCII_VALUE):
        Controller.control_speed_up()
        
    elif c == chr(Controller.L_ASCII_VALUE):
        Controller.control_speed_down()
        
    elif c == chr(Controller.X_ASCII_VALUE):
        Controller.waypoint_delete()
        
    elif c == chr(Controller.Q_ASCII_VALUE):
        Controller.waypoint_save()
        
    elif c == chr(Controller.R_ASCII_VALUE):
        Controller.control_rotation()
    
    elif c == chr(Controller.SPACE_ASCII_VALUE):
        Controller.control_stop()

    elif c == chr(Controller.M_ASCII_VALUE):
        Controller.contorl_move()
    
    time.sleep(0.5)
