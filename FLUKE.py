from FLUKE_CLASS import *

def Read_command():
    text = open("Command.txt","r")
    datas = text.readlines()
    if len(datas) > 0:    
        datas = str(datas[0][:-1])
        
        if len(datas) == 1:
            print("Recieve Command - %s"%(datas))
        else:
            datas = datas.split()
        text.close()
        text = open("Command.txt","w")
    text.close()
    # print(type(datas))
    # print("\n")
    # print(datas)
    return datas

Controller = Control_motor()

Controller.Intial_Setting()

while 1:
    # print("\nPress any key to continue! (or press ESC to quit!)")
    c = Read_command()
    if len(c) == 0:
        continue
     
    if c == chr(Controller.ESC_ASCII_VALUE) or c == chr(Controller.P_ASCII_VALUE):
        Controller.shutdown()
        break
    
    elif c == chr(Controller.W_ASCII_VALUE):
        Controller.control_up()
        
    elif c == chr(Controller.A_ASCII_VALUE):
        Controller.control_left()
        
    elif c == chr(Controller.S_ASCII_VALUE):
        Controller.control_down()
            
    elif c == chr(Controller.D_ASCII_VALUE):
        Controller.control_right()
        
    elif c == chr(Controller.H_ASCII_VALUE):
        Controller.control_speed_up() 
        
    elif c == chr(Controller.L_ASCII_VALUE):
        Controller.control_speed_down()
        
    elif c == chr(Controller.SPACE_ASCII_VALUE):
        Controller.control_stop()
        
    elif c[0] == chr(Controller.X_ASCII_VALUE):
        Controller.waypoint_delete(c[1:])
        
    elif c[0] == chr(Controller.Q_ASCII_VALUE):
        Controller.waypoint_save(c[1:])
    
    elif c[0] == chr(Controller.M_ASCII_VALUE):
        Controller.control_move(c[1:])
    
    elif c[0] == chr(Controller.R_ASCII_VALUE):
        Controller.control_rotation(c[1:])
        
    # else:
    #     if c[0] == chr(Controller.R_ASCII_VALUE):
    #         Controller.control_rotation(c[1:])
    #     elif c[0] == chr(Controller.Q_ASCII_VALUE):
    #         Controller.waypoint_save()
            
    time.sleep(0.5)