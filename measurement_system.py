from vision.msg import VisionMessage

old_msgs = VisionMessage()

def measurement_system(output_msg):
    global old_msgs
    alpha = 0.2
    print(old_msgs)
    for i in range(6):
        output_msg.x[i] = (1-alpha)*output_msg.x[i] + alpha*old_msgs.x[i]
        output_msg.y[i] = (1-alpha)*output_msg.y[i] + alpha*old_msgs.y[i]
        output_msg.th[i] = (1-alpha)*output_msg.th[i] + alpha*old_msgs.th[i]
        output_msg.u[i] = (1-alpha)*output_msg.u[i] + alpha*old_msgs.u[i]
        output_msg.w[i] = (1-alpha)*output_msg.w[i] + alpha*old_msgs.w[i]
    
    output_msg.ball_x = (1-alpha)*output_msg.ball_x + alpha*old_msgs.ball_x
    output_msg.ball_y = (1-alpha)*output_msg.ball_y + alpha*old_msgs.ball_y
    old_msgs = output_msg

    return output_msg