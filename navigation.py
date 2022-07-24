'''
This the main control node that calculates the velocities of each DC Motors and sends it to the microcontroller through rosserial
Created by the No_Mercy_NCD team

'''

# /usr/bin/env python3.8
import rospy
import cv2.aruco as aruco
import cv2
from geometry_msgs.msg import Quaternion


#params
msg = Quaternion()
Kp = 0.4
Ki = 0.002
AreaMargin = 4500
ReachedMargin = 7500
k = 100
k2 = -90

def getCenter(Points):
    x = 0
    y = 0
    diffx=abs(Points[0][0]-Points[1][0])
    diffy=abs(Points[0][1]-Points[1][1])
    area=(pow(diffx,2)+pow(diffy,2))
    for i in Points:
        x += int(i[0])
        y += int(i[1])
    return ((x // 4, y // 4), area)

def main():
    sum = 0

    cap = cv2.VideoCapture(3)

    while True:
        _, img = cap.read()

        #Aruco Detection
        ArucoDict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        ArucoParams = aruco.DetectorParameters_create()
        corners, ids, rej = aruco.detectMarkers(img, ArucoDict, parameters=ArucoParams)
        frame = aruco.drawDetectedMarkers(img, corners)

        try:
            if ids is not None:
                for i in range(len(ids)):
                    # Get Center and Area of Aruco
                    Center = getCenter(corners[i][0])[0]
                    Area = getCenter(corners[i][0])[1]
                    img = cv2.circle(img, Center, 3, [0,0,255, -1])

                    #error and P Control
                    error = img.shape[1] // 2 - Center[0]
                    sum += error
                    signal = Kp * error + Ki * sum

                    # Calculate Velocities
                    if ids[0] == 0:
                            if Area < AreaMargin:
                                msg.x = msg.y = 0
                                msg.z = (ReachedMargin / (ReachedMargin + Area)) * k - signal
                                msg.w = (ReachedMargin / (ReachedMargin + Area)) * k + signal     
                            else:
                                msg.x = msg.y = msg.z = msg.w = (ReachedMargin / (ReachedMargin + Area)) * k2
                    elif ids[0]==1:
                            if Area < AreaMargin:
                                msg.x = msg.y = 0
                                msg.z = (ReachedMargin / (ReachedMargin + Area)) * k - signal
                                msg.w = (ReachedMargin / (ReachedMargin + Area)) * k + signal
                            
                            else:
                                msg.x = msg.y = msg.z = msg.w = (ReachedMargin / (ReachedMargin + Area)) * k2       
                    elif ids[0]==2:
                            if Area < AreaMargin:
                                msg.x = msg.y = 0
                                msg.z = (ReachedMargin / (ReachedMargin + Area)) * k - signal
                                msg.w = (ReachedMargin / (ReachedMargin + Area)) * k + signal
                            
                            else:
                                msg.x = msg.y = msg.z = msg.w = (ReachedMargin / (ReachedMargin + Area)) * k2
                    elif ids[0]==3:
                            if Area < AreaMargin:
                                msg.x = msg.y = 0
                                msg.z = (ReachedMargin / (ReachedMargin + Area)) * k - signal
                                msg.w = (ReachedMargin / (ReachedMargin + Area)) * k + signal
                            
                            else:
                                msg.x = msg.y = msg.z = msg.w = (ReachedMargin / (ReachedMargin + Area)) * k2
        except Exception as inst:
            print(inst)
        pub.publish(msg)    
        cv2.imshow('frame', img)
        if cv2.waitKey(1) == 27:
            break

if __name__ == '__main__':
    rospy.init_node('Navigation', anonymous=True)
    pub = rospy.Publisher('Velocities', Quaternion, queue_size=10)
    main()


