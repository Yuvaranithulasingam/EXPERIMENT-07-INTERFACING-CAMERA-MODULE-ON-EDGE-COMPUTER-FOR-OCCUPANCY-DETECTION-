# EXPERIMENT-07-INTERFACING-CAMERA-MODULE-ON-EDGE-COMPUTER-FOR-OCCUPANCY-DETECTION-

#### Name : YUVARANI T
#### Register no : 212222110057
#### Department : CSE(IOT)
#### Date :

### AIM:
To interface a USB/CSI camera module with an edge computing platform (e.g., Raspberry Pi, Jetson Nano, etc.) and implement an occupancy detection system using the Histogram of Oriented Gradients (HOG) algorithm.

### Apparatus/Software Required:

S. No.	Equipment / Software	Specification<br>
1.	Edge Computing Device	Raspberry Pi 4 / Jetson Nano<br>
2.	Camera Module	USB Webcam / Pi Camera Module<br>
3.	Operating System	Raspbian OS / Ubuntu<br>
4.	Programming Language	Python 3.x<br>
5.	Libraries	OpenCV, imutils, NumPy<br>
6.	Display Output	HDMI Monitor / VNC Viewer<br>

### Theory:
Histogram of Oriented Gradients (HOG) is a feature descriptor used in computer vision and image processing for the purpose of object detection. It counts occurrences of gradient orientation in localized portions of an image. HOG descriptors are particularly useful for detecting humans (pedestrians) in static images or video frames.

Steps involved in HOG-based Occupancy Detection:<br>
Capture frames from the camera.<br>
Resize and preprocess the image.<br>
Use a pre-trained HOG descriptor with a linear SVM to detect people in the image.<br>
Annotate the image with bounding boxes where people are detected.<br>
Display or store the result.<br>
Circuit Diagram / Setup:<br>
Connect the USB camera to the edge computer via a USB port.<br>
Power on the edge device and boot into the OS.<br>
Ensure necessary Python libraries are installed.<br>

### Procedure:
Set up the edge device with a monitor or SSH/VNC connection.<br>
Connect and verify the camera using commands like ls /dev/video* or vcgencmd get_camera.<br>
Install required libraries:<br>
```
pip install opencv-python imutils numpy
```
Write the Python code to initialize the camera and implement the HOG algorithm.<br>
Run the code and verify that the system detects human presence and draws bounding boxes.<br>

 ###  Python Code:
 ```
import cv2
import imutils

###  Initialize HOG descriptor with people detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

### Initialize video capture
cap = cv2.VideoCapture(0)  # Change index if using CSI camera

while True:
    ret, frame = cap.read()
    if not ret:
        break

  ### Resize frame for faster processing
    frame = imutils.resize(frame, width=640)

  ### Detect people in the image
    (rects, weights) = hog.detectMultiScale(frame, winStride=(4, 4),
                                            padding=(8, 8), scale=1.05)

 ### Draw bounding boxes
    for (x, y, w, h) in rects:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

  ### Display the result
    cv2.imshow("Occupancy Detection", frame)

###  Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### OUTPUT :

![image](https://github.com/user-attachments/assets/e74e10ee-f01f-46d8-bf18-ce3650d75aea)

#### RASPI INTERFACE:

![image](https://github.com/user-attachments/assets/3b52e859-8da8-41be-9ab6-83ab7bd0ff9a)

### RESULT:
Occupancy detection using the HOG algorithm was successfully implemented. The system was able to identify and highlight human presence in real-time video streams.

