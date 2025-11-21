import glob
import time
import socket
import iphone
import numpy as np

images = []
nums = []

for file in glob.glob("data/*.jpg"):
    with open(f"{file}",'rb') as f:
        images.append( f.read() )
        nums.append(int(file.split('.')[0].split('/')[-1]))

images = [images[i] for i in np.argsort(nums)]

FRAME_RATE = 60
DELTA_T = 1 / FRAME_RATE

tt = time.time()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect(('localhost', 8892))
    iphone_name = "iphone_emulator"
    header = iphone.Header(
        message_id=iphone.MessageIDs.iPhoneName,
        payload_length=len(iphone_name),
        timestamp_sec=int(tt),
        timestamp_nsec=int(1e9*(tt%1))
    )
    s.send( bytes( header ) + iphone_name.encode('utf-8') )
    while True:
        for im in images:
            time.sleep( max( 0, tt - time.time() ) )
            header = iphone.Header(
                message_id=iphone.MessageIDs.CameraFrame,
                payload_length=len(im),
                timestamp_sec=int(tt),
                timestamp_nsec=int(1e9*(tt%1))
            )
            tt += DELTA_T
            s.send(
                bytes( header ) +
                im 
            )
        print(f"Looping...")

