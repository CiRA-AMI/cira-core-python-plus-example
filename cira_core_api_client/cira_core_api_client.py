import requests
from requests.models import Response
from requests_toolbelt.multipart.encoder import MultipartEncoder
import pathlib
import cv2
import json
import numpy as np

url = "http://localhost:8082" #you can change server ip here

file = open('img.png', 'rb') 

m = MultipartEncoder(
    fields={
        'service_name': 'deepdetect',  #in example you can change to facerec
        'jsonstr': '{"a": 100}',       #send payload
        'image': ('img.png', file)
    }
)

res: Response = requests.post(
    url,
    data=m,
    headers={'Content-Type': m.content_type}
)

json_result = res.headers
have_img = len(res.content) > 0
payload_out = json.loads(json_result['jsonstr'])['payload']
print(payload_out)

if ok and have_img :
    img = cv2.imdecode(np.frombuffer(res.content, np.uint8), cv2.IMREAD_COLOR)
    cv2.imshow('img_out', img)
    cv2.waitKey(3000)
