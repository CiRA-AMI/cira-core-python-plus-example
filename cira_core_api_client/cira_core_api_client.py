import requests
from requests.models import Response
from requests_toolbelt.multipart.encoder import MultipartEncoder
import pathlib
import cv2
import json
import numpy as np
import base64

url = "http://localhost:8082/api" #you can change server ip here

file = open('img.jpg', 'rb') 

m = MultipartEncoder(
    fields={
        'service_name': 'deepdetect',  #in example you can change to facerec
        'jsonstr': '{"a": 100}',       #send payload
        'image': ('img.jpg', file)
    }
)

res: Response = requests.post(
    url,
    data=m,
    headers={'Content-Type': m.content_type}
)

json_result = res.headers
have_img = len(res.content) > 0

if not have_img :
    payload_out = json.loads(json_result['jsonstr'])['payload']
else :
    encoded_jsonstr = res.headers['jsonstr']
    decoded_jsonstr = base64.b64decode(encoded_jsonstr).decode('utf-8')
    payload_out = json.loads(decoded_jsonstr)['payload']


print(payload_out)

if have_img :
    img = cv2.imdecode(np.frombuffer(res.content, np.uint8), cv2.IMREAD_COLOR)
    cv2.imshow('img_out', img)
    cv2.waitKey(3000)
