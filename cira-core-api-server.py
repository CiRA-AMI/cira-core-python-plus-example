from flask import Flask, jsonify, request, Response
import threading
import json
import sys
import platform
from pathlib import Path
import base64

home_path = str(Path.home())
if platform.system() == 'Windows' :
        sys.path.append('C:\\CiRA-CORE\\install\\lib\\site-packages')
        sys.path.append('C:\\opt\\ros\\melodic\\x64\\lib\\site-packages')
else :
        sys.path.append(home_path + '/.cira_core_install/cira_libs_ws/install/lib/python3/dist-packages')

def handle_request(req):
    payload, img = getReqData(req)
    print("Have a nice day ^_^")
    resp = makeRespData(payload, img)

    return resp

from cira_msgs.srv import CiraFlowService, CiraFlowServiceRequest
from sensor_msgs.msg import CompressedImage

ciraServices = {}
locks = {}
app = Flask(__name__)
app.config['SOCK_SERVER_OPTIONS'] = {'ping_interval': 25}

@app.route("/")
def index():
    return 'Success'

@app.route("/api", methods=['POST'])
def api():
    if 'service_name' not in request.form:
        return jsonify({'error': 'Missing service_name in data'}), 400

    service_name = request.form['service_name']
    jsonstr = request.form.get('jsonstr', '{}')
    image = request.files.get('image', None)

    try:
        data_in = json.loads(jsonstr)
    except json.JSONDecodeError:
        return jsonify({'error': 'Invalid JSON'}), 400

    req = CiraFlowServiceRequest()

    req.flow_in.jsonstr = jsonstr

    if image:
        cm = CompressedImage()
        cm.format = "jpg"
        cm.data = image.stream.read()
        req.flow_in.img = cm

    if service_name not in ciraServices.keys():
        ciraServices[service_name] = rospy.ServiceProxy(
            service_name, CiraFlowService, persistent=True)
    if service_name not in locks.keys():
        locks[service_name] = threading.Lock()

    try:
        rospy.wait_for_service(service_name, timeout=5)
        with locks[service_name]:
            res = ciraServices[service_name].call(req)
    except rospy.ServiceException as exc:
        del ciraServices[service_name]
        return jsonify({'error': f'ROS Service call failed: {str(exc)}'}), 500
    except rospy.ROSException as exc:
        del ciraServices[service_name]
        return jsonify({'error': f'ROS Service not available: {str(exc)}'}), 503

    if len(res.flow_out.img.data) > 10:
        img_data = res.flow_out.img.data
        response = Response(img_data, mimetype="image/jpeg") # "image/png"
        response.headers['jsonstr'] = base64.b64encode(res.flow_out.jsonstr.encode('utf-8')).decode('utf-8')
        return response
    else:
        jsonstr_out = json.loads(res.flow_out.jsonstr)
        return jsonify(jsonstr_out)



def run_app():
    app.run(host='0.0.0.0', port=8082, debug=True, use_reloader=False)

thread = threading.Thread(target=run_app)
thread.daemon = True
thread.start()
