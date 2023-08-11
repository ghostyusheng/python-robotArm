import requests
import json

def send(msg):
    if not msg:
        print('==> send nothing!')
        return
    url = 'http://www.tamashi.top:2000/put'
    d = {'log': json.dumps(msg)}
    r = requests.post(url, data=d)
    print(r.text)
    print('==> send!')
