
import json, ast
import get_yolo
from threading import Thread
def startApp():
    while 1:
        with open('data.json', 'r') as f:
            file =  json.load(f)
            if 'cup' in file['names']:
                print('bao')

if __name__ == '__main__':
    #Thread(target = startApp).start() 
    #Thread(target = get_yolo.listener).start()
    startApp()