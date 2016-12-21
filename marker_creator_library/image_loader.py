from flask import Flask
from flask import render_template
import os
from random import choice
from string import Template

import time

image_loader = Flask(__name__)

@image_loader.route('/')
def index():
    template = render_template('index.html')
    return template

if __name__ == '__main__':
    port = int(os.environ.get('PORT', 5000))

    if port == 5000:
        image_loader.debug = True

    image_loader.run(host='130.83.110.224', port=port)
