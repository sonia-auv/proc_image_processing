from flask import Flask, render_template

from model import Model
from src.filter_generator.app.factory import load
from tools import get_conf, get_files_from_path

app = Flask(__name__)
conf = get_conf()
model = Model()


@app.route('/')
def home():
    return render_template('index.html', filters=model.get("filters-list"))


@app.route('/config', methods=['POST'])
def config():
    pass


if __name__ == '__main__':
    model.put("filters-list", get_files_from_path(conf["filters-path"]))
    load(conf["factory-path"], conf["factory-name"], conf["factory-header-name"])
    app.run()
