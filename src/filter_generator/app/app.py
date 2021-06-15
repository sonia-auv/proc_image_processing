from pathlib import Path

from flask import Flask, render_template

import factory as fa
import factory_header as fah
import filter as f
from model import Model
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
    factory_header = fah.load(Path(conf["factory-path"], conf["factory-header-filename"]))
    factory = fa.load(Path(conf["factory-path"], conf["factory-filename"]))
    model.put(
        "filters-list",
        f.load_all(
            get_files_from_path(conf["filters-path"])
        )
    )
    app.run()
