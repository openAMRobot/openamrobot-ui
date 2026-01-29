import rclpy
import webbrowser
from rclpy.node import Node
from flask import Flask,render_template


app = Flask(__name__)

@app.route('/')
@app.route('/route')
@app.route('/control')
@app.route('/info')
def index():
    return render_template('index.html')

class ParamFlask(Node):
    def __init__(self):
        super().__init__('flask')
        #Default adress
        self.declare_parameter("appAddress","127.0.0.1")
        self.declare_parameter("portApp",5050)

def main():
    rclpy.init()
    Nodeserv= ParamFlask() #It is assumed that we need a node for the parameters
    #TODO: Implement the correct way to get parameters from the launch file
    local_ip = Nodeserv.get_parameter('appAddress').get_parameter_value().string_value
    local_port = Nodeserv.get_parameter('portApp').get_parameter_value().integer_value
    app.run(host=local_ip, port=local_port) # Start the Flask app

if __name__ == '__main__':
    main()