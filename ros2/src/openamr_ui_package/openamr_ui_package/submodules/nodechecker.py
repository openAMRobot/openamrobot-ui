import subprocess

def map_server_check():
    nodelist = subprocess.Popen(["ros2 node list"],stdout=subprocess.PIPE, shell=True)
    out = nodelist.communicate()
    Nodes= out[0].decode('utf-8').split('\n')
    return '/lifecycle_manager' in Nodes
