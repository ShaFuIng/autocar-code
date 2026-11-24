import os
c.ServerApp.ip = '0.0.0.0'
c.ServerApp.port = 8888
c.ServerApp.open_browser = False
c.ServerApp.root_dir = '/workspace'
c.ServerApp.allow_root = False
c.ServerApp.allow_remote_access = True
c.ServerApp.tornado_settings = {
    'headers': {'Content-Security-Policy': 'frame-ancestors http://' + os.environ['ORIN_HOST'] + ':8000'}
}
