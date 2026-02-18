"""AVROS Web UI node: phone joystick -> ActuatorCommand.

Ported from ~/AV2.1-API/webui/server_standalone.py.
Replaces raw UDP with ROS2 ActuatorCommand publishing.

FastAPI + WebSocket server (uvicorn) in main thread,
rclpy.spin in a background daemon thread.
"""

import os
import threading

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from avros_msgs.msg import ActuatorCommand, ActuatorState

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import uvicorn


class WebUINode(Node):
    """ROS2 node that bridges WebSocket joystick input to ActuatorCommand."""

    def __init__(self):
        super().__init__('webui_node')

        self.declare_parameter('web_port', 8000)
        self.declare_parameter('ssl_certfile', '')
        self.declare_parameter('ssl_keyfile', '')
        self.declare_parameter('max_throttle', 0.55)

        self.web_port = self.get_parameter('web_port').value
        self.ssl_certfile = self.get_parameter('ssl_certfile').value
        self.ssl_keyfile = self.get_parameter('ssl_keyfile').value
        self.max_throttle = self.get_parameter('max_throttle').value

        self._cmd_pub = self.create_publisher(
            ActuatorCommand, '/avros/actuator_command', 10
        )
        self._state_sub = self.create_subscription(
            ActuatorState, '/avros/actuator_state',
            self._state_callback, 10
        )

        self._state_lock = threading.Lock()
        self._latest_state = {
            'e': 0, 't': 0.0, 'b': 0.0, 's': 0.0, 'm': 'N', 'w': 0
        }

        self.get_logger().info(
            f'WebUI node started (port={self.web_port})'
        )

    def _state_callback(self, msg: ActuatorState):
        with self._state_lock:
            self._latest_state = {
                'e': 1 if msg.estop else 0,
                't': msg.throttle,
                'b': msg.brake,
                's': msg.steer,
                'm': msg.mode,
                'w': 1 if msg.watchdog_active else 0,
            }

    def get_state(self) -> dict:
        with self._state_lock:
            return dict(self._latest_state)

    def publish_command(
        self, throttle: float, brake: float, steer: float,
        mode: str, estop: bool
    ):
        msg = ActuatorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.estop = estop
        msg.throttle = min(max(0.0, throttle), self.max_throttle)
        msg.brake = min(max(0.0, brake), 1.0)
        msg.steer = min(max(-1.0, steer), 1.0)
        msg.mode = mode
        self._cmd_pub.publish(msg)

    def publish_estop(self):
        msg = ActuatorCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.estop = True
        msg.throttle = 0.0
        msg.brake = 1.0
        msg.steer = 0.0
        msg.mode = 'N'
        self._cmd_pub.publish(msg)


def create_app(node: WebUINode) -> FastAPI:
    """Build the FastAPI application wired to the ROS2 node."""
    app = FastAPI()
    static_dir = os.path.join(
        get_package_share_directory('avros_webui'), 'static'
    )

    active_controller = None
    controller_lock = threading.Lock()

    @app.get('/')
    async def root():
        return FileResponse(os.path.join(static_dir, 'index.html'))

    @app.websocket('/ws')
    async def websocket_endpoint(websocket: WebSocket):
        nonlocal active_controller

        with controller_lock:
            if active_controller is not None:
                await websocket.accept()
                await websocket.send_json(
                    {'error': 'Another controller is connected'}
                )
                await websocket.close(code=1008)
                return

        await websocket.accept()
        with controller_lock:
            active_controller = websocket

        estop = False
        mode = 'N'

        try:
            while True:
                data = await websocket.receive_json()
                msg_type = data.get('type')

                if msg_type == 'control':
                    x = float(data.get('x', 0))
                    y = float(data.get('y', 0))
                    throttle = max(0.0, y)
                    brake = max(0.0, -y)
                    node.publish_command(throttle, brake, x, mode, estop)

                elif msg_type == 'estop':
                    estop = data.get('value', False)
                    if estop:
                        node.publish_estop()
                    else:
                        node.publish_command(0.0, 0.0, 0.0, mode, False)

                elif msg_type == 'mode':
                    new_mode = data.get('value', 'N')
                    if new_mode in ('N', 'D', 'S', 'R'):
                        mode = new_mode

                await websocket.send_json(node.get_state())

        except WebSocketDisconnect:
            pass
        finally:
            with controller_lock:
                active_controller = None
            node.publish_estop()
            node.get_logger().warn('WebSocket disconnected — e-stop sent')

    @app.middleware('http')
    async def no_cache(request, call_next):
        response = await call_next(request)
        if request.url.path.endswith(('.js', '.css')):
            response.headers['Cache-Control'] = 'no-store'
        return response

    app.mount(
        '/static',
        StaticFiles(directory=static_dir, follow_symlink=True),
        name='static',
    )

    return app


def main(args=None):
    rclpy.init(args=args)
    node = WebUINode()

    # Spin ROS2 in a background daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = create_app(node)

    ssl_kwargs = {}
    if node.ssl_certfile and node.ssl_keyfile:
        ssl_kwargs['ssl_certfile'] = node.ssl_certfile
        ssl_kwargs['ssl_keyfile'] = node.ssl_keyfile

    try:
        uvicorn.run(
            app, host='0.0.0.0', port=node.web_port,
            log_level='info', **ssl_kwargs
        )
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
