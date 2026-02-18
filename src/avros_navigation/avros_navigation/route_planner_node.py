"""Route planner node: OSMnx routes -> Nav2 waypoints.

Wraps AV2.1-API planning/navigator.py as a ROS2 node.

Subscribes:
  /gnss                     sensor_msgs/NavSatFix (current GPS position)

Publishes:
  /avros/route_waypoints    nav_msgs/Path (visualization)

Service:
  /avros/plan_route         avros_msgs/srv/PlanRoute

Action client:
  /navigate_through_poses   nav2_msgs/NavigateThroughPoses
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateThroughPoses
from robot_localization.srv import FromLL
from avros_msgs.srv import PlanRoute

try:
    import osmnx as ox
    import networkx as nx
    from pyproj import Transformer
    from shapely.geometry import LineString
    HAS_OSMNX = True
except ImportError:
    HAS_OSMNX = False


class RoutePlannerNode(Node):
    """ROS2 node for GPS route planning via OSMnx."""

    def __init__(self):
        super().__init__('route_planner_node')

        # Declare parameters
        self.declare_parameter('network_type', 'bike')
        self.declare_parameter('waypoint_spacing', 5.0)
        self.declare_parameter('destination_lat', 34.059270)
        self.declare_parameter('destination_lon', -117.820934)
        self.declare_parameter('place',
                               'California State Polytechnic University Pomona, '
                               'California, USA')

        self._network_type = self.get_parameter('network_type').value
        self._waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self._place = self.get_parameter('place').value

        # Current GPS position
        self._current_lat = None
        self._current_lon = None

        # OSMnx graph (lazy-loaded on first route request)
        self._graph = None
        self._crs = None
        self._transformer = None
        self._transformer_inv = None

        # Subscriber: current GPS (Xsens publishes on /gnss)
        self._gps_sub = self.create_subscription(
            NavSatFix, '/gnss', self._gps_callback, 10
        )

        # Publisher: route visualization
        self._waypoints_pub = self.create_publisher(
            Path, '/avros/route_waypoints', 10
        )

        # Service: plan route
        self._plan_srv = self.create_service(
            PlanRoute, '/avros/plan_route', self._plan_route_callback
        )

        # Action client: Nav2 navigate through poses
        self._nav_client = ActionClient(
            self, NavigateThroughPoses, '/navigate_through_poses'
        )

        # Service client: robot_localization GPS -> map frame conversion
        self._fromll_client = self.create_client(FromLL, '/fromLL')

        if not HAS_OSMNX:
            self.get_logger().warn(
                'osmnx not installed — route planning will fail. '
                'Install with: pip install osmnx networkx pyproj shapely'
            )

        self.get_logger().info('Route planner node started')

    def _gps_callback(self, msg: NavSatFix):
        """Store current GPS position."""
        self._current_lat = msg.latitude
        self._current_lon = msg.longitude

    def _ensure_graph(self):
        """Download OSMnx graph if not already loaded."""
        if self._graph is not None:
            return True
        if not HAS_OSMNX:
            return False

        self.get_logger().info(
            f'Downloading OSMnx network ({self._network_type})...'
        )
        try:
            self._graph = ox.graph_from_place(
                self._place,
                network_type=self._network_type,
                simplify=True,
            )
            self._graph = ox.project_graph(self._graph)
            self._crs = self._graph.graph['crs']
            self._transformer = Transformer.from_crs(
                'EPSG:4326', self._crs, always_xy=True
            )
            self._transformer_inv = Transformer.from_crs(
                self._crs, 'EPSG:4326', always_xy=True
            )
            self.get_logger().info(
                f'Graph ready: {len(self._graph.nodes)} nodes, '
                f'{len(self._graph.edges)} edges, CRS: {self._crs}'
            )
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to load OSMnx graph: {e}')
            return False

    def _plan_route_callback(self, request, response):
        """Service handler: plan route from current GPS to destination."""
        if self._current_lat is None:
            response.success = False
            response.message = 'No GPS fix available'
            response.distance_meters = 0.0
            response.num_waypoints = 0
            return response

        if not self._ensure_graph():
            response.success = False
            response.message = 'OSMnx graph not available'
            response.distance_meters = 0.0
            response.num_waypoints = 0
            return response

        dest_lat = request.destination_lat
        dest_lon = request.destination_lon

        self.get_logger().info(
            f'Planning route: ({self._current_lat:.6f}, {self._current_lon:.6f}) '
            f'-> ({dest_lat:.6f}, {dest_lon:.6f})'
        )

        try:
            # Convert GPS to projected coordinates
            start_x, start_y = self._transformer.transform(
                self._current_lon, self._current_lat
            )
            end_x, end_y = self._transformer.transform(dest_lon, dest_lat)

            # Find nearest graph nodes
            start_node = ox.distance.nearest_nodes(
                self._graph, start_x, start_y
            )
            end_node = ox.distance.nearest_nodes(
                self._graph, end_x, end_y
            )

            if start_node == end_node:
                response.success = False
                response.message = 'Start and end map to the same node'
                response.distance_meters = 0.0
                response.num_waypoints = 0
                return response

            # Shortest path
            route = nx.shortest_path(
                self._graph, start_node, end_node, weight='length'
            )
            distance = nx.shortest_path_length(
                self._graph, start_node, end_node, weight='length'
            )

            # Generate waypoints
            waypoints = self._generate_waypoints(route)

            if waypoints is None or len(waypoints) < 2:
                response.success = False
                response.message = 'Route too short'
                response.distance_meters = distance
                response.num_waypoints = 0
                return response

            # Publish visualization path
            path_msg = self._waypoints_to_path(waypoints)
            self._waypoints_pub.publish(path_msg)

            # Send to Nav2
            self._send_to_nav2(waypoints)

            response.success = True
            response.message = f'Route: {distance:.1f}m, {len(waypoints)} waypoints'
            response.distance_meters = distance
            response.num_waypoints = len(waypoints)

            self.get_logger().info(response.message)

        except nx.NetworkXNoPath:
            response.success = False
            response.message = 'No path found'
            response.distance_meters = 0.0
            response.num_waypoints = 0
        except Exception as e:
            response.success = False
            response.message = str(e)
            response.distance_meters = 0.0
            response.num_waypoints = 0
            self.get_logger().error(f'Route planning failed: {e}')

        return response

    def _generate_waypoints(self, route):
        """Generate evenly-spaced waypoints along route. Ported from navigator.py."""
        coords = []
        for i, node in enumerate(route):
            coords.append((
                self._graph.nodes[node]['x'],
                self._graph.nodes[node]['y'],
            ))
            if i < len(route) - 1:
                next_node = route[i + 1]
                data = self._graph.get_edge_data(node, next_node)
                if data:
                    for ed in data.values():
                        if 'geometry' in ed:
                            coords.extend(list(ed['geometry'].coords)[1:-1])
                            break

        if len(coords) < 2:
            return None

        path = LineString(coords)
        total_length = path.length
        num_waypoints = int(total_length // self._waypoint_spacing) + 1

        waypoints = []
        for i in range(num_waypoints):
            dist = min(i * self._waypoint_spacing, total_length)
            pt = path.interpolate(dist, normalized=False)
            # Convert UTM back to GPS
            lon, lat = self._transformer_inv.transform(pt.x, pt.y)
            waypoints.append((lat, lon))

        return waypoints

    def _gps_to_map(self, lat, lon):
        """Convert GPS (lat, lon) to map-frame (x, y) via /fromLL service."""
        if not self._fromll_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/fromLL service not available')
            return None

        req = FromLL.Request()
        req.ll_point = Point(x=lat, y=lon, z=0.0)
        future = self._fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is not None:
            result = future.result()
            return (result.map_point.x, result.map_point.y)
        return None

    def _waypoints_to_path(self, waypoints):
        """Convert waypoints to nav_msgs/Path for visualization."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        for lat, lon in waypoints:
            map_xy = self._gps_to_map(lat, lon)
            if map_xy is None:
                continue
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = map_xy[0]
            pose.pose.position.y = map_xy[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path

    def _send_to_nav2(self, waypoints):
        """Send waypoints to Nav2 via NavigateThroughPoses action."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('Nav2 action server not available')
            return

        goal = NavigateThroughPoses.Goal()

        for lat, lon in waypoints:
            map_xy = self._gps_to_map(lat, lon)
            if map_xy is None:
                continue
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = map_xy[0]
            pose.pose.position.y = map_xy[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            goal.poses.append(pose)

        if not goal.poses:
            self.get_logger().error('No waypoints converted to map frame')
            return

        self.get_logger().info(
            f'Sending {len(goal.poses)} waypoints to Nav2'
        )
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response)

    def _nav_goal_response(self, future):
        """Handle Nav2 goal acceptance."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected goal')
            return
        self.get_logger().info('Nav2 accepted goal — navigating')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        """Handle Nav2 navigation result."""
        result = future.result()
        self.get_logger().info(f'Navigation complete')


def main(args=None):
    rclpy.init(args=args)
    node = RoutePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
