from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelState, LinkState
from gazebo_msgs.srv import SetModelState, GetModelState, SetLinkState


class MovingObstaclesNode(Node):
    """Move a single center box obstacle in Gazebo Classic (or log if unavailable).

    This node uses parameters prefixed with `moving_obstacle_` so it does not
    interfere with robot tuning parameters in other launches.
    """

    def __init__(self) -> None:
        super().__init__("moving_obstacles_node")

        # Public parameters for the moving obstacle (distinct from robot params)
        self.declare_parameter("moving_obstacle_tick_hz", 20.0)
        self.declare_parameter("moving_obstacle_z_height", 0.35)
        self.declare_parameter("moving_obstacle_name", "moving_center_box")
        self.declare_parameter("moving_obstacle_amp_y", 8.0)
        # Frequency in radians/sec multiplier used in y = amp * cos(freq * t)
        self.declare_parameter("moving_obstacle_freq", 0.25)

        self._name = str(self.get_parameter("moving_obstacle_name").value)
        self._z = float(self.get_parameter("moving_obstacle_z_height").value)
        hz = max(1.0, float(self.get_parameter("moving_obstacle_tick_hz").value))
        self._amp_y = float(self.get_parameter("moving_obstacle_amp_y").value)
        self._freq = float(self.get_parameter("moving_obstacle_freq").value)

        # Service clients for Gazebo Classic (/gazebo/*). If running modern Gazebo
        # (ros_gz_sim) these will likely not exist; we probe at startup and log.
        self._client = self.create_client(SetModelState, "/gazebo/set_model_state")
        self._get_client = self.create_client(GetModelState, "/gazebo/get_model_state")
        self._set_link_client = self.create_client(SetLinkState, "/gazebo/set_link_state")

        # None => not yet probed; True/False => model exists or not
        self._model_exists: Optional[bool] = None

        # internal timer state
        self._t = 0.0
        self._dt = 1.0 / hz
        self._timer = self.create_timer(self._dt, self._tick)

        # Probe for presence of classic Gazebo services once at startup
        self._probe_timer = self.create_timer(1.0, self._probe_services)

        self.get_logger().info(
            f"moving_obstacles_node started for model='{self._name}' amp_y={self._amp_y} freq={self._freq} hz={hz}"
        )

    def _probe_services(self) -> None:
        # Called by a short-lived timer to check whether classic Gazebo services
        # are available in the environment. Cancel timer after first run.
        try:
            ready = self._get_client.service_is_ready()
            if ready:
                self.get_logger().info(
                    "Detected Gazebo Classic service /gazebo/get_model_state; moving_obstacles_node will attempt model updates."
                )
            else:
                self.get_logger().warning(
                    "Gazebo Classic services (/gazebo/*) not detected. If running modern Gazebo (ros_gz_sim), consider using a world-side actor or adapting this node to the ros_gz bridge."
                )
            # mark probe complete
            if self._probe_timer is not None:
                self._probe_timer.cancel()
                self._probe_timer = None
        except Exception as ex:
            self.get_logger().warning(f"Error while probing Gazebo services: {ex}")
            if self._probe_timer is not None:
                self._probe_timer.cancel()
                self._probe_timer = None

    def _tick(self) -> None:
        # If SetModelState client isn't ready, skip this tick.
        if not self._client.service_is_ready():
            return

        self._t += self._dt

        # On first successful contact with gazebo get_model_state, verify the model exists.
        if self._model_exists is None and self._get_client.service_is_ready():
            try:
                req = GetModelState.Request()
                req.model_name = self._name
                req.relative_entity_name = ""
                fut = self._get_client.call_async(req)
                fut.add_done_callback(self._on_get_model_state)
            except Exception as ex:
                self.get_logger().warning(f"get_model_state call failed: {ex}")
                self._model_exists = False

        # Oscillate along center line using configured params
        y = self._amp_y * math.cos(self._freq * self._t)
        x = 0.0
        # Try to set model pose; if it fails we'll attempt link-level set as a fallback.
        self._set_model(self._name, x, y)

    def _on_get_model_state(self, fut) -> None:
        try:
            res = fut.result()
            if hasattr(res, "success") and res.success:
                self.get_logger().info(f"{self._name} exists in Gazebo; enabling motion updates")
                self._model_exists = True
            else:
                status = getattr(res, "status_message", "")
                self.get_logger().warning(f"{self._name} not found in Gazebo: {status}")
                self._model_exists = False
        except Exception as ex:
            self.get_logger().warning(f"get_model_state future failed: {ex}")
            self._model_exists = False

    def _on_set_model_response(self, fut, name: str, x: float, y: float) -> None:
        try:
            res = fut.result()
            if hasattr(res, "success") and res.success:
                return
            # Fall back to set_link_state if available
            self.get_logger().warning(
                f"SetModelState failed for '{name}' response={res}; trying SetLinkState fallback"
            )
            if self._set_link_client.service_is_ready():
                try:
                    link_req = SetLinkState.Request()
                    link_req.link_state = LinkState()
                    # link name assumed to be '<model>::base_link' or use base_link
                    link_req.link_state.link_name = f"{name}::base_link"
                    link_req.link_state.pose.position.x = x
                    link_req.link_state.pose.position.y = y
                    link_req.link_state.pose.position.z = self._z
                    link_req.link_state.pose.orientation.w = 1.0
                    link_req.link_state.reference_frame = "world"
                    lfut = self._set_link_client.call_async(link_req)
                    lfut.add_done_callback(lambda f: self._on_set_link_response(f, name))
                except Exception as ex2:
                    self.get_logger().error(f"SetLinkState call failed: {ex2}")
        except Exception as ex:
            self.get_logger().warning(f"set_model_state future failed: {ex}")

    def _on_set_link_response(self, fut, name: str) -> None:
        try:
            res = fut.result()
            if hasattr(res, "success") and res.success:
                self.get_logger().info(f"SetLinkState succeeded for {name}")
            else:
                self.get_logger().error(f"SetLinkState failed for {name}: {res}")
        except Exception as ex:
            self.get_logger().error(f"SetLinkState future handling exception: {ex}")

    def _set_model(self, name: str, x: float, y: float) -> None:
        req = SetModelState.Request()
        req.model_state = ModelState()
        req.model_state.model_name = name
        req.model_state.pose.position.x = x
        req.model_state.pose.position.y = y
        req.model_state.pose.position.z = self._z
        req.model_state.pose.orientation.w = 1.0
        req.model_state.reference_frame = "world"
        try:
            fut = self._client.call_async(req)
            fut.add_done_callback(lambda f: self._on_set_model_response(f, name, x, y))
        except Exception as ex:
            self.get_logger().warning(f"set_model_state call exception: {ex}")


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MovingObstaclesNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


