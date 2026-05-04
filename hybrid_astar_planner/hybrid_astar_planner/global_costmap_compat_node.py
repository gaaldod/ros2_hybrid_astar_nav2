from __future__ import annotations

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import ClearEntireCostmap


class GlobalCostmapCompatNode(Node):
    """
    Compatibility shim for Nav2 BT trees when planner_server is omitted.

    The navigate-through-poses tree in Humble may require the global clear-costmap
    service during BT navigator configuration. In our custom-planner wiring we do
    not launch planner_server, so we provide a lightweight no-op service endpoint
    with the expected name to let BT bringup complete.
    """

    def __init__(self) -> None:
        super().__init__("global_costmap_compat_node")
        self._srv = self.create_service(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            self._on_clear_entirely_global_costmap,
        )
        self.get_logger().info(
            "GlobalCostmapCompatNode active: serving /global_costmap/clear_entirely_global_costmap"
        )

    def _on_clear_entirely_global_costmap(
        self, request: ClearEntireCostmap.Request, response: ClearEntireCostmap.Response
    ) -> ClearEntireCostmap.Response:
        _ = request
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GlobalCostmapCompatNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
