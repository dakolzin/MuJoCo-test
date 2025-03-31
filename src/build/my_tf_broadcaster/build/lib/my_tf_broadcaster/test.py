#test.py

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped

"""
Суть работы скрипта:
- Инициализирует узел ROS2 для публикации TF-преобразований.
- Создает статический транслятор (для потенциальных статических преобразований) и динамический транслятор.
- Подписывается на топик 'object_pose' (PoseStamped) для получения позиции объекта относительно камеры.
- При получении сообщения преобразует позицию и ориентацию объекта в TF-сообщение, где "cam_frame" – родительский фрейм, а "object_link" – фрейм объекта.
- Публикует динамическое TF-преобразование, позволяя другим узлам ROS2 использовать эти данные для локализации объекта.
"""

class AllStaticTFPublisher(Node):
    def __init__(self):
        super().__init__('all_static_tf_publisher')
        # Публикуем статические преобразования
        self.static_br = tf2_ros.StaticTransformBroadcaster(self)

        # Инициализируем динамический транслятор для объекта относительно камеры
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # Подписчик на топик с позицией объекта относительно камеры (PoseStamped)
        self.subscription = self.create_subscription(
            PoseStamped,
            'object_pose',
            self.object_pose_callback,
            10
        )
        self.get_logger().info("Узел запущен. Ожидание сообщений с позицией объекта...")

    def object_pose_callback(self, msg: PoseStamped):
        # Создаем TF сообщение для динамического объекта относительно камеры (cam_frame)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "cam_frame"  # родительский фрейм (камера)
        t.child_frame_id = "object_link"       # фрейм объекта
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info("Опубликован динамический TF для объекта относительно камеры")

def main(args=None):
    rclpy.init(args=args)
    node = AllStaticTFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
