import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from hri import HRIListener
from rclpy.node import Node
from hri_msgs.msg import Expression


class PersonTargetNode(Node):

    def __init__(self):
        super().__init__('person_target_node')
        self.hri_listener = HRIListener("target_person")
        self.timer = self.create_timer(0.5, self.target_person)
        self.first_person = ""

    def target_person(self):
        """
        Stores and reacts to the first detected person.

        This method is called periodically. It iterates through tracked
        people trying to match with the first detected person.
        """

        for person_id, person in self.hri_listener.persons.items():
            print(person_id)


def main(args=None):
    rclpy.init(args=args)
    person_target_node = PersonTargetNode()

    try:
        rclpy.spin(person_target_node)
    except KeyboardInterrupt:
        pass
    finally:
        person_target_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
