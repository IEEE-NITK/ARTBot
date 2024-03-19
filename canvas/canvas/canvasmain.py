import rclpy
from rclpy.node import Node
from artbotsim.msg import Target
import cv2
import numpy as np

class artbotcanvas(Node):
    def __init__(self):
        super().__init__('artbotcanvas')
        self.create_timer(0.001, self.update_display)
        self.mouse_coordinates = (0, 0)
        self.mouse_dragging = False
        self.path = []
        self.collect_points = []
        self.total_segment_points = 30
        self.target_publisher = self.create_publisher(Target, 'target', 10)

        self.display_image = np.ones((720, 1280, 3)) * 255

        cv2.namedWindow('ARTBOT Canvas', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('ARTBOT Canvas', self.mouse_callback)

        # Create a publisher for the divided points
        # self.divided_points_pub = self.create_publisher(PointCloud2, 'divided_points', 10)

    def update_display(self):
        mouse_coordinates_text = f'Current Mouse Coordinates: {self.mouse_coordinates[0]}, {self.mouse_coordinates[1]}'

        self.display_image = np.ones((720, 1280, 3)) * 255
        path_length = sum(np.sqrt((self.path[i][0] - self.path[i - 1][0]) ** 2 + (self.path[i][1] - self.path[i - 1][1]) ** 2) 
                          for i in range(1, len(self.path)))

        def draw_line(self, i):
            cv2.line(self.display_image, self.path[i - 1], self.path[i], (0, 0, 255), 2)
            return self.display_image
        
        for i in range(1, len(self.path)):
            draw_line(self, i)

        path_length_text = f'Path Length: {round(path_length, 3)} px'

        cv2.putText(self.display_image, mouse_coordinates_text,
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(self.display_image, path_length_text,
                    (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

        cv2.imshow('ARTBOT Canvas', self.display_image)
        cv2.waitKey(1)

    def divide_path(self):
        path_length = 0
        for i in range(1, len(self.path)):
            path_length += np.sqrt((self.path[i][0] - self.path[i - 1][0]) ** 2 + (self.path[i][1] - self.path[i - 1][1]) ** 2)

        segment_length = path_length / self.total_segment_points
        running_length = 0
        for i in range(1, len(self.path)):
            segment = np.sqrt((self.path[i][0] - self.path[i - 1][0]) ** 2 + (self.path[i][1] - self.path[i - 1][1]) ** 2)
            while running_length + segment > segment_length:
                ratio = (segment_length - running_length) / segment
                x = round(self.path[i - 1][0] * (1 - ratio) + self.path[i][0] * ratio)
                y = round(self.path[i - 1][1] * (1 - ratio) + self.path[i][1] * ratio)
                self.collect_points.append((x, y))
                cv2.circle(self.display_image, (x, y), 5, (255, 0, 0), -1)
                cv2.putText(self.display_image, str(len(self.collect_points)),
                            (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
                running_length -= segment_length
            running_length += segment

        # Display points on the canvas
        collect_points_text = f'All collected points: {self.collect_points}'
        cv2.putText(self.display_image, collect_points_text,
                    (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow('ARTBOT Canvas', self.display_image)
        cv2.waitKey(2500)

        target_msg = Target()
        
        target_msg.target1_x,target_msg.target1_y=self.collect_points[0][0],self.collect_points[0][1]
        target_msg.target2_x,target_msg.target2_y=self.collect_points[1][0],self.collect_points[1][1]
        target_msg.target3_x,target_msg.target3_y=self.collect_points[2][0],self.collect_points[2][1]
        target_msg.target4_x,target_msg.target4_y=self.collect_points[3][0],self.collect_points[3][1]
        target_msg.target5_x,target_msg.target5_y=self.collect_points[4][0],self.collect_points[4][1]
        target_msg.target6_x,target_msg.target6_y=self.collect_points[5][0],self.collect_points[5][1]
        target_msg.target7_x,target_msg.target7_y=self.collect_points[6][0],self.collect_points[6][1]
        target_msg.target8_x,target_msg.target8_y=self.collect_points[7][0],self.collect_points[7][1]
        target_msg.target9_x,target_msg.target9_y=self.collect_points[8][0],self.collect_points[8][1]
        target_msg.target10_x,target_msg.target10_y=self.collect_points[9][0],self.collect_points[9][1]
        target_msg.target11_x,target_msg.target11_y=self.collect_points[10][0],self.collect_points[10][1]
        target_msg.target12_x,target_msg.target12_y=self.collect_points[11][0],self.collect_points[11][1]
        target_msg.target13_x,target_msg.target13_y=self.collect_points[12][0],self.collect_points[12][1]
        target_msg.target14_x,target_msg.target14_y=self.collect_points[13][0],self.collect_points[13][1]
        target_msg.target15_x,target_msg.target15_y=self.collect_points[14][0],self.collect_points[14][1]
        target_msg.target16_x,target_msg.target16_y=self.collect_points[15][0],self.collect_points[15][1]
        target_msg.target17_x,target_msg.target17_y=self.collect_points[16][0],self.collect_points[16][1]
        target_msg.target18_x,target_msg.target18_y=self.collect_points[17][0],self.collect_points[17][1]
        target_msg.target19_x,target_msg.target19_y=self.collect_points[18][0],self.collect_points[18][1]
        target_msg.target20_x,target_msg.target20_y=self.collect_points[19][0],self.collect_points[19][1]
        target_msg.target21_x,target_msg.target21_y=self.collect_points[20][0],self.collect_points[20][1]
        target_msg.target22_x,target_msg.target22_y=self.collect_points[21][0],self.collect_points[21][1]
        target_msg.target23_x,target_msg.target23_y=self.collect_points[22][0],self.collect_points[22][1]
        target_msg.target24_x,target_msg.target24_y=self.collect_points[23][0],self.collect_points[23][1]
        target_msg.target25_x,target_msg.target25_y=self.collect_points[24][0],self.collect_points[24][1]
        target_msg.target26_x,target_msg.target26_y=self.collect_points[25][0],self.collect_points[25][1]
        target_msg.target27_x,target_msg.target27_y=self.collect_points[26][0],self.collect_points[26][1]
        target_msg.target28_x,target_msg.target28_y=self.collect_points[27][0],self.collect_points[27][1]
        target_msg.target29_x,target_msg.target29_y=self.collect_points[28][0],self.collect_points[28][1]
        target_msg.target30_x,target_msg.target30_y=self.collect_points[29][0],self.collect_points[29][1]

        self.target_publisher.publish(target_msg)

        # Publish the list of divided points
        # msg = PointCloud2()
        # Fill in your point cloud message with the collected points
        # Assuming each point is represented as a geometry_msgs/Point32 message
        # Example: msg.points = [Point32(x=pt[0], y=pt[1], z=0) for pt in self.collect_points]
        # self.divided_points_pub.publish(msg)

        self.collect_points = []
        self.path = []



    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_dragging = True
            self.path.append((x, y))

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.mouse_dragging:
                self.mouse_coordinates = (x, y)
                self.path.append((x, y))

        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_dragging = False

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.divide_path()
            self.path = []

        # elif event == cv2.EVENT_KEYDOWN:  # Check if a key is pressed
        #     if cv2.waitKey(1) & 0xFF == ord('q'):  # Check if the pressed key is 'q'
        #         cv2.destroyAllWindows()  # Close the window

        return self.path

    
def main(args=None):
    rclpy.init(args=args)
    node = artbotcanvas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
