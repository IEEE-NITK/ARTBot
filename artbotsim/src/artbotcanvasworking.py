import rclpy
from rclpy.node import Node
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
        self.total_segment_points = 100

        self.display_image = np.ones((720, 1280, 3)) * 255

        cv2.namedWindow('ARTBOT Canvas', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('ARTBOT Canvas', self.mouse_callback)

    def update_display(self):
        mouse_coordinates_text = f'Current Mouse Coordinates: {self.mouse_coordinates[0]}, {self.mouse_coordinates[1]}'

        self.display_image = np.ones((720, 1280, 3)) * 255
        path_length = sum(np.sqrt((self.path[i][0] - self.path[i - 1][0]) ** 2 + (self.path[i][1] - self.path[i - 1][1]) ** 2) 
                          for i in range(1, len(self.path)))

        # for i in range(1, len(self.path)):
        #     cv2.line(self.display_image, self.path[i - 1], self.path[i], (0, 0, 255), 2)
        # print('Path:',self.path)
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

        collect_points_text = f'All collected points: {self.collect_points}'
        cv2.putText(self.display_image, collect_points_text,
                    (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.imshow('ARTBOT Canvas', self.display_image)
        # cv2.waitKey(1)
        cv2.waitKey(2500)
        self.collect_points = []
        self.path = []

    def close(self):
        key = cv2.waitKey(1) & 0xff
        if key==ord('q'):
            cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        # self.path = []
        if event == cv2.EVENT_LBUTTONDOWN:
            # print('Mouse clicked with path:',self.path)
            self.mouse_dragging = True
            self.path.append((x, y))

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.mouse_dragging:
                self.mouse_coordinates = (x, y)
                self.path.append((x, y))

        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_dragging = False
            # print('Mouse lifted with path:',self.path)

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.divide_path()
            self.path = []
            self.close()
            
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
