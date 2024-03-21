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
        self.global_path_list = []  # Stores all drawn paths
        self.current_path = []  # Temporary list for current path
        self.equidistant_points = [] # List to store equidistant points

        self.display_image = np.ones((720, 1280, 3)) * 255

        cv2.namedWindow('ARTBOT Canvas', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('ARTBOT Canvas', self.mouse_callback)

    def update_display(self):
        mouse_coordinates_text = f'Current Mouse Coordinates: {self.mouse_coordinates[0]}, {self.mouse_coordinates[1]}'

        self.display_image = np.ones((720, 1280, 3)) * 255
        for path in self.global_path_list:
            for i in range(1, len(path)):
                cv2.line(self.display_image, path[i - 1], path[i], (0, 0, 255), 2)

        cv2.putText(self.display_image, mouse_coordinates_text,
                    (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

        cv2.imshow('ARTBOT Canvas', self.display_image)
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.close()

    def calculate_equidistant_points(self, num_points):
        total_distance = 0
        for path in self.global_path_list:
            for i in range(1, len(path)):
                total_distance += np.sqrt((path[i][0] - path[i - 1][0]) ** 2 + (path[i][1] - path[i - 1][1]) ** 2)

        # if total_distance == 0:
        #     print("Global path has zero distance. Cannot calculate equidistant points.")
        #     return []

        distance_between_points = total_distance / (num_points - 1)
        current_distance = 0

        self.equidistant_points = []  # Clear existing equidistant points

        for path in self.global_path_list:
            for i in range(len(path)):
                if i == len(path) - 1:
                    self.equidistant_points.append(path[i])
                    continue

                segment_distance = np.sqrt((path[i + 1][0] - path[i][0]) ** 2 + (path[i + 1][1] - path[i][1]) ** 2)

                while current_distance + segment_distance >= distance_between_points:
                    ratio = (distance_between_points - current_distance) / segment_distance
                    self.equidistant_points.append((path[i][0] + ratio * (path[i + 1][0] - path[i][0]),
                                               path[i][1] + ratio * (path[i + 1][1] - path[i][1])))
                    segment_distance = np.sqrt((path[i + 1][0] - self.equidistant_points[-1][0]) ** 2 +
                                               (path[i + 1][1] - self.equidistant_points[-1][1]) ** 2)
                    current_distance = 0

                current_distance += segment_distance
        
        # for point in self.equidistant_points:
        #     print(f"Point: ({point[0]}, {point[1]})")
        return self.equidistant_points

    def mark_and_draw_equidistant_points(self, color=(255, 0, 0)):
        points = self.calculate_equidistant_points(10)  # Number of points can be adjusted
        for point in points:
            point = (int(point[0]), int(point[1]))  # Convert to integer coordinates
            cv2.circle(self.display_image, point, 5, color, -1)  # Draw circle at each point
            cv2.putText(self.display_image, str(len(self.equidistant_points)), point, cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 2, cv2.LINE_AA)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_dragging = True
            self.current_path = []  # Clear temporary path list
            self.current_path.append((x, y))

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.mouse_dragging:
                self.current_path.append((x, y))
                self.mouse_coordinates = (x, y)  # Update current mouse coordinates

        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_dragging = False
            self.global_path_list.append(self.current_path)  # Append to global list
            self.current_path = []  # Clear temporary path list
            # print(f"Equidistant Points: {self.equidistant_points}")
            # print(f"Total Equidistant Points: {len(self.equidistant_points)}")

        elif event == cv2.EVENT_RBUTTONDOWN:  # Right mouse button click
            self.mark_and_draw_equidistant_points(color=(255, 0, 0))  # Mark equidistant points
            print(f"Equidistant Points: {self.equidistant_points}")
            print(f"Total Equidistant Points: {len(self.equidistant_points)}")

    def close(self):
        cv2.destroyAllWindows()
        self.destroy_node()
        rclpy.shutdown()

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
