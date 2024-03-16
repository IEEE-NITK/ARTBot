# import rclpy
# from rclpy.node import Node
# import cv2
# import numpy as np

# class artbotcanvas(Node):
#     def __init__(self):
#         super().__init__('artbotcanvas')
#         self.create_timer(0.001, self.update_display)
#         self.mouse_coordinates = (0, 0)
#         self.mouse_dragging = False
#         self.path = []
#         self.global_path_list = []
#         self.collect_points = []
#         self.total_segment_points = 100

#         self.display_image = np.ones((720, 1280, 3)) * 255

#         cv2.namedWindow('ARTBOT Canvas', cv2.WINDOW_NORMAL)
#         cv2.setMouseCallback('ARTBOT Canvas', self.mouse_callback)

#     def update_display(self):
#         mouse_coordinates_text = f'Current Mouse Coordinates: {self.mouse_coordinates[0]}, {self.mouse_coordinates[1]}'

#         self.display_image = np.ones((720, 1280, 3)) * 255
#         path_length = sum(np.sqrt((self.path[i][0] - self.path[i - 1][0]) ** 2 + (self.path[i][1] - self.path[i - 1][1]) ** 2) 
#                           for i in range(1, len(self.path)))

#         # for i in range(1, len(self.path)):
#         #     cv2.line(self.display_image, self.path[i - 1], self.path[i], (0, 0, 255), 2)
#         # print('Path:',self.path)
#         def draw_line(self, i):
#             cv2.line(self.display_image, self.path[i - 1], self.path[i], (0, 0, 255), 2)
#             return self.display_image
        
#         for i in range(1, len(self.path)):
#             draw_line(self, i)

#         path_length_text = f'Path Length: {round(path_length, 3)} px'

#         cv2.putText(self.display_image, mouse_coordinates_text,
#                     (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
#         cv2.putText(self.display_image, path_length_text,
#                     (5, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)

#         cv2.imshow('ARTBOT Canvas', self.display_image)
#         cv2.waitKey(1)

#     def divide_path(self):
#         path_length = 0
#         for i in range(1, len(self.path)):
#             path_length += np.sqrt((self.path[i][0] - self.path[i - 1][0]) ** 2 + (self.path[i][1] - self.path[i - 1][1]) ** 2)

#         segment_length = path_length / self.total_segment_points
#         running_length = 0
#         for i in range(1, len(self.path)):
#             segment = np.sqrt((self.path[i][0] - self.path[i - 1][0]) ** 2 + (self.path[i][1] - self.path[i - 1][1]) ** 2)
#             while running_length + segment > segment_length:
#                 ratio = (segment_length - running_length) / segment
#                 x = round(self.path[i - 1][0] * (1 - ratio) + self.path[i][0] * ratio)
#                 y = round(self.path[i - 1][1] * (1 - ratio) + self.path[i][1] * ratio)
#                 self.collect_points.append((x, y))
#                 cv2.circle(self.display_image, (x, y), 5, (255, 0, 0), -1)
#                 cv2.putText(self.display_image, str(len(self.collect_points)),
#                     (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
#                 running_length -= segment_length
#             running_length += segment

#         collect_points_text = f'All collected points: {self.collect_points}'
#         cv2.putText(self.display_image, collect_points_text,
#                     (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2, cv2.LINE_AA)
#         cv2.imshow('ARTBOT Canvas', self.display_image)
#         # cv2.waitKey(1)
#         cv2.waitKey(2500)
#         self.collect_points = []
#         self.path = []

#     def close(self):
#         key = cv2.waitKey(1) & 0xff
#         if key==ord('q'):
#             cv2.destroyAllWindows()

#     # def create_list(self, num_points):
#     #     new_list = []
#     #     for _ in range(num_points):
#     #         x = 100 
#     #         y = 1 
#     #         new_list.append([x, y])
#     #     self.global_list.append(new_list)

#     # def mouse_callback(self, event, x, y, flags, param):
#     #     if event == cv2.EVENT_LBUTTONDOWN:
#     #         # print('Mouse clicked with path:',self.path)
#     #         self.mouse_dragging = True
#     #         self.path.append((x, y))

#     #     elif event == cv2.EVENT_MOUSEMOVE:
#     #         if self.mouse_dragging:
#     #             self.mouse_coordinates = (x, y)
#     #             self.path.append((x, y))

#     #     elif event == cv2.EVENT_LBUTTONUP:
#     #         self.mouse_dragging = False
#     #         self.close()
#     #         # print('Mouse lifted with path:',self.path)

#     #     elif event == cv2.EVENT_RBUTTONDOWN:
#     #         self.divide_path()
#     #         self.path = []
#     #         self.close()
            
#     #     return self.path
#     def mouse_callback(self, event, x, y, flags, param):
#         if event == cv2.EVENT_LBUTTONDOWN:
#             self.mouse_dragging = True
#             self.current_path = []  # Create a new list for the current path
#             self.current_path.append((x, y))

#         elif event == cv2.EVENT_MOUSEMOVE:
#             if self.mouse_dragging:
#                 self.current_path.append((x, y))

#         elif event == cv2.EVENT_LBUTTONUP:
#             self.mouse_dragging = False
#             self.global_path_list.append(self.current_path)  # Append to global list
#             self.current_path = []  # Clear temporary path list
#             self.close()
#             # Calculate distance for the current path (optional)
#             total_distance = 0
#             for i in range(1, len(self.current_path)):
#                 total_distance += self.calculate_distance(self.current_path[i - 1], self.current_path[i])
#             print(f"Path Distance: {total_distance}")

#         return self.path  # Not used anymore, can be removed


    
# def main(args=None):
#     rclpy.init(args=args)
#     node = artbotcanvas()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         cv2.destroyAllWindows()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# # import random
# # import math

# # class PointList:
# #     def __init__(self):
# #         self.global_list = []

# #     def create_list(self, num_points):
# #         new_list = []
# #         for _ in range(num_points):
# #             x = random.randint(0, 100)  # Random x coordinate
# #             y = random.randint(0, 100)  # Random y coordinate
# #             new_list.append((x, y))  # Store coordinates as tuple
# #         self.global_list.append(new_list)

# #     def add_list(self, new_list):
# #         self.global_list.append(new_list)

# #     def display(self):
# #         for i, point_list in enumerate(self.global_list):
# #             print(f"List {i+1}: {point_list}")

# #     def calculate_distance(self, point1, point2):
# #         x1, y1 = point1
# #         x2, y2 = point2
# #         return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# #     def total_distance(self):
# #         total_dist = 0
# #         for point_list in self.global_list:
# #             for i in range(len(point_list) - 1):
# #                 total_dist += self.calculate_distance(point_list[i], point_list[i+1])
# #         return total_dist

# #     def extract_points(self, factor):
# #         extracted_points = []
# #         total_dist = self.total_distance()
# #         interval_length = total_dist / factor
# #         current_distance = 0

# #         for point_list in self.global_list:
# #             for i in range(len(point_list) - 1):
# #                 dist = self.calculate_distance(point_list[i], point_list[i+1])
# #                 while current_distance + dist >= interval_length:
# #                     remaining_dist = interval_length - current_distance
# #                     ratio = remaining_dist / dist
# #                     new_point = (
# #                         point_list[i][0] + ratio * (point_list[i+1][0] - point_list[i][0]),
# #                         point_list[i][1] + ratio * (point_list[i+1][1] - point_list[i][1])
# #                     )
# #                     extracted_points.append(new_point)
# #                     dist -= remaining_dist
# #                     current_distance = 0
# #                 current_distance += dist

# #         return extracted_points

# # # Example usage:
# # point_manager = PointList()
# # point_manager.create_list(5)  # Create a new list with 5 random points
# # point_manager.create_list(3)  # Create another list with 3 random points
# # point_manager.display()  # Display all lists of points
# # print(point_manager.global_list)
# # print(len(point_manager.global_list))
# # print("Total distance:", point_manager.total_distance())  # Calculate and print the total distance

# # factor = 10  # Define the factor to divide the total length
# # extracted_points = point_manager.extract_points(factor)
# # print(f"Points extracted at every {factor} units:", extracted_points)

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
