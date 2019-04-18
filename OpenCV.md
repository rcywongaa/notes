# OpenCV

- Quickly create matrix of certain color

      color_hsv = np.zeros((100, 100, 3), dtype=np.uint8)
      color_hsv[:,:,0] = color_H
      color_hsv[:,:,1] = color_S
      color_hsv[:,:,2] = color_V

- Matrix are indexed [row, col], points are indexed [x, y]

- Erode / Dilate

    morph_kernel = np.ones((5, 5), np.int8)
    morphed_output = cv2.dilate(input, morph_kernel, iterations = 4)
    morphed_output = cv2.erode(morphed_output, morph_kernel, iterations = 6)

- Bounding box of bunch of points

    bounding_box_bgr = np.zeros(processed_hsv.shape, dtype=np.uint8)
    area = 0.0

    if USE_EDGES:
        edges = cv2.Canny(morphed_bin, CANNY_LOWER, CANNY_UPPER)

        edge_points_list = []

        for row in range(edges.shape[0]):
            for col in range(edges.shape[1]):
                # https://stackoverflow.com/questions/25642532/opencv-pointx-y-represent-column-row-or-row-column
                if edges[row, col]:
                    x = col
                    y = row
                    edge_points_list.append((x, y))

        points = np.array(edge_points_list)

    else:
        points = cv2.findNonZero(morphed_bin)

    if np.any(points):
        if USE_AXIS_ALIGNED_BOUNDING_BOX:
            box_x, box_y, box_width, box_height = cv2.boundingRect(points)
            box_corner_points = np.array([(box_x, box_y), (box_x, box_y + box_height), (box_x + box_width, box_y + box_height), (box_x + box_width, box_y)])
            area = cv2.contourArea(box_corner_points)
        else:
            rect = cv2.minAreaRect(points)
            box_corner_points = cv2.boxPoints(rect)
            box_corner_points = np.int0(box_corner_points)
            area = cv2.contourArea(box_corner_points)

        cv2.fillConvexPoly(bounding_box_bgr, box_corner_points, (255, 255, 255))
