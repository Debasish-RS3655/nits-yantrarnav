import cv2
import numpy as np
import time
import math

def main():
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("Error: Could not open the camera.")
        return

    # --- Parameters ---
    center_tolerance = 20     # Allowed horizontal deviation in pixels for "centered"
    angle_threshold = 20      # Degrees from vertical to consider it a corner
    min_area = 100            # Minimum area to consider a valid tape contour
    flash_duration = 0.5      # Seconds for flashing corner/off-center warnings

    # For flashing text
    last_flash_time = time.time()
    show_warning = True

    # For storing centroid history while off-center
    centroid_history = []

    print("\n--- Starting Single-Tape Line-Follow & Corner-Detection Script ---\n")

    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error: Could not read frame from the camera.")
            break

        height, width = frame.shape[:2]
        horizontal_center = width // 2

        # --- Convert to HSV and Segment Yellow ---
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Adjust these HSV ranges if necessary
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        yellow_mask = cv2.inRange(hsv_frame, yellow_lower, yellow_upper)

        # Morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        # --- Find Contours ---
        contours, _ = cv2.findContours(yellow_mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        status_message = ""
        command_message = ""
        detailed_info = ""
        box_color = (0, 255, 0)

        if len(contours) == 0:
            # No contour found
            status_message = "Error: No yellow tape detected"
        else:
            # Take the largest contour
            main_contour = max(contours, key=cv2.contourArea)
            region_area = cv2.contourArea(main_contour)

            if region_area < min_area:
                status_message = "Error: Yellow tape too small"
            else:
                # Compute bounding box & centroid
                x, y, w, h = cv2.boundingRect(main_contour)
                M = cv2.moments(main_contour)
                if M["m00"] != 0:
                    centroid_x = int(M["m10"] / M["m00"])
                    centroid_y = int(M["m01"] / M["m00"])
                else:
                    centroid_x = x + w // 2
                    centroid_y = y + h // 2

                # Compute horizontal deviation from the frame center
                deviation = horizontal_center - centroid_x

                # Orientation using cv2.fitLine
                line = cv2.fitLine(main_contour, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy = line[0][0], line[1][0]
                angle_deg = math.degrees(math.atan2(vy, vx))

                # Corner logic: For a nearly vertical tape, the angle should be near 90°.
                angle_deviation = abs(abs(angle_deg) - 90)

                if angle_deviation > angle_threshold:
                    # Corner detected
                    status_message = "Corner reached!"
                    box_color = (0, 0, 255)
                elif abs(deviation) > center_tolerance:
                    # Tape off-center
                    status_message = "Error: Tape off center!"
                    box_color = (0, 0, 255)
                else:
                    # Tape centered
                    status_message = "Tape centered"
                    box_color = (0, 255, 0)

                # Forward command logic:
                # If tape is off-center, then issue a forward command.
                if "off center" in status_message:
                    command_message = "Forward Command"
                    # Store and "send" the centroid while off-center
                    centroid_history.append((centroid_x, centroid_y))
                    print(f"[SEND] Centroid Coordinates Sent: ({centroid_x}, {centroid_y})")
                else:
                    command_message = "No Forward Command"

                # Draw bounding box and centroid on the frame
                cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                cv2.circle(frame, (centroid_x, centroid_y), 5, box_color, -1)
                # Draw a reference vertical line at the horizontal center
                cv2.line(frame, (horizontal_center, 0), (horizontal_center, height), (255, 0, 0), 2)

                # Detailed info string to display and log
                detailed_info = (
                    f"Centroid=({centroid_x},{centroid_y})  "
                    f"Deviation={deviation}  "
                    f"Angle={angle_deg:.1f}°  "
                    f"AngleDev={angle_deviation:.1f}°"
                )

                # Display deviation & angle on the frame
                cv2.putText(frame, f"Deviation: {deviation}", (10, height - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(frame, f"Angle: {angle_deg:.1f}", (10, height - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # --- Flashing / Display Logic for Warning Messages ---
        if status_message.startswith("Error") or "Corner" in status_message:
            current_time = time.time()
            if current_time - last_flash_time > flash_duration:
                show_warning = not show_warning
                last_flash_time = current_time
            if show_warning:
                cv2.putText(frame, status_message, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
        else:
            if status_message:
                color = (0, 255, 0) if "centered" in status_message else (0, 255, 255)
                cv2.putText(frame, status_message, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # Display forward command message
        if command_message:
            cv2.putText(frame, command_message, (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Display detailed information
        if detailed_info:
            cv2.putText(frame, detailed_info, (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # --- Print Logs to Terminal ---
        print("----- Frame Log -----")
        print(f"[STATUS] {status_message}")
        print(f"[COMMAND] {command_message}")
        print(f"[DETAIL] {detailed_info}")
        print("---------------------\n")

        # Display the annotated frame
        cv2.imshow("Frame", frame)

        # Exit loop when 'q' is pressed.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

    # Print out stored centroid history
    print("\n--- Centroid History (while tape was off-center) ---")
    if centroid_history:
        for idx, (cx, cy) in enumerate(centroid_history, start=1):
            print(f"{idx}: (x={cx}, y={cy})")
    else:
        print("No off-center centroids recorded.")
    print("--- End of Program ---")

if __name__ == "__main__":
    main()
