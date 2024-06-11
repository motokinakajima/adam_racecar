import tkinter as tk
from PIL import Image, ImageTk
import cv2
import numpy as np

# Load the image
image = cv2.imread('blue_cone.jpg')

# Initialize the HSV range
l_h, l_s, l_v = 0, 0, 0
u_h, u_s, u_v = 179, 255, 255

# Function to update the image based on the slider values
def update_image(value):
    global l_h, l_s, l_v, u_h, u_s, u_v

    # Get the slider values
    l_h = l_h_slider.get()
    l_s = l_s_slider.get()
    l_v = l_v_slider.get()
    u_h = u_h_slider.get()
    u_s = u_s_slider.get()
    u_v = u_v_slider.get()

    # Define the lower and upper bounds of the HSV range
    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])

    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only the desired color
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw the contours on a black image
    contour_image = np.zeros_like(image)
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)

    # Convert the contour image to PIL format and display it
    contour_image = cv2.cvtColor(contour_image, cv2.COLOR_BGR2RGB)
    contour_photo = ImageTk.PhotoImage(image=Image.fromarray(contour_image))
    image_label.configure(image=contour_photo)
    image_label.image = contour_photo

# Create the Tkinter window
root = tk.Tk()
root.title("HSV Contour Detection")

# Create sliders for HSV range
l_h_slider = tk.Scale(root, from_=0, to=179, orient=tk.HORIZONTAL, label="Lower Hue", command=update_image)
l_s_slider = tk.Scale(root, from_=0, to=255, orient=tk.HORIZONTAL, label="Lower Saturation", command=update_image)
l_v_slider = tk.Scale(root, from_=0, to=255, orient=tk.HORIZONTAL, label="Lower Value", command=update_image)
u_h_slider = tk.Scale(root, from_=0, to=179, orient=tk.HORIZONTAL, label="Upper Hue", command=update_image)
u_s_slider = tk.Scale(root, from_=0, to=255, orient=tk.HORIZONTAL, label="Upper Saturation", command=update_image)
u_v_slider = tk.Scale(root, from_=0, to=255, orient=tk.HORIZONTAL, label="Upper Value", command=update_image)

# Create a label to display the image
image_label = tk.Label(root)
image_label.pack()

# Pack the sliders
l_h_slider.pack()
l_s_slider.pack()
l_v_slider.pack()
u_h_slider.pack()
u_s_slider.pack()
u_v_slider.pack()

# Call the update_image function initially
update_image(0)

# Start the Tkinter event loop
root.mainloop()