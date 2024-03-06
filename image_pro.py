import cv2
import numpy as np
import matplotlib.pyplot as plt

class ImageProcessor:
    def __init__(self, bgr_image):
        self.img = bgr_image.copy()
        self.org_img = self.img.copy()
        self.gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.cropped_objects = []
        self.processing_steps_images = []
        self.processing_steps_labels = ["Original Image", "Mask Image", "Distance Transform", "Foreground Image"]
        self.all_cnts = []  # Store detected contours
        self.cropped_objects_info = []  # Store cropped objects info
        
    def create_mask(self):
        lower = np.array([70, 70, 70])
        upper = np.array([255, 255, 255])
        mask = cv2.bitwise_not(cv2.inRange(self.img, lower, upper))

        # Filling holes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            cv2.drawContours(mask, [contour], 0, 255, -1)  # 255 is color, -1 means fill the contour  
        return mask
    
    def resize_with_padding(self, img, target_size=(128, 128), fill_value=0):
        """
        Resize the input image to the target size, maintaining the original aspect ratio by padding.
        """
        h, w = img.shape[:2]
        ratio = min(target_size[0] / h, target_size[1] / w)
        new_size = (int(w * ratio), int(h * ratio))
        resized_img = cv2.resize(img, new_size)

        delta_w = target_size[1] - new_size[0]
        delta_h = target_size[0] - new_size[1]
        top, bottom = delta_h // 2, delta_h - (delta_h // 2)
        left, right = delta_w // 2, delta_w - (delta_w // 2)

        padded_img = cv2.copyMakeBorder(resized_img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=fill_value)
        return padded_img
    
    def categorize_ripeness(self, image):
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        
        red_lower = np.array([100, 0, 0])
        red_upper = np.array([255, 100, 100])
        green_lower = np.array([0, 100, 0])
        green_upper = np.array([100, 255, 100])

        red_mask = cv2.inRange(image_rgb, red_lower, red_upper)
        green_mask = cv2.inRange(image_rgb, green_lower, green_upper)

        red_area = np.sum(red_mask > 0)
        # green_area = np.sum(green_mask > 0)
        total_area = image.shape[0] * image.shape[1]
        red_percentage = (red_area / total_area) * 100

        if red_percentage >= 90:
            return "fully_ripened"
        elif 30 <= red_percentage < 90:
            return "half_ripened"
        else:
            return "green"

    def display_results(self):
        for i, cropped in enumerate(self.cropped_objects):
            plt.subplot(1, len(self.cropped_objects), i + 1)
            plt.imshow(cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB))
            plt.title(f"Cropped Object {i + 1}")
            plt.axis('off')
        plt.show()
    
    def display_masks(self):
        fig, ax = plt.subplots(1, len(self.processing_steps_images), figsize=(20, 20))
        for i, img in enumerate(self.processing_steps_images):
            if i == 0:
                ax[i].imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            else:
                ax[i].imshow(img, cmap='gray')
            ax[i].set_title(self.processing_steps_labels[i])
            ax[i].axis('off')

    def apply_watershed(self):
        self.processing_steps_images.append(self.img)
        thresh = self.create_mask()
        self.processing_steps_images.append(thresh)

        sure_bg = cv2.dilate(thresh, np.ones((5, 5), np.uint8), iterations=1)

        dist_transform = cv2.distanceTransform(thresh, cv2.DIST_L2, 5)
        _, sure_fg = cv2.threshold(dist_transform, 0.4 * dist_transform.max(), 255, 0)
        self.processing_steps_images.append(dist_transform)
        self.processing_steps_images.append(sure_fg)


        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg, sure_fg)

        _, markers = cv2.connectedComponents(sure_fg)
        markers = markers + 1
        markers[unknown == 255] = 0
        cv2.watershed(self.img, markers)
        self.img[markers == -1] = [0, 0, 255]
        

        image_area = self.org_img.shape[0] * self.org_img.shape[1]
        max_allowed_area = image_area * 0.9  # 90% of the image area

        for label in np.unique(markers):
            if label <= 1: 
                continue
            mask = np.zeros(self.gray.shape, dtype="uint8")
            mask[markers == label] = 255
            cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                x, y, w, h = cv2.boundingRect(c)
                area = w * h

                # Skip the object if it covers more than 90% of the image
                if area > max_allowed_area:
                    continue

                cropped = self.org_img[y:y+h, x:x+w]
                cropped_resized = self.resize_with_padding(cropped)
                center_x = x + w // 2
                center_y = y + h // 2
                ripeness_cat = self.categorize_ripeness(cropped)

                cropped_info = {
                    'cropped_image': cropped_resized,
                    'center_coordinates': (center_x, center_y),
                    'bounding_box': (x, y, w, h),
                    'cat': ripeness_cat  
                }
                self.cropped_objects_info.append(cropped_info)
                self.cropped_objects.append(cropped_resized)  
                self.all_cnts.append(cnts)
    
    def draw_rectangles(self):
        self.apply_watershed()
        display_img = self.org_img.copy()
        cv2.circle(display_img, (0, 0), radius=50, color=(0, 255, 0), thickness=-1)
        for objects in self.cropped_objects_info:
            x, y, w, h = objects['bounding_box']
            cv2.rectangle(display_img, (x, y), (x + w, y + h), (0, 255, 0), 5)
            center_x, center_y = objects['center_coordinates']
            category = objects['cat']
            
            cv2.circle(display_img, (center_x, center_y), radius=20, color=(0, 255, 0), thickness=-1)
            cv2.putText(display_img, f"({center_x}, {center_y})", (center_x + 5, center_y + 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 12)
            cv2.putText(display_img, f"{category}", (center_x + 5, center_y + 5 + 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 12)

        return display_img
    
    def detect_objects(self):
        self.apply_watershed()
        object_centers = []
        for object_info in self.cropped_objects_info:
            object_centers.append(object_info['center_coordinates'])
        
        return object_centers

