import imgaug.augmenters as iaa
import cv2
import os
import numpy as np

# Define an advanced augmentation sequence (without flipping & rotation)
augmenters = iaa.Sequential([
    iaa.Sharpen(alpha=(0.2, 0.8), lightness=(0.75, 1.25)),  # Sharpen the image
    iaa.AdditiveGaussianNoise(scale=(0, 0.05 * 255)),  # Add noise
    iaa.GammaContrast((0.8, 1.2)),  # Adjust contrast
    iaa.MultiplyHueAndSaturation((0.8, 1.2)),  # Randomly adjust hue & saturation
    iaa.ElasticTransformation(alpha=(0, 5.0), sigma=0.25),  # Slight elastic deformation
    iaa.Crop(percent=(0, 0.05)),  # Randomly crop up to 5% of the image
    iaa.GaussianBlur(sigma=(0.0, 1.5))  # Apply slight blur
])

# Paths
input_folder = "Hard surfaces"
output_folder = "augmented_hardsurface"

os.makedirs(output_folder, exist_ok=True)

# Load all images in one batch for efficiency
image_filenames = os.listdir(input_folder)
images = [cv2.imread(os.path.join(input_folder, img)) for img in image_filenames]

# Ensure images are not None (handling grayscale images)
images = [img for img in images if img is not None]

# Augment each image 5 times
for idx, img in enumerate(images):
    img_name = image_filenames[idx]
    
    for i in range(2):  # Generate 5 augmented versions
        augmented_img = augmenters(image=img)
        
        # Save with a unique name
        cv2.imwrite(os.path.join(output_folder, f"{img_name.split('.')[0]}_aug{i}.jpg"), augmented_img)

print(f"✅ Augmentation completed! {len(images) * 5} images generated.")