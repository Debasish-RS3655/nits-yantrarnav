import cv2
import numpy as np
import os
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------- 1. Load & Prepare Image ----------------------
def prepare_image(image_path):
    """
    Loads an image, converts it to grayscale, and returns it as a NumPy array.
    """
    img_bgr = cv2.imread(image_path)
    
    if img_bgr is None:
        print(f"Error: Could not load image from {image_path}")
        return None

    grayscale_img = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    return grayscale_img

# ---------------------- 2. MGG Feature Extraction ----------------------
def extract_mgg_features_standalone(patch, num_scales=2, num_thresholds=5, base_dg=5):
    """
    Extracts Multiscale Gray Gradient-Grade (MGG) features from an image patch.
    """
    Pg_feature_vector = []
    patch_height, patch_width = patch.shape

    for scale_index in range(num_scales):
        dg_scale = base_dg * (scale_index + 1) 

        gu = patch[:, 1:] - patch[:, :-1]  
        gv = patch[1:, :] - patch[:-1, :]  

        gu = np.pad(gu, [(0, 0), (0, 1)], mode='constant')  
        gv = np.pad(gv, [(0, 1), (0, 0)], mode='constant')  

        g = np.sqrt(gu**2 + gv**2)

        P_g_scale = []  
        for j in range(1, num_thresholds + 1):
            thgj = j * dg_scale
            Ngj = np.sum(g > thgj)  
            pgj = Ngj / (patch_height * patch_width)  
            P_g_scale.append(pgj)

        Pg_feature_vector.extend(P_g_scale) 

    return np.array(Pg_feature_vector)

# ---------------------- 3. Canny Edge-Based Feature Extraction ----------------------
def extract_edges(image, low_threshold=10, high_threshold=50):
    """Apply Histogram Equalization and Canny edge detection."""
    image = cv2.equalizeHist(image)  
    blurred = cv2.GaussianBlur(image, (3,3), 0)  
    edges = cv2.Canny(blurred, low_threshold, high_threshold)
    return edges

def compute_edge_strength_distribution(edges, num_levels=10):
    """Compute pixel proportion for different edge strength levels."""
    edge_pixels = edges[edges > 0]  
    max_grad = np.max(edge_pixels) if edge_pixels.size > 0 else 1
    thresholds = np.linspace(0, max_grad, num_levels + 1)

    edge_strength_counts = np.histogram(edge_pixels, bins=thresholds)[0]
    total_pixels = edges.size

    pixel_proportions = edge_strength_counts / total_pixels
    return pixel_proportions

def extract_multiscale_features(image, window_sizes=[5, 10]):
    """Extract edge-based features using multiple window sizes."""
    edges = extract_edges(image)
    feature_vectors = []

    for window_size in window_sizes:
        step = window_size // 5
        features = []

        for u in range(0, image.shape[0] - window_size, step):
            for v in range(0, image.shape[1] - window_size, step):
                window = edges[u:u+window_size, v:v+window_size]
                feature_vector = compute_edge_strength_distribution(window)
                features.append(feature_vector)

        feature_vectors.append(np.mean(features, axis=0))  

    full_vector = np.concatenate(feature_vectors)  

    if len(full_vector) > 10:
        reduced_vector = np.sort(full_vector)[-10:]  
    else:
        reduced_vector = np.pad(full_vector, (0, 10 - len(full_vector)), 'constant')  

    return reduced_vector

# ---------------------- 4. Process Images and Save Features to CSV ----------------------
def process_images_and_save_to_csv(image_paths, output_csv="features.csv"):
    """Extracts features from images and saves them to a CSV file."""
    feature_data = []

    for image_path in image_paths:
        print(f"Processing: {image_path}")
        grayscale_image = prepare_image(image_path)

        if grayscale_image is None:
            continue  

        # Extract Features
        mgg_features = extract_mgg_features_standalone(grayscale_image)
        mse_features = extract_multiscale_features(grayscale_image)

        # Combine all features into a single row
        combined_features = np.concatenate((mgg_features, mse_features))
        feature_data.append(combined_features)

    # Convert to DataFrame and save
    feature_df = pd.DataFrame(feature_data)
    feature_df.to_csv(output_csv, index=False)
    print(f"\nFeatures saved to {output_csv}")

# ---------------------- Run the Feature Extraction ----------------------
if __name__ == '__main__':
    image_dir = "msl-images/calibrated"  
    image_files = [os.path.join(image_dir, img) for img in os.listdir(image_dir) if img.endswith(".JPG")]

    if not image_files:
        print("No images found! Please check the directory.")
    else:
        process_images_and_save_to_csv(image_files, output_csv="extracted_features.csv")
