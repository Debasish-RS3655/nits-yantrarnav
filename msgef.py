import cv2
import numpy as np
import matplotlib.pyplot as plt

def extract_edges(image, low_threshold=10, high_threshold=50):
    """Apply Histogram Equalization and Canny edge detection."""
    image = cv2.equalizeHist(image)  # Enhances contrast
    blurred = cv2.GaussianBlur(image, (3,3), 0)  # Reduce noise
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
    """Extract features using multiple window sizes and reduce to 10 values."""
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

        feature_vectors.append(np.mean(features, axis=0))  # Average per window

    full_vector = np.concatenate(feature_vectors)  # Full feature vector

    # **Reduce to 10 values using simple feature selection**
    if len(full_vector) > 10:
        reduced_vector = np.sort(full_vector)[-10:]  # Top 10 highest values
    else:
        reduced_vector = np.pad(full_vector, (0, 10 - len(full_vector)), 'constant')  # Pad if less than 10

    return reduced_vector

# Load the image
image_path = r"msl-images\calibrated\0003ML0000000520100072E01_DRCL.JPG"
image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

# Extract feature vector (now exactly 10 values)
feature_vector = extract_multiscale_features(image)
print("Extracted Feature Vector (10 values):", feature_vector * 200)

# Debugging: Show images
plt.figure(figsize=(10,5))
plt.subplot(1,2,1)
plt.imshow(image, cmap='gray')
plt.title("Original Image")

plt.subplot(1,2,2)
plt.imshow(extract_edges(image), cmap='gray')
plt.title("Enhanced Canny Edge Detection")

plt.show()
