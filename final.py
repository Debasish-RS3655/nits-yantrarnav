import os
import cv2
import numpy as np
import pandas as pd
from scipy.stats import skew, kurtosis

# ---------------------- Prepare Image ----------------------
def prepare_image(image_path, target_size=(64, 64)):
    """Load, convert to grayscale, and resize image."""
    img = cv2.imread(image_path)
    if img is None:
        print(f"⚠️ Error: Unable to load image {image_path}")
        return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.resize(gray, target_size)

# ---------------------- Compute FFT & Spectral Features ----------------------
def compute_fft(image):
    """Compute FFT magnitude spectrum."""
    fft_image = cv2.dft(np.float32(image), flags=cv2.DFT_COMPLEX_OUTPUT)
    fft_shifted = np.fft.fftshift(fft_image)
    magnitude_spectrum = 20 * np.log(cv2.magnitude(fft_shifted[:, :, 0], fft_shifted[:, :, 1]) + 1)
    return magnitude_spectrum

def ms_fdma_feature(spectrum):
    return np.mean(spectrum)

def ms_symmetry_feature(spectrum):
    center = spectrum.shape[0] // 2
    upper, lower = spectrum[:center, :], np.flipud(spectrum[center:, :])
    sym_x = np.mean(np.abs(upper - lower))

    left, right = spectrum[:, :center], np.fliplr(spectrum[:, center:])
    sym_y = np.mean(np.abs(left - right))

    return sym_x, sym_y

def ms_amplitude_moment(spectrum):
    spectrum_flat = spectrum.flatten()
    return np.mean(spectrum_flat), np.var(spectrum_flat), skew(spectrum_flat), kurtosis(spectrum_flat)

# ---------------------- MGG Feature Extraction ----------------------
def extract_mgg_features(image, num_scales=2, num_thresholds=2, base_dg=2):
    """Extracts 4 MGG features (2 scales × 2 thresholds)."""
    Pg_feature_vector = []
    h, w = image.shape

    for scale_index in range(num_scales):
        dg_scale = base_dg * (scale_index + 1)
        gu, gv = image[:, 1:] - image[:, :-1], image[1:, :] - image[:-1, :]
        gu, gv = np.pad(gu, [(0, 0), (0, 1)], mode='constant'), np.pad(gv, [(0, 1), (0, 0)], mode='constant')
        g = np.sqrt(gu**2 + gv**2)

        for j in range(1, num_thresholds + 1):
            thgj = j * dg_scale
            Pg_feature_vector.append(np.sum(g > thgj) / (h * w))

    return np.array(Pg_feature_vector)

# ---------------------- MSGEF Feature Extraction ----------------------
def extract_multiscale_features(image, window_sizes=[5, 10]):
    """Extracts 2 MSGEF features using edge detection."""
    edges = cv2.Canny(cv2.equalizeHist(image), 10, 50)
    feature_vectors = []

    for window_size in window_sizes:
        step = window_size // 5
        features = []

        for u in range(0, image.shape[0] - window_size, step):
            for v in range(0, image.shape[1] - window_size, step):
                window = edges[u:u+window_size, v:v+window_size]
                edge_strength = np.sum(window) / (window_size * window_size)
                features.append(edge_strength)

        feature_vectors.append(np.mean(features))

    return np.array(feature_vectors)

# ---------------------- Process Images and Save to CSV ----------------------
def process_images_and_save_to_csv(image_paths, labels, output_csv):
    """Extract features, add class labels, and save to CSV."""
    feature_data = []
    column_names = ["Filename", "MFDMA", "Symmetry_X", "Symmetry_Y", "Amp_Mean", "Amp_Variance", 
                    "Amp_Skewness", "Amp_Kurtosis", "msgggf_1", "msgggf_2", "msgggf_3", "msgggf_4", 
                    "msgef_1", "msgef_2", "Class"]

    for image_path, label in zip(image_paths, labels):
        print(f"Processing: {image_path}")
        grayscale_image = prepare_image(image_path)
        if grayscale_image is None:
            continue

        # Extract Features
        fft_magnitude = compute_fft(grayscale_image)
        fdma = ms_fdma_feature(fft_magnitude)
        sym_x, sym_y = ms_symmetry_feature(fft_magnitude)
        amp_mean, amp_variance, amp_skewness, amp_kurtosis = ms_amplitude_moment(fft_magnitude)
        msggf_features = extract_mgg_features(grayscale_image)  # Extract 4 features
        msgef_features = extract_multiscale_features(grayscale_image)  # Extract 2 features

        row = [os.path.basename(image_path), fdma, sym_x, sym_y, amp_mean, amp_variance, amp_skewness, 
               amp_kurtosis] + list(msggf_features) + list(msgef_features) + [label]
        feature_data.append(row)

    # Save to CSV
    feature_df = pd.DataFrame(feature_data, columns=column_names)
    feature_df.to_csv(output_csv, index=False)
    print(f"\n✅ Features saved to: {output_csv}")

# ---------------------- Run Extraction ----------------------
if __name__ == '__main__':
    dataset_root = "Dataset"  # Change this to your dataset root path
    output_csv = "FINAL_RESULT.csv"

    class_folders = {
        "Hard_surfaces": "hard",
        "Sandy": "sandy",
        "Rocky": "rocky"
    }

    image_files = []
    labels = []

    # Iterate through each class folder and collect images
    for folder_name, class_label in class_folders.items():
        folder_path = os.path.join(dataset_root, folder_name)
        if not os.path.exists(folder_path):
            print(f"⚠️ Warning: Folder '{folder_path}' not found, skipping...")
            continue

        for root, _, files in os.walk(folder_path):
            for file in files:
                if file.lower().endswith((".jpg", ".jpeg", ".png")):
                    image_files.append(os.path.join(root, file))
                    labels.append(class_label)

    if not image_files:
        print("⚠️ No images found! Please check your dataset folder.")
    else:
        print(f"✅ Found {len(image_files)} images. Starting extraction...")
        process_images_and_save_to_csv(image_files, labels, output_csv)
