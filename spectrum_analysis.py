import os
import cv2
import numpy as np
import pandas as pd
from scipy.stats import skew, kurtosis

# Input and Output Directories
input_dir = "/Users/champakjyotikonwar/CODING/PROJECTS/MARS/calibrated"
output_dir = "/Users/champakjyotikonwar/CODING/PROJECTS/MARS/Result"
csv_output_path = "/Users/champakjyotikonwar/CODING/PROJECTS/MARS/spectrum_analysis_results.csv"

# Ensure output directory exists
os.makedirs(output_dir, exist_ok=True)

# List to store results
results = []


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


def extract_mgg_features_standalone(patch, num_scales=3, num_thresholds=10, base_dg=2):
    """
    Standalone function to extract Multiscale Gray Gradient-Grade (MGG) features from an image patch.
    (Modified base_dg to 2 for potentially finer gradient levels)
    """
    Pg_feature_vector = []
    patch_height, patch_width = patch.shape

    for scale_index in range(num_scales):
        dg_scale = base_dg * (scale_index + 1) # Define dg for the current scale

        # 1. Calculate Gradients (Equation 1)
        gu = patch[:, 1:] - patch[:, :-1]  # Horizontal gradient (valid for columns 0 to width-2)
        gv = patch[1:, :] - patch[:-1, :]  # Vertical gradient (valid for rows 0 to height-2)

        # Pad gradients to be the same size as the input patch for simplicity.
        # Zero padding at the right/bottom edge
        gu = np.pad(gu, [(0, 0), (0, 1)], mode='constant') # Pad right with 0s
        gv = np.pad(gv, [(0, 1), (0, 0)], mode='constant') # Pad bottom with 0s

        # 2. Gradient Magnitude (Equation 2)
        g = np.sqrt(gu**2 + gv**2)

        P_g_scale = [] # Proportions for the current scale
        for j in range(1, num_thresholds + 1):
            # 3. Thresholds (Equation 3)
            thgj = j * dg_scale

            # 4. Proportion of pixels exceeding threshold (Equation 4)
            Ngj = np.sum(g > thgj) # Count pixels where gradient magnitude > threshold
            pgj = Ngj / (patch_height * patch_width) # Proportion

            P_g_scale.append(pgj) # Append proportion for this threshold

        Pg_feature_vector.extend(P_g_scale) # Add proportions for this scale to the feature vector

    return np.array(Pg_feature_vector)


# Function to Convert Image to Grayscale and Resize to 64x64
def convert_to_grayscale_and_save(input_path, output_path, target_size=(64, 64)):
    img = cv2.imread(input_path)
    if img is None:
        print(f"Error: Unable to load image {input_path}")
        return None

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Resize to 64x64
    gray_resized = cv2.resize(gray, target_size)

    cv2.imwrite(output_path, gray_resized)
    return gray_resized

# Compute Fourier Transform (FFT)
def compute_fft(image):
    fft_image = cv2.dft(np.float32(image), flags=cv2.DFT_COMPLEX_OUTPUT)
    fft_shifted = np.fft.fftshift(fft_image)
    magnitude_spectrum = 20 * np.log(cv2.magnitude(fft_shifted[:, :, 0], fft_shifted[:, :, 1]) + 1)
    return magnitude_spectrum

# Feature 1: FDMA Feature (Mean of Spectrum)
def ms_fdma_feature(spectrum):
    return np.mean(spectrum)

# Feature 2: Symmetry Feature (X & Y)
def ms_symmetry_feature(spectrum):
    center = spectrum.shape[0] // 2
    upper = spectrum[:center, :]
    lower = np.flipud(spectrum[center:, :])
    sym_x = np.mean(np.abs(upper - lower))
    
    left = spectrum[:, :center]
    right = np.fliplr(spectrum[:, center:])
    sym_y = np.mean(np.abs(left - right))
    
    return sym_x, sym_y

# Feature 3: Amplitude Moments (Mean, Variance, Skewness, Kurtosis)
def ms_amplitude_moment(spectrum):
    spectrum_flat = spectrum.flatten()  # Flatten to 1D array

    mean = np.mean(spectrum_flat)
    variance = np.var(spectrum_flat)
    skewness = skew(spectrum_flat)  # Measures asymmetry
    kurt = kurtosis(spectrum_flat)  # Measures sharp variations
    
    return mean, variance, skewness, kurt

# Process All Images in Input Directory
for filename in os.listdir(input_dir):
    if filename.lower().endswith((".jpg", ".jpeg", ".png")):  # Process only image files
        input_path = os.path.join(input_dir, filename)
        output_path = os.path.join(output_dir, filename)

        try:
            # Convert to Grayscale and Resize
            gray_image = convert_to_grayscale_and_save(input_path, output_path)
            if gray_image is None:
                continue  # Skip if image could not be processed

            # Compute FFT
            fft_magnitude = compute_fft(gray_image)


            # Compute Spectrum Analysis Features
            fdma = ms_fdma_feature(fft_magnitude)
            sym_x, sym_y = ms_symmetry_feature(fft_magnitude)
            amp_mean, amp_variance, amp_skewness, amp_kurtosis = ms_amplitude_moment(fft_magnitude)

            

            # Save Results
            results.append([filename, fdma, sym_x, sym_y, amp_mean, amp_variance, amp_skewness, amp_kurtosis])
            print(f"Processed {filename}: FDMA = {fdma:.4f}, Sym_X = {sym_x:.4f}, Sym_Y = {sym_y:.4f}, Amp_Mean = {amp_mean:.4f}, Amp_Var = {amp_variance:.4f}, Skewness = {amp_skewness:.4f}, Kurtosis = {amp_kurtosis:.4f}")

        except Exception as e:
            print(f"Error processing {filename}: {e}")




# Save results to CSV
df = pd.DataFrame(results, columns=["Filename", "MFDMA", "Symmetry_X", "Symmetry_Y", "Amp_Mean", "Amp_Variance", "Amp_Skewness", "Amp_Kurtosis"])
df.to_csv(csv_output_path, index=False)

print(f"\nProcessing completed. Results saved to: {csv_output_path}")
