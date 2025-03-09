import cv2
import numpy as np

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

if __name__ == '__main__':
    # Example Usage:
    print("Example Usage of standalone extract_mgg_features_standalone function (with base_dg=2):\n")

    # 1. Create a dummy grayscale patch (replace with your actual patch loading if needed)
    dummy_patch = np.array([[100, 120, 150, 150],
                           [110, 130, 160, 170],
                           [140, 160, 190, 200],
                           [140, 160, 190, 200]], dtype=np.uint8)
    print("Example Input Patch (4x4 grayscale):\n", dummy_patch)

    # 2. Extract MGG features with default parameters (now base_dg=2)
    mgg_features = extract_mgg_features_standalone(dummy_patch)

    # 3. Print the extracted features
    print("\nExtracted MGG Features (default parameters, base_dg=2):")
    print(mgg_features)
    print("\nFeature vector shape:", mgg_features.shape)
    print("\nBy default, the feature vector length is 30 (3 scales * 10 thresholds).\n")

    # 4. Example with different parameters:
    mgg_features_custom = extract_mgg_features_standalone(dummy_patch, num_scales=2, num_thresholds=5, base_dg=5) # base_dg changed to 5
    print("Extracted MGG Features (custom parameters - 2 scales, 5 thresholds, base_dg=5):")
    print(mgg_features_custom)
    print("\nFeature vector shape (custom):", mgg_features_custom.shape)
    print("\nWith custom parameters, the feature vector length is now 10 (2 scales * 5 thresholds).")