import os

def filter_land_images_dataset(synset_file_path, train_file_path, val_file_path, test_file_path, output_dir):
    """
    Filters the Mars rover image dataset to keep only land/terrain images and updates label files.

    Args:
        synset_file_path (str): Path to 'msl_synset_words-indexed.txt' file.
        train_file_path (str): Path to 'train-calibrated-shuffled.txt' file.
        val_file_path (str): Path to 'val-calibrated-shuffled.txt' file.
        test_file_path (str): Path to 'test-calibrated-shuffled.txt' file.
        output_dir (str): Directory to save the filtered label files.
    """

    # ==========================================================================
    # ==>>  USER: MODIFY THIS LIST BELOW TO UPDATE NON-LAND LABELS  <<==
    # ==========================================================================
    non_land_label_indices = [
        1, 2, 12, 14,  # Calibration targets
        0, 3, 6, 7, 10, 11, 13, 15, 16, 17, 18, 19, 20, 23, 24, # Rover parts/instruments
        22, # Sun
        9   # horizon - Potentially remove horizon as well, as it's not direct terrain view
    ]
    # ==========================================================================
    # ==>>  END OF USER-MODIFIABLE SECTION  <<==
    # ==========================================================================


    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Function to process a single label file (no changes needed here usually)
    def process_label_file(input_file_path, output_file_path):
        filtered_lines = []
        with open(input_file_path, 'r') as infile:
            for line in infile:
                parts = line.strip().split()
                if not parts:  # Skip empty lines
                    continue
                image_filename = parts[0]
                label_index = int(parts[1])

                if label_index not in non_land_label_indices:
                    filtered_lines.append(line.strip())

        with open(output_file_path, 'w') as outfile:
            for line in filtered_lines:
                outfile.write(line + '\n')
        print(f"Filtered labels saved to: {output_file_path}")

    # Process each label file
    print("Filtering training labels...")
    process_label_file(train_file_path, os.path.join(output_dir, "train-land-images.txt"))
    print("Filtering validation labels...")
    process_label_file(val_file_path, os.path.join(output_dir, "val-land-images.txt"))
    print("Filtering test labels...")
    process_label_file(test_file_path, os.path.join(output_dir, "test-land-images.txt"))

    print("Filtering process complete. New label files created in:", output_dir)

if __name__ == "__main__":
    # **--- USER MUST UPDATE THESE PATHS - USE RAW STRINGS (r'...') ---**
    synset_file = r"C:\Users\Yuvraj\OneDrive\Files\Work\MARS\msl-images\msl_synset_words-indexed.txt"
    train_labels_file = r"C:\Users\Yuvraj\OneDrive\Files\Work\MARS\msl-images\train-calibrated-shuffled.txt"
    val_labels_file = r"C:\Users\Yuvraj\OneDrive\Files\Work\MARS\msl-images\val-calibrated-shuffled.txt"
    test_labels_file = r"C:\Users\Yuvraj\OneDrive\Files\Work\MARS\msl-images\test-calibrated-shuffled.txt"
    output_directory = r"C:\Users\Yuvraj\OneDrive\Files\Work\MARS\msl-images\filtered_labels" # Directory to save filtered labels

    filter_land_images_dataset(synset_file, train_labels_file, val_labels_file, test_labels_file, output_directory)

    print("\n**Remember to update your data loading code to use the new filtered label files**")
    print(f"  e.g., when calling `load_images_and_labels`, point it to files in '{output_directory}'")