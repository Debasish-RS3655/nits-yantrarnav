import pandas as pd

# Load the CSV file
input_csv = "FINAL_RESULT.csv"
output_csv = "SHUFFLED_FINAL_RESULT.csv"

# Read the dataset
df = pd.read_csv(input_csv)

# Shuffle the dataset
df_shuffled = df.sample(frac=1, random_state=42).reset_index(drop=True)

# Save the shuffled dataset
df_shuffled.to_csv(output_csv, index=False)

print(f"✅ Shuffled CSV saved to: {output_csv}")
