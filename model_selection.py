import pandas as pd
from lazypredict.Supervised import LazyClassifier
from sklearn.model_selection import train_test_split

# Load the dataset
file_path = "FINAL_RESULT.csv"  # Update the correct path if needed
df = pd.read_csv(file_path)

# Drop the 'file name' column if it exists
if "Filename" in df.columns:
    df.drop(columns=["Filename"], inplace=True)


class_mapping = {"hard": 0, "sandy": 1, "rocky": 2}  # Updated third category
df["Class"] = df["Class"].map(class_mapping)

# Print class encoding mapping
print("Class Encoding Mapping:", class_mapping)

# Split data into features and target
X = df.drop(columns=["Class"])
y = df["Class"]

# Split into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

# Apply LazyPredict
clf = LazyClassifier()
models, predictions = clf.fit(X_train, X_test, y_train, y_test)

# Display model performance
print(models)
