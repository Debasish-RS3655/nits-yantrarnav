import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN, KMeans
from sklearn.mixture import GaussianMixture
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.neighbors import KNeighborsClassifier
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score, classification_report

# Load the dataset
file_path = "extracted_features (2).csv"
df = pd.read_csv(file_path)

# Drop the non-numeric column if exists
data = df.select_dtypes(include=[np.number])

# Standardize the data
scaler = StandardScaler()
data_scaled = scaler.fit_transform(data)

# Apply PCA for dimensionality reduction
pca = PCA(n_components=2)
data_pca = pca.fit_transform(data_scaled)

# Apply Gaussian Mixture Model (GMM) for clustering
gmm = GaussianMixture(n_components=3, random_state=42)
df["GMM_Cluster"] = gmm.fit_predict(data_pca)

# Prepare data for classification
X_train, X_test, y_train, y_test = train_test_split(data_pca, df["GMM_Cluster"], test_size=0.2, random_state=42)

# Train KNN classifier
knn = KNeighborsClassifier(n_neighbors=5)
knn.fit(X_train, y_train)

# Predict and evaluate KNN
y_pred_knn = knn.predict(X_test)
accuracy_knn = accuracy_score(y_test, y_pred_knn)
print("KNN Classification Accuracy:", accuracy_knn)
print("KNN Classification Report:\n", classification_report(y_test, y_pred_knn))

# Train Random Forest classifier
rf = RandomForestClassifier(n_estimators=100, random_state=42)
rf.fit(X_train, y_train)

# Predict and evaluate Random Forest
y_pred_rf = rf.predict(X_test)
accuracy_rf = accuracy_score(y_test, y_pred_rf)
print("Random Forest Classification Accuracy:", accuracy_rf)
print("Random Forest Classification Report:\n", classification_report(y_test, y_pred_rf))

# Visualizing classification results
plt.figure(figsize=(8, 6))
plt.scatter(data_pca[:, 0], data_pca[:, 1], c=df["GMM_Cluster"], cmap='coolwarm', alpha=0.5)
plt.xlabel("Principal Component 1")
plt.ylabel("Principal Component 2")
plt.title("Classification Results after PCA")
plt.colorbar(label="Cluster")
plt.show()

# Save the classified data
df.to_csv("final_output.csv", index=False)

print("Classification complete using KNN and Random Forest with PCA. The output file is saved as 'classified_data.csv'.")
