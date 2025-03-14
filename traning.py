import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.model_selection import train_test_split, RandomizedSearchCV, StratifiedKFold
from sklearn.ensemble import RandomForestClassifier, StackingClassifier, VotingClassifier
from xgboost import XGBClassifier
from sklearn.metrics import accuracy_score, confusion_matrix
from scipy.stats import randint, uniform
import warnings
warnings.filterwarnings("ignore")

file_path = "FINAL_RESULT.csv"
df = pd.read_csv(file_path)
if "Filename" in df.columns:
    df.drop(columns=["Filename"], inplace=True)
class_mapping = {"hard": 0, "sandy": 1, "rocky": 2}
df["Class"] = df["Class"].map(class_mapping)
print("Class Encoding Mapping:", class_mapping)
X = df.drop(columns=["Class"])
y = df["Class"]
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42, stratify=y)
skf = StratifiedKFold(n_splits=3, shuffle=True, random_state=42)

rf_params = {
    "n_estimators": randint(100, 500),
    "max_depth": [None, 10, 20, 30],
    "min_samples_split": randint(2, 10),
    "min_samples_leaf": randint(1, 5),
    "max_features": ["sqrt", "log2", None]
}
rf_random = RandomizedSearchCV(RandomForestClassifier(random_state=42), rf_params, n_iter=20, cv=skf, n_jobs=-1, verbose=1, random_state=42)
rf_random.fit(X_train, y_train)
rf_best = rf_random.best_estimator_
print("Best RandomForest Parameters:", rf_random.best_params_)
rf_preds = rf_best.predict(X_test)
rf_accuracy = accuracy_score(y_test, rf_preds)
print(f"Optimized Random Forest Accuracy: {rf_accuracy:.4f}")

xgb_params = {
    "n_estimators": randint(100, 500),
    "learning_rate": uniform(0.01, 0.19),
    "max_depth": randint(3, 10),
    "subsample": [0.8, 1],
    "colsample_bytree": [0.8, 1]
}
xgb_random = RandomizedSearchCV(XGBClassifier(use_label_encoder=False, eval_metric="mlogloss", random_state=42), xgb_params, n_iter=20, cv=skf, n_jobs=-1, verbose=1, random_state=42)
xgb_random.fit(X_train, y_train)
xgb_best = xgb_random.best_estimator_
print("Best XGB Parameters:", xgb_random.best_params_)
xgb_preds = xgb_best.predict(X_test)
xgb_accuracy = accuracy_score(y_test, xgb_preds)
print(f"Optimized XGBoost Accuracy: {xgb_accuracy:.4f}")

stacking_clf = StackingClassifier(
    estimators=[("RandomForest", rf_best), ("XGBoost", xgb_best)],
    final_estimator=XGBClassifier(use_label_encoder=False, eval_metric="mlogloss", random_state=42),
    cv=skf,
    n_jobs=-1
)
stacking_clf.fit(X_train, y_train)
stacking_preds = stacking_clf.predict(X_test)
stacking_accuracy = accuracy_score(y_test, stacking_preds)
print(f"Stacking Classifier Accuracy: {stacking_accuracy:.4f}")

voting_clf = VotingClassifier(
    estimators=[("RandomForest", rf_best), ("XGBoost", xgb_best)],
    voting="hard",
    n_jobs=-1
)
voting_clf.fit(X_train, y_train)
voting_preds = voting_clf.predict(X_test)
voting_accuracy = accuracy_score(y_test, voting_preds)
print(f"Voting Classifier Accuracy: {voting_accuracy:.4f}")

plt.figure(figsize=(10, 5))
rf_importances = rf_best.feature_importances_
sns.barplot(x=rf_importances, y=X.columns)
plt.xlabel("Importance Score")
plt.ylabel("Feature")
plt.title("Feature Importance - Random Forest")
plt.show()

plt.figure(figsize=(10, 5))
xgb_importances = xgb_best.feature_importances_
sns.barplot(x=xgb_importances, y=X.columns, color="blue")
plt.xlabel("Importance Score")
plt.ylabel("Feature")
plt.title("Feature Importance - XGBoost")
plt.show()

def plot_confusion_matrix(y_true, y_pred, title):
    cm = confusion_matrix(y_true, y_pred)
    plt.figure(figsize=(6, 5))
    sns.heatmap(cm, annot=True, cmap="Blues", fmt="d")
    plt.xlabel("Predicted")
    plt.ylabel("Actual")
    plt.title(title)
    plt.show()

plot_confusion_matrix(y_test, rf_preds, "Confusion Matrix - Random Forest")
plot_confusion_matrix(y_test, xgb_preds, "Confusion Matrix - XGBoost")
plot_confusion_matrix(y_test, stacking_preds, "Confusion Matrix - Stacking Classifier")
plot_confusion_matrix(y_test, voting_preds, "Confusion Matrix - Voting Classifier")

accuracies = {
    "Random Forest": rf_accuracy,
    "XGBoost": xgb_accuracy,
    "Stacking Classifier": stacking_accuracy,
    "Voting Classifier": voting_accuracy
}
best_classifier = max(accuracies, key=accuracies.get)
print(f"\nBest Classifier: {best_classifier} with Accuracy: {accuracies[best_classifier]:.4f}")
