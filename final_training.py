import pandas as pd
import numpy as np
from sklearn.model_selection import train_test_split, RandomizedSearchCV, StratifiedKFold
from xgboost import XGBClassifier
from sklearn.metrics import accuracy_score, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.stats import randint, uniform
import warnings
warnings.filterwarnings("ignore")

from joblib import dump

class xgboost:
    def __init__(self):
        pass
    

    def run_xgboost(self,file_path):
        df = pd.read_csv(file_path)
        if "Filename" in df.columns:
            df.drop(columns=["Filename"], inplace=True)
        class_mapping = {"hard": 0, "sandy": 1, "rocky": 2}
        df["Class"] = df["Class"].map(class_mapping)
        X = df.drop(columns=["Class"])
        y = df["Class"]
        
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=0.2, random_state=42, stratify=y
        )
        
        skf = StratifiedKFold(n_splits=3, shuffle=True, random_state=42)
        
        xgb_params = {
            "n_estimators": randint(100, 500),
            "learning_rate": uniform(0.01, 0.19),
            "max_depth": randint(3, 10),
            "subsample": [0.8, 1],
            "colsample_bytree": [0.8, 1]
        }
        
        xgb_random = RandomizedSearchCV(
            XGBClassifier(use_label_encoder=False, eval_metric="mlogloss", random_state=42),
            xgb_params,
            n_iter=20,
            cv=skf,
            n_jobs=-1,
            verbose=1,
            random_state=42
        )
        xgb_random.fit(X_train, y_train)
        xgb_best = xgb_random.best_estimator_
        print("Best XGB Parameters:", xgb_random.best_params_)
        
        # Dump the best model using joblib
        dump(xgb_best, "xgb_best_model.joblib")
        print("Model dumped as 'xgb_best_model.joblib'")
        
        xgb_preds = xgb_best.predict(X_test)
        accuracy = accuracy_score(y_test, xgb_preds)
        print("Optimized XGBoost Accuracy:", accuracy)
        
        cm = confusion_matrix(y_test, xgb_preds)
        plt.figure(figsize=(6, 5))
        sns.heatmap(cm, annot=True, cmap="Blues", fmt="d")
        plt.xlabel("Predicted")
        plt.ylabel("Actual")
        plt.title("Confusion Matrix - XGBoost")
        plt.show()
        
        plt.figure(figsize=(10, 5))
        sns.barplot(x=xgb_best.feature_importances_, y=X.columns, color="blue")
        plt.xlabel("Importance Score")
        plt.ylabel("Feature")
        plt.title("Feature Importance - XGBoost")
        plt.show()

if __name__ == "__main__":
    run = xgboost()
    run.run_xgboost("FINAL_RESULT.csv")
